"""ROS 2 interface of the warehouse.

Runs inside the Isaac Sim process (rclpy from /opt/ros/jazzy, same Python 3.12 as Isaac Sim 6)
and is driven from the main loop: `want_render()` before a physics step, `after_step()` after it.

Out (frame `map` = the zone frame, metres; stamps are simulation time):
  /clock                                   rosgraph_msgs/Clock
  /tf, /tf_static                          robots' base_link frames, arm bases, people
  /carter{1,2,3}/odom, /forklift{1,2}/odom nav_msgs/Odometry
  /forklift{1,2}/lift                      std_msgs/Float64  (fork lift height, m)
  /ur5/joint_states, /franka/joint_states  sensor_msgs/JointState
  /warehouse/people, /warehouse/packages   geometry_msgs/PoseArray
  /warehouse/markers                       visualization_msgs/MarkerArray (RViz / Foxglove)
  /map                                     nav_msgs/OccupancyGrid (latched)
  /warehouse/layout                        std_msgs/String JSON (latched)  - static geometry
  /warehouse/state                         std_msgs/String JSON (5 Hz)     - live state for the web
  /warehouse/events                        std_msgs/String                 - one line per event
  /warehouse/events_recent                 std_msgs/String JSON (latched)  - last 150 event lines (backfill)
  /warehouse/camera/{overview,follow}/compressed   sensor_msgs/CompressedImage (jpeg)
In:
  /warehouse/orders               std_msgs/String  {"line": "A", "qty": 1, "id": "ORD-0001"}
  /warehouse/camera/follow_target std_msgs/String  "C1" | "F2" | "ur5" | "person1" | "overview"
  /carter{N}/cmd_vel              (not consumed: carters are driven by the fleet planner)
  /warehouse/pause, /warehouse/resume   std_srvs/Trigger
  /warehouse/estop                      std_srvs/SetBool
"""
import io, json, math, time
import numpy as np
import config as C

ROS_NAME = {"C1": "carter1", "C2": "carter2", "C3": "carter3", "F1": "forklift1", "F2": "forklift2"}
ARM_NAME = {"A": "ur5", "B": "franka"}


def _yaw_q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RosBridge:
    def __init__(self, sim, people, hw, arms, stage, building=None, cameras=True, cam_hz=4.0,
                 cam_res=(1280, 720), jpeg_q=75):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String, Float64
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import JointState, CompressedImage
        from geometry_msgs.msg import PoseArray, Pose, TransformStamped
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        from visualization_msgs.msg import Marker, MarkerArray
        from std_srvs.srv import Trigger, SetBool
        self.M = dict(String=String, Float64=Float64, Odometry=Odometry, OccupancyGrid=OccupancyGrid,
                      JointState=JointState, CompressedImage=CompressedImage, PoseArray=PoseArray, Pose=Pose,
                      TransformStamped=TransformStamped, TFMessage=TFMessage, Clock=Clock, Marker=Marker,
                      MarkerArray=MarkerArray)
        self.sim, self.people, self.hw, self.arms, self.stage = sim, people, hw, arms, stage
        self.building = building
        if not rclpy.ok():
            rclpy.init(args=[])
        self.rclpy = rclpy
        n = self.node = rclpy.create_node("warehouse_sim")
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        one = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.p_clock = n.create_publisher(Clock, "/clock", rel)
        self.p_tf = n.create_publisher(TFMessage, "/tf", rel)
        self.p_tf_static = n.create_publisher(TFMessage, "/tf_static", latched)
        self.p_odom = {k: n.create_publisher(Odometry, f"/{v}/odom", rel) for k, v in ROS_NAME.items()}
        self.p_lift = {k: n.create_publisher(Float64, f"/{ROS_NAME[k]}/lift", rel) for k in sim.forks}
        self.p_js = {t: n.create_publisher(JointState, f"/{ARM_NAME[t]}/joint_states", rel) for t in arms}
        self.p_people = n.create_publisher(PoseArray, "/warehouse/people", rel)
        self.p_pkgs = n.create_publisher(PoseArray, "/warehouse/packages", rel)
        self.p_markers = n.create_publisher(MarkerArray, "/warehouse/markers", one)
        self.p_map = n.create_publisher(OccupancyGrid, "/map", latched)
        self.p_layout = n.create_publisher(String, "/warehouse/layout", latched)
        self.p_state = n.create_publisher(String, "/warehouse/state", one)
        self.p_events = n.create_publisher(String, "/warehouse/events", QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.p_events_recent = n.create_publisher(String, "/warehouse/events_recent", latched)
        self._recent, self._recent_dirty = [], False
        self.p_cam = {k: n.create_publisher(CompressedImage, f"/warehouse/camera/{k}/compressed", one)
                      for k in ("overview", "follow")}

        n.create_subscription(String, "/warehouse/orders", self._on_order, rel)
        n.create_subscription(String, "/warehouse/camera/follow_target", self._on_follow, rel)
        n.create_service(Trigger, "/warehouse/pause", self._srv_pause)
        n.create_service(Trigger, "/warehouse/resume", self._srv_resume)
        n.create_service(SetBool, "/warehouse/estop", self._srv_estop)

        self.paused = False
        self.rtf = 0.0
        self.t_phys = 0.0
        self.step_i = 0
        self.vacuum = {t: False for t in arms}
        self.follow = "C1"
        self._follow_eye = None
        self._prev_fork = {k: (f.x, f.y) for k, f in sim.forks.items()}
        self._delivered_t = []                 # sim times of deliveries (throughput window)
        sim.listeners.append(self._on_event)

        self.layout = self._layout()
        self.p_layout.publish(String(data=json.dumps(self.layout)))
        self.p_map.publish(self._map())
        self._publish_static_tf()

        self.cameras = None
        if cameras:
            try:
                self.cameras = _Cameras(stage, cam_res, jpeg_q)
                self.cam_every = max(1, int(round(1.0 / (cam_hz * C_DT()))))
            except Exception as e:
                n.get_logger().error(f"cameras disabled: {e}")
                print(f"[ros] cameras disabled: {e}", flush=True)
        n.get_logger().info("warehouse_sim up: state, layout, map, odom, joint_states, cameras, orders, estop")
        print(f"[ros] node /warehouse_sim up (ROS_DOMAIN_ID from the environment), cameras "
              f"{'on' if self.cameras else 'off'}", flush=True)

    # ------------------------------------------------------------------ loop hooks
    def want_render(self):
        return self.cameras is not None and self.step_i % self.cam_every == 0

    def after_step(self, t_phys, rendered):
        self.t_phys = t_phys
        i = self.step_i
        self._clock()
        if i % 2 == 0:
            self._tf_odom_joints()
        if i % 6 == 0:
            self._people_pkgs()
        if i % 12 == 0:
            self._state()
        if i % 30 == 0:
            self._markers()
        if i % 60 == 0 and self._recent_dirty:
            self._recent_dirty = False
            self.p_events_recent.publish(self.M["String"](data=json.dumps(self._recent)))
        if i % 600 == 0 and i:
            self.p_layout.publish(self.M["String"](data=json.dumps(self.layout)))
        if self.cameras is not None:
            if i % self.cam_every == 0:
                self._aim_follow()
            if rendered:
                for k, jpg in self.cameras.grab().items():
                    m = self.M["CompressedImage"]()
                    m.header.stamp = self._stamp()
                    m.header.frame_id = f"{k}_camera"
                    m.format = "jpeg"
                    m.data = jpg
                    self.p_cam[k].publish(m)
        for _ in range(8):                      # orders / services / follow target
            self.ex.spin_once(timeout_sec=0.0)
        self.step_i += 1

    def spin_idle(self):
        """While paused: keep answering services and publishing state."""
        for _ in range(8):
            self.ex.spin_once(timeout_sec=0.0)
        if self.step_i % 12 == 0:
            self._state()
        self.step_i += 1

    def shutdown(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------------ inputs
    def _on_order(self, msg):
        try:
            d = json.loads(msg.data) if msg.data.strip().startswith("{") else {"line": msg.data.strip(), "qty": 1}
            line, qty = str(d.get("line", "")).upper(), int(d.get("qty", 1))
        except Exception as e:
            self.node.get_logger().warn(f"bad order {msg.data!r}: {e}")
            return
        if line not in self.sim.belts or qty < 1:
            self.node.get_logger().warn(f"order for unknown line {line!r} ignored")
            return
        if not self.sim.order_mode:
            self.sim.order_mode = True
            self.sim.log("wms: order mode on - belts release a crate only against an order")
        self.sim.orders[line] += qty
        self.sim.log(f"wms: order {d.get('id', '')} line {line} x{qty} (pending {self.sim.orders[line]})".replace("  ", " "))

    def _on_follow(self, msg):
        self.follow = msg.data.strip()
        self._follow_eye = None

    def _srv_pause(self, req, res):
        self.paused = True
        res.success, res.message = True, "paused"
        self.sim.log("control: simulation paused")
        return res

    def _srv_resume(self, req, res):
        self.paused = False
        res.success, res.message = True, "running"
        self.sim.log("control: simulation resumed")
        return res

    def _srv_estop(self, req, res):
        on = bool(req.data)
        if on != self.sim.estop:
            self.sim.estop = on
            self.hw.set_belts_running(not on)
            self.sim.log("control: E-STOP engaged - all machines halted" if on else "control: E-STOP released")
        res.success, res.message = True, "estop engaged" if on else "estop released"
        return res

    def _on_event(self, t, msg):
        line = f"[{t:7.1f}s] {msg}"
        self.p_events.publish(self.M["String"](data=line))
        self._recent.append(line)
        del self._recent[:-150]
        self._recent_dirty = True
        if msg.startswith("arm ") and "vacuum ON" in msg:
            self.vacuum[msg[4]] = True
        elif msg.startswith("arm ") and "vacuum OFF" in msg:
            self.vacuum[msg[4]] = False
        if ": delivered total" in msg:
            self._delivered_t.append(t)

    # ------------------------------------------------------------------ helpers
    def _stamp(self, t=None):
        from builtin_interfaces.msg import Time
        t = self.t_phys if t is None else t
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def _tfs(self, parent, child, x, y, z, yaw, stamp):
        ts = self.M["TransformStamped"]()
        ts.header.stamp, ts.header.frame_id, ts.child_frame_id = stamp, parent, child
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = float(x), float(y), float(z)
        q = _yaw_q(yaw)
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = q
        return ts

    def _clock(self):
        m = self.M["Clock"]()
        m.clock = self._stamp()
        self.p_clock.publish(m)

    def _fork_yaw(self, k):
        return math.radians(self.hw.fork_info[k]["yaw_deg"])

    def _arm_base(self, t):
        a = self.arms[t]
        try:
            p = a.chain.T0[:3, 3]
            return float(p[0]), float(p[1]), float(p[2])
        except Exception:
            cfg = C.CONVEYORS[t]
            sgn = -1 if cfg["y"] > 0 else 1
            return C.PICK_X, cfg["y"] + sgn * C.ARM_OFFSET_Y, C.ARM_PEDESTAL_H

    def _publish_static_tf(self):
        st = self._stamp(0.0)
        tfs = []
        for t in self.arms:
            x, y, z = self._arm_base(t)
            tfs.append(self._tfs("map", f"{ARM_NAME[t]}/base_link", x, y, z, 0.0, st))
        self.p_tf_static.publish(self.M["TFMessage"](transforms=tfs))

    def _tf_odom_joints(self):
        st = self._stamp()
        tfs = []
        for k, c in self.sim.carters.items():
            m = c.mob
            tfs.append(self._tfs("map", f"{ROS_NAME[k]}/base_link", m.x, m.y, 0.0, m.yaw, st))
            self.p_odom[k].publish(self._odom(k, m.x, m.y, m.yaw, m.v, getattr(m, "w", 0.0), st))
        for k, f in self.sim.forks.items():
            px, py = self._prev_fork[k]
            v = math.hypot(f.x - px, f.y - py) / (2 * C_DT())
            self._prev_fork[k] = (f.x, f.y)
            yaw = self._fork_yaw(k)
            tfs.append(self._tfs("map", f"{ROS_NAME[k]}/base_link", f.x, f.y, 0.0, yaw, st))
            self.p_odom[k].publish(self._odom(k, f.x, f.y, yaw, v, 0.0, st))
            self.p_lift[k].publish(self.M["Float64"](data=float(f.lift)))
        for p in self.people:
            m = p["mob"]
            tfs.append(self._tfs("map", p["name"], m.x, m.y, 0.0, m.yaw, st))
        self.p_tf.publish(self.M["TFMessage"](transforms=tfs))
        for t, a in self.arms.items():
            if getattr(a, "art", None) is None:
                continue
            js = self.M["JointState"]()
            js.header.stamp = st
            js.name = list(a.info["dof_names"])
            js.position = [float(v) for v in a.q()]
            self.p_js[t].publish(js)

    def _odom(self, k, x, y, yaw, v, w, st):
        o = self.M["Odometry"]()
        o.header.stamp, o.header.frame_id, o.child_frame_id = st, "map", f"{ROS_NAME[k]}/base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
        q = _yaw_q(yaw)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(v), float(w)
        return o

    def _people_pkgs(self):
        st = self._stamp()
        pa = self.M["PoseArray"]()
        pa.header.stamp, pa.header.frame_id = st, "map"
        for p in self.people:
            pa.poses.append(self._pose(p["mob"].x, p["mob"].y, 0.0, p["mob"].yaw))
        self.p_people.publish(pa)
        pk = self.M["PoseArray"]()
        pk.header.stamp, pk.header.frame_id = st, "map"
        for name, pkg in list(self.hw.pkgs.items()):
            try:
                x, y, z = pkg.pos()
            except Exception:
                continue
            pk.poses.append(self._pose(x, y, z, 0.0))
        self.p_pkgs.publish(pk)

    def _pose(self, x, y, z, yaw):
        p = self.M["Pose"]()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        q = _yaw_q(yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p

    # ------------------------------------------------------------------ JSON for the web
    def _layout(self):
        s, hw = self.sim, self.hw
        hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
        pallets = []
        for k in s.forks:
            try:
                px, py, top, sx, sy = hw.pallet_info(k)
                pallets.append(dict(name=k, x=px, y=py, w=sx, h=sy, top=top))
            except Exception:
                pass
        belts = [dict(line=t, x0=C.BELT_START_X, x1=C.BELT_START_X + C.BELT_LEN, y=b.y, w=C.BELT_W,
                      pick_x=C.PICK_X) for t, b in s.belts.items()]
        docks = [dict(line=t, x=b.dock[0], y=b.dock[1], w=0.8, h=0.6) for t, b in s.belts.items()]
        strips = [dict(name=k, x0=5.7, x1=12.9, y0=f.fy - 1.0, y1=f.fy + 1.0) for k, f in s.forks.items()]
        arms = [dict(name=ARM_NAME[t], line=t, x=self._arm_base(t)[0], y=self._arm_base(t)[1]) for t in self.arms]
        L, D, H = C.PKG_SIZE
        return dict(zone=dict(x0=-hx, x1=hx, y0=-hy, y1=hy), building=self.building,
                    belts=belts, docks=docks, forklift_strips=strips, pallets=pallets, arms=arms,
                    statics=[[float(v) for v in r] for r in C.STATIC_RECTS],
                    parking={k: list(v) for k, v in C.PARK_SPOTS.items()},
                    crate=dict(l=L, w=D, h=H), carter_radius=C.CARTER_RADIUS,
                    ros_names=dict(ROS_NAME, **{ARM_NAME[t]: ARM_NAME[t] for t in self.arms}))

    def _map(self):
        from nav import Grid
        hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
        res = 0.1
        g = Grid(-hx, hx, -hy, hy, res)
        rects = [tuple(r) for r in C.STATIC_RECTS]
        for b in self.layout["belts"]:
            rects.append((b["x0"], b["x1"], b["y"] - b["w"] / 2, b["y"] + b["w"] / 2))
        for a in self.layout["arms"]:
            rects.append((a["x"] - 0.18, a["x"] + 0.18, a["y"] - 0.18, a["y"] + 0.18))
        for p in self.layout["pallets"]:
            rects.append((p["x"] - p["w"] / 2, p["x"] + p["w"] / 2, p["y"] - p["h"] / 2, p["y"] + p["h"] / 2))
        g.build(rects, 0.0, 0.0, 0.0)
        occ = self.M["OccupancyGrid"]()
        occ.header.stamp, occ.header.frame_id = self._stamp(0.0), "map"
        occ.info.resolution = res
        occ.info.width, occ.info.height = g.nx, g.ny
        occ.info.origin.position.x, occ.info.origin.position.y = -hx, -hy
        occ.info.origin.orientation.w = 1.0
        occ.data = [int(v) for v in np.where(g.raw.T, 100, 0).astype(np.int8).ravel()]   # row-major, y rows
        return occ

    def _state(self):
        s = self.sim
        carters = []
        for k, c in s.carters.items():
            carters.append(dict(name=k, ros=ROS_NAME[k], x=round(c.mob.x, 3), y=round(c.mob.y, 3), yaw=round(c.mob.yaw, 3),
                                v=round(float(c.mob.v), 3), status=s.status.get(k, "idle"), cargo=c.cargo is not None,
                                job=(c.job[0].tag if c.job else None), parked=bool(c.parked),
                                blocked_by_person=bool(getattr(c, "_people_blocked", False))))
        forks = []
        for k, f in s.forks.items():
            forks.append(dict(name=k, ros=ROS_NAME[k], x=round(f.x, 3), y=round(f.y, 3), yaw=round(self._fork_yaw(k), 3),
                              lift=round(f.lift, 3), cargo=f.cargo is not None, delivered=f.delivered,
                              on_pallet=len(f.stack), status=s.status.get(k, "waiting for a carter")))
        arms = []
        for t, a in self.arms.items():
            x, y, _ = self._arm_base(t)
            arms.append(dict(name=ARM_NAME[t], line=t, x=x, y=y, vacuum=self.vacuum[t],
                             status=s.status.get(f"arm {t}", "stowed")))
        belts = []
        for t, b in s.belts.items():
            belts.append(dict(line=t, y=b.y, x0=C.BELT_START_X, x1=C.BELT_START_X + C.BELT_LEN, pick_x=C.PICK_X,
                              ready=bool(b.ready), placed=b.placed, orders_pending=s.orders.get(t, 0),
                              status=s.status.get(f"belt {t}", "")))
        people = []
        for p in self.people:
            m = p["mob"]
            people.append(dict(name=p["name"], kind=p.get("kind", "person"), x=round(m.x, 3), y=round(m.y, 3),
                               yaw=round(m.yaw, 3), v=round(float(m.v), 3), blocked=bool(p.get("block_reason"))))
        pkgs = []
        for name, pkg in list(self.hw.pkgs.items()):
            try:
                x, y, z = pkg.pos()
            except Exception:
                continue
            pkgs.append(dict(id=name, x=round(float(x), 3), y=round(float(y), 3), z=round(float(z), 3), yaw=0.0))
        total = sum(f.delivered for f in s.forks.values())
        win = [t for t in self._delivered_t if s.t - t <= 1800.0]
        span = min(max(s.t, 1.0), 1800.0)
        state = dict(t=round(s.t, 2), t_phys=round(self.t_phys, 2), rtf=round(self.rtf, 2), paused=self.paused,
                     estop=s.estop, mode="orders" if s.order_mode else "continuous",
                     carters=carters, forklifts=forks, arms=arms, belts=belts, people=people, packages=pkgs,
                     kpi=dict(delivered_total=total, per_hour=round(len(win) * 3600.0 / span, 1),
                              min_carter_sep=round(s.min_sep, 2), min_person_sep=round(s.min_person_sep, 2),
                              person_yields=s.person_yields, pallets_shipped=s.shipped,
                              stalls=getattr(s, "stalls", 0)),
                     follow=self.follow)
        self.p_state.publish(self.M["String"](data=json.dumps(state)))

    def _markers(self):
        M, MA = self.M["Marker"], self.M["MarkerArray"]
        st = self._stamp()
        ma = MA()
        mid = [0]

        def mk(ns, typ, x, y, z, sx, sy, sz, rgba, yaw=0.0, text=""):
            m = M()
            m.header.stamp, m.header.frame_id = st, "map"
            m.ns, m.id, m.type, m.action = ns, mid[0], typ, M.ADD
            mid[0] += 1
            m.pose = self._pose(x, y, z, yaw)
            # every field must be a real float: Isaac's Python runs with asserts off, so an int here
            # is not rejected by the message setter and aborts the process inside rosidl's C conversion
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            m.text = text
            m.lifetime.sec = 2
            ma.markers.append(m)

        for b in self.layout["belts"]:
            mk("belts", M.CUBE, (b["x0"] + b["x1"]) / 2, b["y"], C.BELT_H / 2, b["x1"] - b["x0"], b["w"], C.BELT_H, (0.1, 0.1, 0.12, 1.0))
        for r in self.layout["statics"]:
            mk("statics", M.CUBE, (r[0] + r[1]) / 2, (r[2] + r[3]) / 2, 1.0, r[1] - r[0], r[3] - r[2], 2.0, (0.85, 0.45, 0.1, 0.6))
        for p in self.people:
            m = p["mob"]
            mk("people", M.CYLINDER, m.x, m.y, 0.9, 0.5, 0.5, 1.8, (0.2, 0.5, 0.95, 0.9))
            mk("people_names", M.TEXT_VIEW_FACING, m.x, m.y, 2.1, 0, 0, 0.3, (1, 1, 1, 1), text=p["name"])
        L, D, H = C.PKG_SIZE
        for name, pkg in list(self.hw.pkgs.items()):
            try:
                x, y, z = pkg.pos()
            except Exception:
                continue
            mk("crates", M.CUBE, x, y, z, L, D, H, (0.72, 0.52, 0.3, 1.0))
        for k, c in self.sim.carters.items():
            mk("carters", M.CUBE, c.mob.x, c.mob.y, 0.3, 0.9, 0.7, 0.6, (0.2, 0.75, 0.35, 1.0), c.mob.yaw)
            mk("names", M.TEXT_VIEW_FACING, c.mob.x, c.mob.y, 1.2, 0, 0, 0.35, (1, 1, 1, 1), text=k)
        for k, f in self.sim.forks.items():
            mk("forklifts", M.CUBE, f.x, f.y, 1.0, 2.6, 1.2, 2.0, (0.15, 0.3, 0.8, 1.0), self._fork_yaw(k))
            mk("names", M.TEXT_VIEW_FACING, f.x, f.y, 2.5, 0, 0, 0.35, (1, 1, 1, 1), text=k)
        self.p_markers.publish(ma)

    # ------------------------------------------------------------------ follow camera
    def _aim_follow(self):
        tgt = self.follow
        pose = None
        if tgt in self.sim.carters:
            m = self.sim.carters[tgt].mob
            pose, back, up = (m.x, m.y, m.yaw), 3.6, 2.4
        elif tgt in self.sim.forks:
            f = self.sim.forks[tgt]
            pose, back, up = (f.x, f.y, self._fork_yaw(tgt)), 5.5, 3.2
        elif tgt in ("ur5", "franka"):
            t = "A" if tgt == "ur5" else "B"
            x, y, _ = self._arm_base(t)
            sgn = -1 if C.CONVEYORS[t]["y"] > 0 else 1
            eye, look = np.array([x + 2.6, y + sgn * 2.6, 2.6]), np.array([x, y - sgn * 0.3, 0.9])
            self.cameras.aim_follow(eye, look)
            return
        else:
            for p in self.people:
                if p["name"] == tgt:
                    m = p["mob"]
                    pose, back, up = (m.x, m.y, m.yaw), 3.4, 2.0
        if pose is None:                                    # "overview" or unknown: a high wide shot
            self.cameras.aim_follow(np.array([-14.0, 0.0, 7.5]), np.array([8.0, 0.0, 0.0]))
            return
        x, y, yaw = pose
        eye = np.array([x - back * math.cos(yaw), y - back * math.sin(yaw), up])
        look = np.array([x + 1.0 * math.cos(yaw), y + 1.0 * math.sin(yaw), 0.6])
        if self._follow_eye is None:
            self._follow_eye = eye
        self._follow_eye = self._follow_eye * 0.6 + eye * 0.4
        self.cameras.aim_follow(self._follow_eye, look)


def C_DT():
    from logic import DT
    return DT


class _Cameras:
    """Overview + follow render products read back through Replicator rgb annotators."""

    def __init__(self, stage, res, q):
        import omni.replicator.core as rep
        from pxr import UsdGeom, Gf
        self.q = q
        self.paths = {"overview": "/World/OverviewCam", "follow": "/World/FollowCamROS"}
        for k, p in self.paths.items():
            cam = UsdGeom.Camera.Get(stage, p) or UsdGeom.Camera.Define(stage, p)
            if k == "follow":
                cam.GetFocalLengthAttr().Set(16.0)
                cam.GetHorizontalApertureAttr().Set(20.955)
                cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
        self.ann = {}
        for k, p in self.paths.items():
            rp = rep.create.render_product(p, tuple(res))
            a = rep.AnnotatorRegistry.get_annotator("rgb")
            a.attach([rp])
            self.ann[k] = a

    def aim_follow(self, eye, target):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.asarray(eye, float), target=np.asarray(target, float),
                        camera_prim_path=self.paths["follow"])

    def grab(self):
        from PIL import Image
        out = {}
        for k, a in self.ann.items():
            try:
                d = a.get_data()
            except Exception:
                continue
            if d is None or not hasattr(d, "shape") or d.size == 0 or d.ndim != 3:
                continue
            buf = io.BytesIO()
            Image.fromarray(np.ascontiguousarray(d[:, :, :3])).save(buf, format="JPEG", quality=self.q)
            out[k] = buf.getvalue()
        return out
