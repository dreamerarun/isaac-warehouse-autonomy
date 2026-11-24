"""
Step 3: autonomous warehouse.
  belt -> UR5 / Franka (pneumatic suction cup) -> Nova Carter (load deck) -> forklift (forks under the crate skids) -> pallet.
  Carter C3 roams as a backup/supervisor.  Next crate is released only after the previous one is on a carter.
  Racks, pallets with cartons and columns are real warehouse props and are avoided by every vehicle.

    ./python.sh ~/warehouse/step3_autonomy.py
    ./python.sh ~/warehouse/step3_autonomy.py --headless --steps 36000     (10 min sim, no GUI)
"""
import argparse, sys, os, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(line_buffering=True)
ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--steps", type=int, default=0, help="physics steps to run; 0 = until the window is closed")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--no-statics", action="store_true", help="skip racks/props (debug)")
ap.add_argument("--shots", type=int, default=0, help="capture a viewport frame every N steps into ~/warehouse/shots")
ap.add_argument("--track", type=int, default=-1, help="cinematic follow-cam on person N (-1 = fixed camera)")
ap.add_argument("--record", default="", help="directory to write a numbered frame sequence of the tracked shot")
ap.add_argument("--every", type=int, default=2, help="with --record: one frame every N physics steps (2 = 30 fps)")
ap.add_argument("--record-from", type=float, default=0.0, help="with --record: start recording at this sim time (s)")
ap.add_argument("--view", default="", help="fixed camera 'ex,ey,ez,tx,ty,tz' (eye, target in metres) instead of the overview")
ap.add_argument("--focal", type=float, default=9.0, help="focal length (mm) of the fixed camera")
ap.add_argument("--room", action="store_true", help="plain 26 x 16 m shell instead of the NVIDIA warehouse building")
ap.add_argument("--ros", action="store_true", help="ROS 2 interface (topics, services, cameras) - see ros_bridge.py")
ap.add_argument("--orders", action="store_true", help="belts release crates only against /warehouse/orders (WMS)")
ap.add_argument("--realtime", action="store_true", help="pace the simulation to wall-clock time")
ap.add_argument("--cam-hz", type=float, default=4.0, help="with --ros: camera frame rate")
ap.add_argument("--no-cams", action="store_true", help="with --ros: no camera topics (saves rendering)")
args, _ = ap.parse_known_args()

import config as C
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless, "width": C.RENDER_W, "height": C.RENDER_H, "renderer": C.RENDERER})

import numpy as np
from pxr import Gf, UsdGeom
import omni.usd
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
import scene, assets, statics
from hw_isaac import IsaacArm, IsaacHW
from logic import Sim, DT

root = get_assets_root_path()
if not root:
    raise SystemExit("No asset root - check internet connection")
A = C.ASSETS
building_url = None if (args.room or not C.BUILDING_URL) else f"{root}/{C.BUILDING_URL}"
if building_url:
    C.WALL_H = C.BUILDING_CEILING_H          # structural columns run up to the hall's ceiling

# =============================================================================================
# Phase 0: measure the real assets off-screen and derive every size from them
# =============================================================================================
mf = assets.measure(f"{root}/{A['forklift']}", (1.5, 6.0))
fg = assets.forklift_geometry(mf)
if fg is None:
    print("[step3] WARNING: fork tines not recognised in the forklift asset; using default fork geometry")
    fg = dict(forward=1.0, tip=max(float(mf["max"][0]), 1.0), surf=0.07, thk=0.05, width=0.70, count=2, length=1.0)
fork_visual_yaw = (180.0 if fg["forward"] > 0 else 0.0) + C.FORKLIFT_VISUAL_YAW_FIX      # forks must point toward -x
print(f"[step3] forklift: scale {mf['scale']:g}, forks {fg['count']} tines, tip {fg['tip']:.2f} m, top {fg['surf']:.3f} m, "
      f"thickness {fg['thk']:.3f} m, outer width {fg['width']:.2f} m -> visual yaw {fork_visual_yaw:.0f} deg")

Wp = min(max(fg["width"] + 2 * (C.PKG_SKID_W + 0.04), 0.80), 1.30)
Dp, Hp = 0.50, 0.32
skid_h = min(max(0.09, fg["thk"] + 0.045), 0.5 * Hp)   # must stay below the crate height -> valid colliders
C.PKG_SIZE, C.PKG_SKID_H = (Wp, Dp, Hp), skid_h
print(f"[step3] crate: {Wp:.2f} x {Dp:.2f} x {Hp:.2f} m on skids {skid_h:.2f} m high (forks fit between the skids)")

mc = assets.measure(f"{root}/{A['carter']}")
cg = assets.carter_geometry(mc)
alpha = math.radians(C.CARTER_VISUAL_YAW_OFFSET_DEG)
plate_L, plate_W, plate_t = Wp + 0.10, Dp + 0.10, 0.03
plate_top_local = cg["top"] + 0.02 + plate_t
cx, cy = cg["centre"]
if math.hypot(cx, cy) > 0.10:
    print(f"[step3] carter body centre is offset ({cx:.2f}, {cy:.2f}) m from its drive axis; centring the load deck on the axis")
    cx = cy = 0.0
C.CARTER_PLATE = dict(size=(plate_L, plate_W), thick=plate_t, centre=(cx, cy))
C.CARTER_SLOT = (cx * math.cos(alpha) - cy * math.sin(alpha), cx * math.sin(alpha) + cy * math.cos(alpha))
C.CARTER_DECK_H = C.CARTER_Z0 + plate_top_local + 0.004
C.CARTER_RADIUS = min(max(math.hypot(plate_L / 2, plate_W / 2) + math.hypot(cx, cy) + 0.05, 0.55), 0.95)
print(f"[step3] carter: body {cg['size'][0]:.2f} x {cg['size'][1]:.2f} m, top {cg['top']:.2f} m; load deck {plate_L:.2f} x {plate_W:.2f} m "
      f"at z={C.CARTER_DECK_H:.2f} m; planning radius {C.CARTER_RADIUS:.2f} m")

mp = assets.measure(f"{root}/{A['pallet']}", (0.8, 1.6))
pallet_yaw = 90.0 if (mp["max"][0] - mp["min"][0]) > (mp["max"][1] - mp["min"][1]) else 0.0     # long side along y

C.PICK_X = C.BELT_START_X + C.BELT_LEN - Wp / 2 - 0.35          # whole crate sits on the belt
C.DOCK_OFFSET_Y = C.ARM_OFFSET_Y + 0.175 + plate_W / 2 + 0.15   # deck clears the pedestal, within arm reach
print(f"[step3] pick point x={C.PICK_X:.2f}; dock {C.DOCK_OFFSET_Y:.2f} m from the belt centre line")

# =============================================================================================
# Phase 1: build the scene
# =============================================================================================
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=2 * DT)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
scene.build(stage, pallet_boxes=False, legacy_statics=False, building_url=building_url)   # shell + conveyors + lights (step-3 dressing is statics.py)

arm_specs = {}
for tag, cfg in C.CONVEYORS.items():
    sgn = -1 if cfg["y"] > 0 else +1
    ped_y = cfg["y"] + sgn * C.ARM_OFFSET_Y
    yaw = 90.0 if cfg["y"] > 0 else -90.0
    path = f"/World/Manip_{cfg['arm']}/Robot"
    assets.place(stage, root, A, path, cfg["arm"], (C.PICK_X, ped_y, C.ARM_PEDESTAL_H), yaw, expect=(0.4, 1.6))
    arm_specs[tag] = (cfg["arm"], path)

carter_paths = {}
for name, (x, y) in C.CARTER_START.items():
    p, sc = assets.place(stage, root, A, f"/World/Carters/{name}", "carter", (x, y, C.CARTER_Z0), 0.0)
    assets.strip_physics(p)
    assets.add_carrier_plate(stage, f"/World/Carters/{name}", (plate_L / sc, plate_W / sc), plate_t / sc,
                             (cx / sc, cy / sc), plate_top_local / sc, cg["chassis_top"] / sc, -C.CARTER_VISUAL_YAW_OFFSET_DEG)
    carter_paths[name] = f"/World/Carters/{name}"

fork_info, pallet_paths = {}, {}
for name, f in C.FORKLIFTS.items():
    fx, fy = f["pos"]
    p, sc = assets.place(stage, root, A, f"/World/Forklifts/{name}", "forklift", (fx, fy, 0.0), fork_visual_yaw, expect=(1.5, 6.0))
    lift_path = assets.find_joint_body(p, "lift_joint")
    assets.strip_physics(p)
    fork_info[name] = dict(path=f"/World/Forklifts/{name}", pos=(fx, fy), yaw_deg=fork_visual_yaw, scale=sc, lift_path=lift_path)
    C.FORK_GEOM[name] = fg
    pp, _ = assets.place(stage, root, A, f"/World/Forklifts/{name}_pallet_asset", "pallet",
                         (fx + C.PALLET_OFFSET_X, fy, 0.0), pallet_yaw, expect=(0.8, 1.6))
    assets.strip_physics(pp)
    pallet_paths[name] = f"/World/Forklifts/{name}_pallet_asset"

# ---- static warehouse dressing (racks, pallets with cartons, columns) ------------------------------
keep_out = []
for tag, cfg in C.CONVEYORS.items():
    y = cfg["y"]
    dock_y = y + (-1 if y > 0 else 1) * C.DOCK_OFFSET_Y
    keep_out.append((C.BELT_START_X - 0.6, C.PICK_X + 1.6, min(y, dock_y) - 1.2, max(y, dock_y) + 1.2))
for name, f in C.FORKLIFTS.items():
    fx, fy = f["pos"]
    s = 1 if fy > 0 else -1
    keep_out.append((5.0, 13.0, fy - 1.4, fy + 1.4))
    keep_out.append((C.CARTER_TRANSFER_X - 1.8, C.CARTER_TRANSFER_X + 1.8, min(fy, fy - s * 4.0), max(fy, fy - s * 4.0)))
for _, (x, y) in list(C.CARTER_START.items()) + list(C.PARK_SPOTS.items()):
    keep_out.append((x - 1.2, x + 1.2, y - 1.2, y + 1.2))
C.STATIC_RECTS = [] if args.no_statics else statics.build(stage, root, keep_out)
print(f"[step3] {len(C.STATIC_RECTS)} static obstacles registered for navigation")

# ---- dynamic people + moving props (SimReady characters, retargeted walk clips) ----------------
import people as people_mod
people = people_mod.build(stage, root, keep_out=keep_out, seed=args.seed)
if os.environ.get("STANDTEST"):        # force the standing pose, to eyeball the blocked state
    for p in people:
        p["force_stand"] = True

# ---- arm kinematics + suction cups (authoring, before physics starts) --------------------------------
arms = {}
for tag, (kind, path) in arm_specs.items():
    ee = ("wrist_3_link", "tool0", "ee_link") if kind == "ur5" else ("panda_hand", "panda_link8", "panda_link7")
    arms[tag] = IsaacArm(f"{tag}-{kind}", stage, path, ee, hide_fingers_flag=(kind == "franka"), tag=tag)

from usd_kin import filter_arm_collisions
removed = filter_arm_collisions(stage, [p for _, p in arm_specs.values()])
print(f"[step3] arms cannot physically hit crates (collision group); {removed} arm colliders removed")

def _overview_cam():
    """High corner view down the room diagonal with a wide lens, so one frame holds the whole
    floor. The eye stays inside the shell (ceiling at z=5.1); a ~99 deg horizontal FOV comes
    from a 9 mm focal length instead of backing the camera through the wall."""
    # the built-in /OmniverseKit_Persp camera recomputes its lens from the viewport controller,
    # so a focal length written there is ignored; author a real USD camera and render through it.
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    path = "/World/OverviewCam"
    try:
        cam = UsdGeom.Camera.Get(omni.usd.get_context().get_stage(), path) or \
            UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), path)
        cam.GetFocalLengthAttr().Set(float(args.focal))
        cam.GetHorizontalApertureAttr().Set(20.955)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
        # aimed well below the far wall: the eye sits 0.2 m under the ceiling, so a shallow
        # pitch would fill the top third of the frame with the unlit ceiling underside
        eye, tgt = np.array([-10.6, -6.0, 4.70]), np.array([13.4, 5.8, -7.4])
        if building_url:                    # 9 m hall: higher eye, the rack aisles show beyond the cell
            eye, tgt = np.array(C.OVERVIEW_EYE), np.array(C.OVERVIEW_TARGET)
        if args.view:
            v = [float(s) for s in args.view.split(",")]
            eye, tgt = np.array(v[:3]), np.array(v[3:6])
        set_camera_view(eye=eye, target=tgt, camera_prim_path=path)
        get_active_viewport().camera_path = path
        print(f"[step3] overview camera {path} focal={cam.GetFocalLengthAttr().Get():.2f}", flush=True)
    except Exception as e:
        print(f"[step3] wide-lens camera setup skipped: {e}", flush=True)
        set_camera_view(eye=np.array([-12.4, -7.4, 4.85]), target=np.array([0.0, 0.0, 0.6]))


_FOLLOW_CAM = "/World/FollowCam"
_FOLLOW_LENS = [False]


def _follow_cam(eye, target):
    """Render the tracking shot through its own camera prim.

    An `--record` run points the active viewport at /World/OverviewCam, so a bare
    set_camera_view() (which drives the default Persp camera) would never reach the frame and
    every "follow" video would just be the wide corner shot.
    """
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    if not _FOLLOW_LENS[0]:
        cam = UsdGeom.Camera.Get(stage, _FOLLOW_CAM) or UsdGeom.Camera.Define(stage, _FOLLOW_CAM)
        cam.GetFocalLengthAttr().Set(18.14756)      # the kit default lens the tracking shots used
        cam.GetHorizontalApertureAttr().Set(20.955)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
        print(f"[step3] follow camera {_FOLLOW_CAM} focal={cam.GetFocalLengthAttr().Get():.2f}", flush=True)
        _FOLLOW_LENS[0] = True
    set_camera_view(eye=eye, target=target, camera_prim_path=_FOLLOW_CAM)
    get_active_viewport().camera_path = _FOLLOW_CAM


# camera stays *inside* the warehouse (the shell has a ceiling at z=5.1, so a higher eye
# only sees the roof and every capture comes out black)
if not args.headless or args.shots or args.record or args.ros:
    _overview_cam()

world.reset()
# reset() stops the omni timeline; restart it so the bound walk clips keep advancing while
# world.step() drives the physics (otherwise the people slide along in their rest pose)
world.play()
# keep the overview camera after reset so a headless --record run still frames the whole room
if args.record:
    _overview_cam()
for a in arms.values():
    a.init_sim()
for a in arms.values():
    a.tune(world)

hw = IsaacHW(stage, carter_paths, fork_info, pallet_paths, arms)
sim = Sim(hw, arms, seed=args.seed, people=people)
sim.order_mode = bool(args.orders)
sim.start()

bridge = None
if args.ros:
    from ros_bridge import RosBridge
    bi = None
    if building_url:
        x0, x1, y0, y1 = scene.building_interior()
        bi = dict(x0=x0, x1=x1, y0=y0, y1=y1)
    vp_rp = None
    if not args.headless:                 # the window shows the overview camera: reuse its frames
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp_rp = get_active_viewport().render_product_path
        except Exception as e:
            print(f"[step3] viewport render product unavailable ({e}); overview renders separately", flush=True)
    bridge = RosBridge(sim, people, hw, arms, stage, building=bi, cameras=not args.no_cams, cam_hz=args.cam_hz,
                       viewport_rp=vp_rp)

step, checked = 0, False
shot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shots")
if args.shots:
    os.makedirs(shot_dir, exist_ok=True)
if args.record:
    os.makedirs(args.record, exist_ok=True)


def _capture_frame(path):
    """Render one viewport frame to `path` and wait until the async write has settled."""
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    if os.path.exists(path):       # never let the settle-wait accept a stale frame
        os.remove(path)
    world.render()                 # let the viewport produce the frame before reading it back
    app.update()
    capture_viewport_to_file(get_active_viewport(), path)
    last = -1
    for _ in range(120):
        app.update()
        size = os.path.getsize(path) if os.path.exists(path) else 0
        if size > 0 and size == last:
            break
        last = size


_cam = {"eye": None, "tgt": None}
GUI_EVERY = max(1, int(os.environ.get("GUI_EVERY", "2")))
if not args.headless:
    # the hall is a heavy RTX scene: render the window at 1280x720 with DLSS in performance mode
    # so the GUI run keeps (close to) real time; GUI_EVERY=3 trades window fps for more speed
    try:
        import carb
        from omni.kit.viewport.utility import get_active_viewport
        carb.settings.get_settings().set("/rtx/post/dlss/execMode", 0)
        get_active_viewport().resolution = (C.RENDER_W, C.RENDER_H)
        print(f"[step3] GUI: viewport {C.RENDER_W}x{C.RENDER_H}, DLSS performance, a frame every "
              f"{GUI_EVERY} physics steps", flush=True)
    except Exception as e:
        print(f"[step3] GUI render tuning skipped: {e}", flush=True)
t_phys = 0.0                                  # physics time; people walk on it (they keep going in an e-stop)
wall0 = time.monotonic()
rtf_mark = (0.0, wall0)
while app.is_running():
    if bridge is not None and bridge.paused:
        if world.is_playing():
            world.pause()
        app.update()
        bridge.spin_idle()
        time.sleep(0.02)
        wall0 = time.monotonic() - t_phys        # real-time pacing restarts from here
        continue
    if bridge is not None and not world.is_playing():
        world.play()
    shot = bool(args.shots) and step % args.shots == 0
    frame = bool(args.record) and step % max(args.every, 1) == 0 and step * DT >= args.record_from
    want = bridge.want_render() if bridge is not None else False
    gui_frame = not args.headless and step % GUI_EVERY == 0   # window frame rate = 60 / GUI_EVERY fps
    world.step(render=gui_frame or shot or frame or args.track >= 0 or want)
    sim.step()
    t_phys += DT
    # dynamic people/props: plan their routes, walk them, write their transforms, loop the walk clip
    people_mod.step(people, DT, t_phys,
                    obstacles=lambda: [(c.mob.x, c.mob.y, C.CARTER_RADIUS)
                                       for c in sim.carters.values()])
    if bridge is not None:
        now = time.monotonic()
        if now - rtf_mark[1] >= 2.0:
            bridge.rtf = (t_phys - rtf_mark[0]) / (now - rtf_mark[1])
            rtf_mark = (t_phys, now)
        bridge.after_step(t_phys, rendered=want)
    if args.realtime:
        ahead = t_phys - (time.monotonic() - wall0)
        if ahead > 0:
            time.sleep(ahead)
    if 0 <= args.track < len(people):
        from isaacsim.core.utils.viewports import set_camera_view
        m0 = people[args.track]["mob"]
        hx, hy = math.cos(m0.yaw), math.sin(m0.yaw)

        grid = people[args.track]["grid"]

        def clear(pt):
            """False when a rack/belt/keep-out occupies this camera point (the people grid knows
            every static obstacle, while C.STATIC_RECTS only lists the nav-registered few)."""
            if not (-12.6 <= pt[0] <= 12.6 and -7.6 <= pt[1] <= 7.6):
                return False
            try:
                return bool(grid.free_point(float(pt[0]), float(pt[1])))
            except Exception:
                return True

        # 3/4 front view, so the shot shows the face and the full stride as the person walks
        # towards the camera; side/height are chosen to stay out of the racks, never inside one
        # side tracking shot: the profile is the clearest read of a walk (heel strike, toe-off,
        # arm swing), so the camera rides alongside with a little forward lead
        want_eye, want_tgt = None, np.array([m0.x + hx * 0.15, m0.y + hy * 0.15, 1.00])
        for fwd, side, zc in ((0.8, 2.7, 1.60), (0.8, -2.7, 1.60), (-0.6, 2.4, 2.10),
                              (-0.6, -2.4, 2.10), (2.6, 1.3, 1.75), (2.6, -1.3, 1.75)):
            cand = np.array([m0.x + hx * fwd - hy * side, m0.y + hy * fwd + hx * side, zc])
            if clear(cand):
                want_eye = cand
                break
        if want_eye is None:                                  # nowhere clear: hold the last good eye
            want_eye = np.array([m0.x - hx * 2.6, m0.y - hy * 2.6, 3.4]) \
                if _cam["eye"] is None else np.array(_cam["eye"])
        if _cam["eye"] is None:
            _cam["eye"], _cam["tgt"] = want_eye, want_tgt
        else:                                                 # smooth, cinematic follow
            _cam["eye"] = _cam["eye"] * 0.88 + want_eye * 0.12
            _cam["tgt"] = _cam["tgt"] * 0.75 + want_tgt * 0.25
        gap = _cam["eye"][:2] - np.array([m0.x, m0.y])       # never get closer than 2 m
        d = float(np.linalg.norm(gap))
        if d < 2.0:
            _cam["eye"][:2] = np.array([m0.x, m0.y]) + (gap / max(d, 1e-6)) * 2.0
        _follow_cam(_cam["eye"], _cam["tgt"])
    if os.environ.get("CLOSEUP") and people:
        from isaacsim.core.utils.viewports import set_camera_view
        ci = int(os.environ.get("CLOSEUP_IDX", "0"))
        m0 = people[ci]["mob"]
        # stand off to the side of the walker's heading so a stride is not foreshortened
        _hx, _hy = np.cos(m0.yaw), np.sin(m0.yaw)
        set_camera_view(eye=np.array([m0.x - _hy * 2.4, m0.y + _hx * 2.4, 0.9]),
                        target=np.array([m0.x, m0.y, 0.6]))
    step += 1
    if shot:
        _capture_frame(os.path.join(shot_dir, f"step3_{step:05d}.png"))
    if frame:
        _capture_frame(os.path.join(args.record, f"frame_{step:06d}.png"))
    if step == 30 and not checked:
        checked = True
        for a in arms.values():
            a.selfcheck()
    if os.environ.get("WAREHOUSE_DIAG") and shot:
        for prim in stage.Traverse():
            if not prim.GetName() or "World" not in str(prim.GetPath()):
                continue
            try:
                mn, mx = assets.world_bbox(prim, render=True)
            except Exception:
                continue
            if 0.20 < float(mn[2]) < 2.0 and float(mx[0] - mn[0]) < 2.5 and float(mx[1] - mn[1]) < 2.5:
                print(f"[diag] {prim.GetPath()} xy {(mn[0]+mx[0])/2:.2f},{(mn[1]+mx[1])/2:.2f} z {mn[2]:.2f}..{mx[2]:.2f}", flush=True)
    diag_every = float(os.environ.get("CARTER_DIAG", "0") or 0)
    if diag_every and step % max(int(diag_every / DT), 1) == 0:
        for c in sim.carters.values():
            print(f"[diag {sim.t:6.1f}s] {c.name} ({c.mob.x:6.2f},{c.mob.y:6.2f}) yaw {math.degrees(c.mob.yaw):6.1f} "
                  f"v {c.mob.v:5.2f} job {c.job[0].tag if c.job else '-'} parked {c.parked} evading {c.evading} "
                  f"pblk {c._people_blocked} | {sim.status.get(c.name, '')}", flush=True)
        for p in people:
            m = p["mob"]
            print(f"[diag {sim.t:6.1f}s]   {p['name']} ({m.x:6.2f},{m.y:6.2f}) v {m.v:4.2f} path {len(p['path'] or [])} "
                  f"goal {p['goal_idx']} fails {p.get('plan_fails', 0)} block '{p.get('block_reason', '')}'", flush=True)
    if step % int(120 / DT) == 0:
        print(f"[status {sim.t:6.0f}s] delivered {{{', '.join(f'{k}: {f.delivered}' for k, f in sim.forks.items())}}} "
              f"| closest carters {sim.min_sep:.2f} m | closest person {sim.min_person_sep:.2f} m", flush=True)
        for p in people:                      # evidence that people/props really move
            m = p["mob"]
            print(f"[step3 people] {p['name']} at ({m.x:5.1f}, {m.y:5.1f}) "
                  f"yaw {math.degrees(m.yaw):6.1f} deg, v={m.v:.2f} m/s, "
                  f"blocked {p.get('blocked_s', 0.0):5.1f} s in {p.get('blocks', 0)} frames, "
                  f"anim {p.get('anim_state', '-')}", flush=True)
    if args.steps and step >= args.steps:
        break
for p in people:
    print(f"[step3 people] summary {p['name']}: blocked {p.get('blocked_s', 0.0):.1f} s in "
          f"{p.get('blocks', 0)} frames, longest run {p.get('max_block_s', 0.0):.2f} s, "
          f"ended in '{p.get('anim_state', '-')}' pose", flush=True)
if bridge is not None:
    bridge.shutdown()
app.close()
