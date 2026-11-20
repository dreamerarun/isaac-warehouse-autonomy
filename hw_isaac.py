"""Isaac Sim implementation of the hardware interface used by logic.Sim."""
import math, traceback
import numpy as np
import config as C
from assets import world_bbox


# ---------------------------------------------------------------------------------------------
# packages: crate body on two skids (fork clearance), compound rigid body
# ---------------------------------------------------------------------------------------------
def make_crate(stage, path, pos):
    from pxr import UsdGeom, UsdPhysics, Gf
    L, D, H = C.PKG_SIZE
    hs, sw = C.PKG_SKID_H, C.PKG_SKID_W
    root = UsdGeom.Xform.Define(stage, path)
    UsdGeom.Xformable(root).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))

    def part(name, size, p, rgb):
        c = UsdGeom.Cube.Define(stage, f"{path}/{name}")
        c.GetSizeAttr().Set(1.0)
        f = UsdGeom.Xformable(c)
        f.AddTranslateOp().Set(Gf.Vec3d(*p))
        f.AddScaleOp().Set(Gf.Vec3f(*size))
        c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

    part("body", (L, D, H - hs), (0, 0, hs / 2), (0.72, 0.52, 0.30))
    part("tape", (L * 0.98, 0.05, H - hs + 0.004), (0, 0, hs / 2), (0.85, 0.75, 0.5))
    for i, sx in enumerate((-1, 1)):
        part(f"skid{i}", (sw, D, hs), (sx * (L / 2 - sw / 2), 0, -H / 2 + hs / 2), (0.45, 0.32, 0.18))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(C.PKG_MASS)
    return prim


class IsaacPkg:
    def __init__(self, prim):
        self.prim = prim
        self.detached = False
        self._last, self._still, self._p = None, 0, None
        self.t_op = self.r_op = None

    def _world(self):
        from pxr import Usd, UsdGeom
        t = UsdGeom.Xformable(self.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        return np.array([t[0], t[1], t[2]])

    def pos(self):
        return self._p.copy() if self.detached else self._world()

    def settled(self):
        p = self._world()
        moved = float(np.linalg.norm(p - self._last)) if self._last is not None else 1.0
        self._last = p
        self._still = self._still + 1 if moved < 3e-4 else 0
        return self._still >= 15

    def detach_physics(self):
        if self.detached:
            return
        from pxr import Usd, UsdGeom, UsdPhysics
        self._p = self._world()
        for p in Usd.PrimRange(self.prim):
            for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.MassAPI):
                try:
                    if p.HasAPI(api):
                        p.RemoveAPI(api)
                except Exception:
                    pass
        xf = UsdGeom.Xformable(self.prim)
        xf.ClearXformOpOrder()
        self.t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.r_op = xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble)
        self.detached = True
        self.set_pose(self._p, 0.0)

    def set_pose(self, p, yaw=0.0):
        from pxr import Gf
        if not self.detached:
            self.detach_physics()
        self._p = np.array(p, float)
        self.t_op.Set(Gf.Vec3d(*self._p))
        self.r_op.Set(math.degrees(yaw))


class RootOps:
    """translate / rotateZ ops of a placed asset root."""
    def __init__(self, prim):
        from pxr import UsdGeom
        self.t = self.r = None
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self.t = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                self.r = op

    def set(self, x, y, z, yaw_deg):
        from pxr import Gf
        self.t.Set(Gf.Vec3d(x, y, z))
        self.r.Set(float(yaw_deg))


class LiftSetter:
    """Raises the fork carriage by translating its link (visual only)."""
    def __init__(self, stage, link_path, unit_scale):
        self.op, self.k = None, unit_scale
        try:
            from pxr import UsdGeom
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(link_path))
            op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, opSuffix="lift")
            rest = [o for o in xf.GetOrderedXformOps() if o.GetName() != op.GetName()]
            xf.SetXformOpOrder([op] + rest)
            self.op = op
        except Exception as e:
            print(f"[hw] fork lift animation unavailable ({e}); forks stay low, package still follows")

    def set(self, h):
        if self.op is not None:
            from pxr import Gf
            self.op.Set(Gf.Vec3d(0, 0, h / self.k))


# ---------------------------------------------------------------------------------------------
class IsaacArm:
    def __init__(self, name, stage, robot_path, ee_candidates, hide_fingers_flag, tag=None):
        self.name, self.stage, self.robot_path, self.tag = name, stage, robot_path, tag
        self.ok, self.n, self.art, self.idx, self._cmd = False, 0, None, None, None
        try:
            from usd_kin import build_chain, hide_fingers, add_suction_cup, ensure_fixed_base
            from pxr import Usd, UsdPhysics
            self.chain, self.info = build_chain(stage, robot_path, ee_candidates, C.SUCTION_LEN)
            msg = ensure_fixed_base(stage, robot_path, self.info)
            if hide_fingers_flag:
                hide_fingers(stage, robot_path)
            add_suction_cup(stage, self.info["ee_path"], self.chain.tool_dir, self.info["mount"], C.SUCTION_LEN)
            self.art_path = robot_path
            for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path)):
                if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                    self.art_path = p.GetPath().pathString
                    break
            self.n = self.chain.n
            self.ok = True
            print(f"[arm {name}] chain: {self.n} DOF {self.info['dof_names']} | EE {self.info['ee_path']}")
            print(f"[arm {name}] {self.info['reach_dbg']} | articulation root {self.art_path} | base: {msg}")
            print(f"[arm {name}] joint limits (deg): " + ", ".join(
                f"{j.name.replace('_joint', '')} {math.degrees(j.lo):.0f}..{math.degrees(j.hi):.0f}" for j in self.chain.act))
            for jp in self.info["joint_prims"]:
                d = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(jp), "angular")
                if d:
                    print(f"[arm {name}]   drive {jp.split('/')[-1]}: stiffness={d.GetStiffnessAttr().Get()} "
                          f"damping={d.GetDampingAttr().Get()} maxForce={d.GetMaxForceAttr().Get()}")
                else:
                    print(f"[arm {name}]   WARNING joint {jp.split('/')[-1]} has no angular drive")
        except Exception:
            print(f"[arm {name}] kinematic setup FAILED -> direct-transfer fallback will be used:")
            traceback.print_exc()

    def init_sim(self):
        if not self.ok:
            return
        try:
            from isaacsim.core.prims import SingleArticulation
            self.art = SingleArticulation(prim_path=self.art_path, name=f"art_{self.name}")
            self.art.initialize()
            names = list(self.art.dof_names)
            self.idx = np.array([names.index(n) for n in self.info["dof_names"]])
            self._cmd = self.q().copy()
            self.command(self._cmd)
            print(f"[arm {self.name}] articulation ready, dofs {names}")
            self.stow()
        except Exception:
            print(f"[arm {self.name}] articulation init FAILED -> fallback:")
            traceback.print_exc()
            self.ok = False

    def stow(self):
        """Start (and rest) in a compact pose above the pedestal, on the carter side - never over the belt."""
        if self.tag is None or not self.ok:
            return
        try:
            base = self.chain.T0[:3, 3]
            belt_y = C.CONVEYORS[self.tag]["y"]
            away = -1.0 if belt_y > base[1] else 1.0
            boxes = arm_boxes_for(self.tag) + [crate_corridor(self.tag)]
            q0 = self.q()
            for off in (0.30, 0.38, 0.22, 0.45):
                for dz in (0.35, 0.50, 0.25, 0.60):
                    tgt = np.array([base[0], base[1] + away * off, base[2] + dz])
                    q, k = self.chain.ik(tgt, q0)
                    if k and not self.chain.collides(q, boxes):
                        full = np.array(self.art.get_joint_positions(), float)
                        full[self.idx] = q
                        self.art.set_joint_positions(full)
                        self.art.set_joint_velocities(np.zeros_like(full))
                        self.command(q)
                        print(f"[arm {self.name}] stow pose set (tip above the pedestal, off the belt): q={np.round(q, 2)}")
                        return
            fallbacks = ([[0, -1.5708, 0, -1.5708, 0, 0], [0, -1.2, 0.6, -1.5, 0, 0]] if self.n == 6 else
                         [[0, -0.6, 0, -2.2, 0, 1.6, 0.78], [0, -0.3, 0, -2.0, 0, 1.7, 0.78]])
            for qf in fallbacks:
                q = np.clip(np.array(qf, float), self.chain.lo, self.chain.hi)
                if not self.chain.collides(q, boxes):
                    full = np.array(self.art.get_joint_positions(), float)
                    full[self.idx] = q
                    self.art.set_joint_positions(full)
                    self.art.set_joint_velocities(np.zeros_like(full))
                    self.command(q)
                    print(f"[arm {self.name}] IK stow failed; set an upright stow pose instead: q={np.round(q, 2)}")
                    return
            print(f"[arm {self.name}] WARNING: no collision-free stow pose found; arm stays in its default pose")
        except Exception:
            print(f"[arm {self.name}] stow failed:")
            traceback.print_exc()

    def tune(self, world):
        """Make sure the joint drives really track targets; raise gains if they do not."""
        if not self.ok or self.art is None:
            return
        try:
            ctrl = self.art.get_articulation_controller()
            kps, kds = ctrl.get_gains()
            kps, kds = np.array(kps, float), np.array(kds, float)
            new_kd = np.maximum(kds[self.idx], 0.12 * kps[self.idx])     # critically-damped-ish: no wobble
            if np.any(new_kd > kds[self.idx] * 1.05):
                kds[self.idx] = new_kd
                ctrl.set_gains(kps=kps, kds=kds)
                print(f"[arm {self.name}] drive damping was too low for the stiffness (oscillation risk) -> damping now "
                      f"{np.round(new_kd, 1)} for stiffness {np.round(kps[self.idx], 0)}")
            for attempt in range(3):
                q0 = self.q().copy()
                tgt = q0.copy()
                j = 1 if self.n > 1 else 0
                tgt[j] += 0.15
                self.command(tgt)
                for _ in range(150):
                    world.step(render=False)
                got = float(self.q()[j] - q0[j])
                ratio = got / 0.15
                print(f"[arm {self.name}] drive test {attempt}: joint {j} moved {got:.3f} of 0.150 rad ({ratio * 100:.0f}%)")
                self.command(q0)
                for _ in range(150):
                    world.step(render=False)
                if ratio > 0.85:
                    return
                kps, kds = ctrl.get_gains()
                kps, kds = np.array(kps, float), np.array(kds, float)
                kps[self.idx] = np.maximum(kps[self.idx], 1.0) * 8.0
                kds[self.idx] = np.maximum(kds[self.idx], 1.0) * 3.0
                ctrl.set_gains(kps=kps, kds=kds)
                print(f"[arm {self.name}] joint tracking weak -> raised drive gains (x8 stiffness, x3 damping)")
        except Exception:
            print(f"[arm {self.name}] drive test failed:")
            traceback.print_exc()

    def q(self):
        if self.art is None:
            return np.zeros(self.n)
        v = self.art.get_joint_positions()
        return np.array(v)[self.idx].astype(float) if v is not None else (self._cmd if self._cmd is not None else np.zeros(self.n))

    def command(self, q):
        if self.art is None:
            return
        from isaacsim.core.utils.types import ArticulationAction
        self._cmd = np.asarray(q, float)
        self.art.apply_action(ArticulationAction(joint_positions=self._cmd, joint_indices=self.idx))

    def tip(self, q):
        return self.chain.tip(np.asarray(q, float))

    def ik(self, target, seed):
        return self.chain.ik(np.asarray(target, float), np.asarray(seed, float))

    def ik_candidates(self, target, seed, n=12):
        return self.chain.ik_candidates(np.asarray(target, float), np.asarray(seed, float), n=n)

    def collides(self, q, boxes, radius=0.05):
        return self.chain.collides(np.asarray(q, float), boxes, radius=radius, tool_radius=min(radius, 0.035))

    def path_free(self, q0, q1, boxes, carried=None, radius=0.05):
        return self.chain.path_free(q0, q1, boxes, carried=carried, radius=radius, tool_radius=min(radius, 0.035))

    def collision_info(self, q, boxes, radius=0.05):
        return self.chain.collision_info(np.asarray(q, float), boxes, radius=radius, tool_radius=min(radius, 0.035))

    def selfcheck(self):
        if not self.ok or self.art is None:
            return
        from pxr import Usd, UsdGeom
        T = self.chain.fk(self.q())
        m = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.info["ee_path"])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        err = np.linalg.norm(T[:3, 3] - np.array([t[0], t[1], t[2]]))
        print(f"[arm {self.name}] model-vs-sim EE position error: {err * 1000:.1f} mm "
              f"({'OK' if err < 0.01 else 'CHECK: model or USD write-back mismatch'})")


def arm_boxes_for(tag):
    """Axis-aligned boxes (world frame) an arm must keep clear of."""
    cfg = C.CONVEYORS[tag]
    y = cfg["y"]
    sgn = -1 if y > 0 else 1
    x0, x1 = C.BELT_START_X, C.BELT_START_X + C.BELT_LEN
    ped_y = y + sgn * C.ARM_OFFSET_Y
    stop_x = C.PICK_X + C.PKG_SIZE[0] / 2 + 0.02
    return [
        (np.array([x0, y - C.BELT_W / 2 - 0.04, 0.0]), np.array([x1, y + C.BELT_W / 2 + 0.04, C.BELT_H - 0.01])),       # belt + legs
        (np.array([x0, y - C.BELT_W / 2 - 0.04, C.BELT_H]), np.array([x1, y - C.BELT_W / 2, C.BELT_H + 0.10])),         # rail
        (np.array([x0, y + C.BELT_W / 2, C.BELT_H]), np.array([x1, y + C.BELT_W / 2 + 0.04, C.BELT_H + 0.10])),         # rail
        (np.array([stop_x - 0.015, y - C.BELT_W / 2, C.BELT_H]), np.array([stop_x + 0.015, y + C.BELT_W / 2, C.BELT_H + 0.20])),  # stopper
        (np.array([C.PICK_X - 0.175, ped_y - 0.175, 0.0]), np.array([C.PICK_X + 0.175, ped_y + 0.175, C.ARM_PEDESTAL_H - 0.02])),  # pedestal
    ]


ARM_BOX_NAMES = ["belt", "near rail", "far rail", "stopper", "pedestal", "carter deck"]


def crate_corridor(tag):
    """Volume swept by crates travelling on the belt - the stowed arm must stay out of it."""
    y = C.CONVEYORS[tag]["y"]
    L, D, H = C.PKG_SIZE
    return (np.array([C.BELT_START_X, y - D / 2 - 0.08, C.BELT_H]),
            np.array([C.PICK_X + L / 2 + 0.05, y + D / 2 + 0.08, C.BELT_H + H + 0.08]))


# ---------------------------------------------------------------------------------------------
class IsaacHW:
    def __init__(self, stage, carter_paths, fork_info, pallet_paths, arms):
        self.stage, self.arms = stage, arms
        self.carter_ops = {n: RootOps(stage.GetPrimAtPath(p)) for n, p in carter_paths.items()}
        self.fork_ops = {n: RootOps(stage.GetPrimAtPath(i["path"])) for n, i in fork_info.items()}
        self.fork_info = fork_info
        self.lifts = {n: LiftSetter(stage, i["lift_path"], i["scale"]) if i.get("lift_path") else None
                      for n, i in fork_info.items()}
        self.pallet_paths = pallet_paths
        self.n_spawn = {}
        self.yaw_off = C.CARTER_VISUAL_YAW_OFFSET_DEG

    def spawn_pkg(self, tag):
        n = self.n_spawn.get(tag, 0)
        self.n_spawn[tag] = n + 1
        L, D, H = C.PKG_SIZE
        y = C.CONVEYORS[tag]["y"]
        pos = (C.BELT_START_X + L / 2 + 0.15, y, C.BELT_H + H / 2 + 0.02)
        return IsaacPkg(make_crate(self.stage, f"/World/Packages/Crate{tag}_{n}", pos))

    def carter_set(self, name, x, y, yaw):
        self.carter_ops[name].set(x, y, C.CARTER_Z0, math.degrees(yaw) + self.yaw_off)

    def fork_set(self, name, x, y, lift):
        self.fork_ops[name].set(x, y, 0.0, self.fork_info[name]["yaw_deg"])
        if self.lifts[name] is not None:
            self.lifts[name].set(lift)

    def fork_geometry(self, name):
        g = C.FORK_GEOM.get(name)
        print(f"[hw] {name}: fork tip {g['tip']:.2f} m ahead of origin, fork top {g['surf']:.3f} m, thickness {g['thk']:.3f} m")
        return g

    def pallet_info(self, name):
        mn, mx = world_bbox(self.stage.GetPrimAtPath(self.pallet_paths[name]))
        return (float((mn[0] + mx[0]) / 2), float((mn[1] + mx[1]) / 2), float(mx[2]),
                float(mx[0] - mn[0]), float(mx[1] - mn[1]))

    def static_rects(self):
        return list(C.STATIC_RECTS)

    def arm_boxes(self, tag):
        return arm_boxes_for(tag)
