"""Build a kinematic chain (kin.Chain) for an arm directly from its USD joint prims, and mount a suction cup."""
import math
import numpy as np
from kin import Joint, Chain, T_of, quat_R, rot_axis

AXES = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}


def _np4(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T


def _frame(pos, rot):
    return T_of(quat_R(rot.GetReal(), *rot.GetImaginary()), np.array([pos[0], pos[1], pos[2]], float))


def build_chain(stage, robot_path, ee_candidates, suction_len):
    from pxr import Usd, UsdGeom, UsdPhysics
    robot = stage.GetPrimAtPath(robot_path)
    by_child, ee_path = {}, None
    for p in Usd.PrimRange(robot):
        if p.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(p)
            t1, t0 = j.GetBody1Rel().GetTargets(), j.GetBody0Rel().GetTargets()
            if not t1:
                continue
            kind = "rev" if p.IsA(UsdPhysics.RevoluteJoint) else ("fixed" if p.IsA(UsdPhysics.FixedJoint) else "other")
            by_child[str(t1[0])] = (p, kind, str(t0[0]) if t0 else None)
    for name in ee_candidates:
        for p in Usd.PrimRange(robot):
            if p.GetName() == name and p.IsA(UsdGeom.Xformable):
                ee_path = p.GetPath().pathString
                break
        if ee_path:
            break
    if ee_path is None:
        raise RuntimeError(f"no end-effector link among {ee_candidates}")
    chain_info, cur, base, has_root = [], ee_path, None, False
    while cur in by_child:
        p, kind, b0 = by_child[cur]
        if b0 is None:
            base, has_root = cur, True
            break
        if kind == "other":
            raise RuntimeError(f"unsupported joint type on arm chain: {p.GetPath()}")
        chain_info.append((p, kind))
        cur = b0
    if base is None:
        base = cur
    chain_info.reverse()
    joints, names = [], []
    for p, kind in chain_info:
        j = UsdPhysics.Joint(p)
        F0 = _frame(j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get())
        F1 = _frame(j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())
        axis, lo, hi = (0, 0, 1), -2 * math.pi, 2 * math.pi
        if kind == "rev":
            rj = UsdPhysics.RevoluteJoint(p)
            axis = AXES[str(rj.GetAxisAttr().Get())]
            l, h = rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()
            if l is not None and h is not None and h > l and abs(h - l) < 720:
                lo, hi = math.radians(l), math.radians(h)
            names.append(p.GetName())
        joints.append(Joint(p.GetName(), kind, F0, np.linalg.inv(F1), axis, lo, hi))
    base_prim = stage.GetPrimAtPath(base)
    T0 = _np4(UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(0))
    # tool axis = axis of the last revolute joint, expressed in the EE frame
    T, last_axis = T0.copy(), None
    for j in joints:
        T = T @ j.F0
        if j.kind == "rev":
            last_axis = T[:3, :3] @ j.axis
            T = T @ T_of(rot_axis(j.axis, 0.0), np.zeros(3))
        T = T @ j.F1inv
    axis_ee = T[:3, :3].T @ last_axis
    # extent of the EE geometry (fingers excluded) along that axis -> flange face
    ee_prim = stage.GetPrimAtPath(ee_path)
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    proj = []
    for p in Usd.PrimRange(ee_prim, Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Gprim) and "finger" not in p.GetPath().pathString.lower():
            r = cache.ComputeRelativeBound(p, ee_prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            lo_c, hi_c = r.GetMin(), r.GetMax()
            vals = [lo_c[0], lo_c[1], lo_c[2], hi_c[0], hi_c[1], hi_c[2]]
            if any((v != v) or abs(v) > 3.0 for v in vals):          # empty / infinite bounds (FLT_MAX) -> ignore
                continue
            for cx in (lo_c[0], hi_c[0]):
                for cy in (lo_c[1], hi_c[1]):
                    for cz in (lo_c[2], hi_c[2]):
                        proj.append(float(np.dot([cx, cy, cz], axis_ee)))
    emax, emin = (max(proj), min(proj)) if proj else (0.0823, 0.0)
    sign, mount = (1.0, emax) if emax >= -emin else (-1.0, -emin)
    note = ""
    if not (0.0 <= mount <= 0.30):                      # implausible flange distance -> typical UR/Franka flange offset
        note = f" (measured {mount:.3f} implausible, using 0.0823)"
        sign, mount = 1.0, 0.0823
    tool_dir = sign * axis_ee
    chain = Chain(T0, joints, tool_dir, mount + suction_len)
    info = dict(ee_path=ee_path, base=base, has_root=has_root, T0=T0, joint_prims=[p.GetPath().pathString for p, k in chain_info if k == 'rev'], dof_names=names, tool_dir=tool_dir, mount=mount,
                reach_dbg=f"axis_ee={np.round(axis_ee, 3)} sign={sign:+.0f} flange_at={mount:.3f}{note}")
    return chain, info


def hide_fingers(stage, robot_path):
    from pxr import Usd, UsdGeom
    for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path)):
        if "finger" in p.GetName().lower() and p.IsA(UsdGeom.Xformable):
            UsdGeom.Imageable(p).MakeInvisible()


def add_suction_cup(stage, ee_path, tool_dir, mount, cup_len):
    """Pneumatic cup: metal stem + black rubber lip, mounted on the flange along tool_dir (EE-local)."""
    from pxr import UsdGeom, Gf
    d = np.asarray(tool_dir, float)
    ax = int(np.argmax(np.abs(d)))
    token = "XYZ"[ax]
    sgn = 1.0 if d[ax] >= 0 else -1.0

    def cyl(name, h, r, off, rgb):
        c = UsdGeom.Cylinder.Define(stage, f"{ee_path}/SuctionCup/{name}")
        c.CreateAxisAttr(token)
        c.CreateHeightAttr(h)
        c.CreateRadiusAttr(r)
        v = [0.0, 0.0, 0.0]
        v[ax] = sgn * off
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*v))
        c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])

    UsdGeom.Xform.Define(stage, f"{ee_path}/SuctionCup")
    cyl("fitting", 0.020, 0.018, mount + 0.010, (0.75, 0.75, 0.78))
    cyl("stem", cup_len - 0.040, 0.009, mount + 0.020 + (cup_len - 0.040) / 2, (0.55, 0.55, 0.6))
    cyl("lip", 0.020, 0.030, mount + cup_len - 0.010, (0.05, 0.05, 0.05))


def rot_to_quat(R):
    """3x3 rotation -> (w, x, y, z)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def ensure_fixed_base(stage, robot_path, info):
    """If the asset has no joint tying its base to the world, add one so the arm can't fall or drift."""
    if info.get("has_root"):
        return "asset already has a root joint to the world"
    from pxr import UsdPhysics, Gf, Sdf
    T0 = info["T0"]
    j = UsdPhysics.FixedJoint.Define(stage, f"{robot_path}/world_fixed_joint")
    j.CreateBody1Rel().SetTargets([Sdf.Path(info["base"])])
    j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in T0[:3, 3]]))
    w, x, y, z = rot_to_quat(T0[:3, :3])
    j.CreateLocalRot0Attr(Gf.Quatf(float(w), float(x), float(y), float(z)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    return "added a fixed joint to the world (asset had none)"


def filter_arm_collisions(stage, arm_roots, crate_root="/World/Packages"):
    """Arms never physically touch crates (the planner already keeps them apart); a mis-posed arm can no
    longer jam a crate on the belt."""
    from pxr import UsdGeom, UsdPhysics, Usd, Sdf
    UsdGeom.Xform.Define(stage, crate_root)
    UsdGeom.Scope.Define(stage, "/World/CollisionGroups")
    arms = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/arms")
    crates = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/crates")
    for g, roots in ((arms, arm_roots), (crates, [crate_root])):
        col = Usd.CollectionAPI.Apply(g.GetPrim(), UsdPhysics.Tokens.colliders)
        for r in roots:
            col.CreateIncludesRel().AddTarget(Sdf.Path(r))
    arms.CreateFilteredGroupsRel().AddTarget(crates.GetPath())
    removed = 0
    for r in arm_roots:
        for p in Usd.PrimRange(stage.GetPrimAtPath(r)):
            try:
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    p.RemoveAPI(UsdPhysics.CollisionAPI)
                    removed += 1
            except Exception:
                pass
    return removed
