"""Asset placement helpers for Isaac Sim (shared by step3)."""
import math
import numpy as np


def asset_mpu(url):
    from pxr import Usd, UsdGeom
    try:
        st = Usd.Stage.Open(url, Usd.Stage.LoadNone)
        return float(UsdGeom.GetStageMetersPerUnit(st)) or 1.0
    except Exception as e:
        print(f"[assets] could not read units of {url}: {e}")
        return 1.0


def world_bbox(prim, render=False):
    """World bounds of `prim`.  `render=True` also includes render-purpose geometry, matching
    `measure()` so the returned bottom is the surface that is actually visible."""
    from pxr import UsdGeom
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render] if render else [UsdGeom.Tokens.default_]
    cache = UsdGeom.BBoxCache(0, purposes)
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.array(r.GetMin()), np.array(r.GetMax())


def strip_physics(prim):
    """Turn a decorative asset into pure visuals: no bodies, colliders, joints or articulation."""
    from pxr import Usd, UsdPhysics
    for p in list(Usd.PrimRange(prim)):
        try:
            tn = p.GetTypeName()
            if tn in ("OmniGraph", "OmniGraphNode") or p.GetName() in ("ActionGraph", "ROS_Graph") \
                    or (tn.startswith("Isaac") and tn.endswith("Sensor")):
                # drive/ROS graphs and physics sensors (the Nova Carter's IsaacImuSensor prims) would
                # look for the rigid bodies we just removed and flood the log at every reset
                p.SetActive(False)
        except Exception:
            pass
    for p in list(Usd.PrimRange(prim)):
        if p.IsA(UsdPhysics.Joint):
            try:
                p.SetActive(False)
            except Exception:
                pass
    for p in list(Usd.PrimRange(prim)):
        for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.MassAPI,
                    UsdPhysics.ArticulationRootAPI):
            try:
                if p.HasAPI(api):
                    p.RemoveAPI(api)
            except Exception:
                pass


def find_joint_body(prim, joint_name):
    """Path of the child body of a named joint (read before the joint is deactivated)."""
    from pxr import Usd, UsdPhysics
    for p in Usd.PrimRange(prim):
        if p.GetName() == joint_name and p.IsA(UsdPhysics.Joint):
            t = UsdPhysics.Joint(p).GetBody1Rel().GetTargets()
            if t:
                return str(t[0])
    return None


def place(stage, root, assets, prim_path, key, pos, yaw_deg=0.0, inert=False, expect=None, scale_override=None):
    """Reference an asset, scale by its declared units, optionally sanity-fix size, optionally strip physics.
    Returns (prim, scale_factor_applied)."""
    from pxr import Gf, UsdGeom
    from isaacsim.core.utils.stage import add_reference_to_stage
    url = f"{root}/{assets[key]}"
    prim = add_reference_to_stage(usd_path=url, prim_path=prim_path)
    mpu = asset_mpu(url) if scale_override is None else scale_override
    scale = mpu
    xf = UsdGeom.Xformable(prim)

    def apply(s):
        xf.ClearXformOpOrder()
        try:
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
            xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw_deg))
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(s, s, s))
        except Exception:
            # some props (PackingTable crates) ship their own float3 ops on the root prim, so a
            # double-precision op of the same name is rejected; fall back to the default precision
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
            xf.AddRotateZOp().Set(float(yaw_deg))
            xf.AddScaleOp().Set(Gf.Vec3d(s, s, s))

    apply(scale)
    if expect:
        lo, hi = expect
        mn, mx = world_bbox(prim)
        size = float(np.max(mx - mn))
        if size > 0 and not (lo <= size <= hi):
            k = 10.0 ** (-round(math.log10(size / math.sqrt(lo * hi))))
            scale = mpu * k
            apply(scale)
            mn, mx = world_bbox(prim)
            size = float(np.max(mx - mn))
            print(f"[assets] {key}: size corrected x{k:g}")
        print(f"[assets] {prim_path} <- {key} | scale {scale:g} | longest side {size:.2f} m")
    else:
        print(f"[assets] {prim_path} <- {key} | scale {scale:g}")
    return prim, scale


# ---------------------------------------------------------------------------------------------
# Off-screen measurement (opens the asset in its own stage; no physics, nothing is added to the scene)
# ---------------------------------------------------------------------------------------------
def _fit_scale(url, size_native, mpu, expect):
    if not expect:
        return mpu
    lo, hi = expect
    s = size_native * mpu
    if s > 0 and not (lo <= s <= hi):
        return mpu * 10.0 ** (-round(math.log10(s / math.sqrt(lo * hi))))
    return mpu


_MEASURE_CACHE = {}


def measure(url, expect=None):
    """Return dict(scale, gprims=[(path, min3, max3) in metres, asset frame], min, max)."""
    key = (url, expect)
    if key in _MEASURE_CACHE:
        return _MEASURE_CACHE[key]
    from pxr import Usd, UsdGeom
    st = Usd.Stage.Open(url)
    mpu = float(UsdGeom.GetStageMetersPerUnit(st)) or 1.0
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    root = st.GetDefaultPrim() or st.GetPseudoRoot()
    whole = cache.ComputeWorldBound(root).ComputeAlignedRange()
    size_native = float(max(whole.GetSize()))
    s = _fit_scale(url, size_native, mpu, expect)
    g = []
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Gprim):
            r = cache.ComputeWorldBound(p).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            g.append((p.GetPath().pathString, np.array(r.GetMin()) * s, np.array(r.GetMax()) * s))
    mn, mx = np.array(whole.GetMin()) * s, np.array(whole.GetMax()) * s
    _MEASURE_CACHE[key] = dict(scale=s, gprims=g, min=mn, max=mx, stage=st)
    return _MEASURE_CACHE[key]


def forklift_geometry(m):
    """Fork tines = long, thin, low gprims. Returns geometry in the forklift's own frame plus which way the forks point."""
    import re
    named = [(p, a, b) for p, a, b in m["gprims"] if re.search(r"(fork|tine|blade|tooth)(?!lift)", p.lower().replace("forklift", ""))
             and (b[0] - a[0]) > 0.4]
    tines = named or [(p, a, b) for p, a, b in m["gprims"]
                      if a[2] < 0.22 and (b[2] - a[2]) < 0.25 and (b[0] - a[0]) > 0.5 and (b[1] - a[1]) < 1.4]
    if not tines:
        low = sorted([(a[2], p, b - a) for p, a, b in m["gprims"] if (b[0] - a[0]) > 0.4], key=lambda t: t[0])[:8]
        print("[assets] forklift: no fork tines recognised. Lowest long parts (z_min, name, size):")
        for z, p, s in low:
            print(f"[assets]    z0={z:.3f}  {p.split('/')[-3:]}  size={np.round(s, 3)}")
        return None
    print(f"[assets] forklift tines: {[t[0].split('/')[-1] for t in tines][:6]}")
    mn = np.min([a for _, a, _ in tines], axis=0)
    mx = np.max([b for _, _, b in tines], axis=0)
    cx = float((mn[0] + mx[0]) / 2)
    forward = 1.0 if cx > 0 else -1.0          # +1: forks extend toward local +x
    tip = float(mx[0]) if forward > 0 else float(-mn[0])
    surf, thk = float(mx[2]), float(np.median([b[2] - a[2] for _, a, b in tines]))
    # A tine mesh is usually L-shaped (horizontal blade + vertical shank on the carriage), so its
    # bounding-box top is the top of the shank, not the blade the load rests on. Read the mesh
    # points and keep only the blade: everything ahead of the shank.
    blade = _blade_from_points(m, [p for p, _, _ in tines], forward)
    if blade is not None:
        if abs(blade[0] - surf) > 0.02:
            print(f"[assets] forklift: blade top {blade[0]:.3f} m, thickness {blade[1]:.3f} m "
                  f"(bounding box said {surf:.3f} / {thk:.3f} m: the shank, not the blade)")
        surf, thk = blade
    return dict(forward=forward, tip=tip, surf=surf, thk=thk,
                width=float(mx[1] - mn[1]), count=len(tines), length=float(mx[0] - mn[0]))


def _blade_from_points(m, paths, forward):
    """(top, thickness) of the horizontal fork blades from their mesh points, or None."""
    from pxr import Usd, UsdGeom
    st = m.get("stage")
    if st is None:
        return None
    xc = UsdGeom.XformCache(0)
    tops, bots = [], []
    for path in paths:
        p = st.GetPrimAtPath(path)
        if not p or not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        pts = np.asarray(pts, float)
        M = np.asarray(xc.GetLocalToWorldTransform(p), float)
        w = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3] * m["scale"]
        x = w[:, 0] * forward                              # distance along the fork direction
        heel, toe = float(x.min()), float(x.max())
        sel = w[x >= heel + 0.25 * (toe - heel)]           # skip the shank at the heel
        if len(sel) < 4:
            continue
        tops.append(float(sel[:, 2].max()))
        bots.append(float(sel[:, 2].min()))
    if not tops:
        return None
    top = float(np.median(tops))
    return top, max(top - float(np.median(bots)), 0.01)


def carter_geometry(m):
    g = m["gprims"]
    mn, mx = m["min"], m["max"]
    big = max(g, key=lambda t: (t[2][0] - t[1][0]) * (t[2][1] - t[1][1]) * min(1.0, (t[2][2] - t[1][2]) + 0.01)) if g else None
    chassis_top = float(big[2][2]) if big else float(mx[2])
    return dict(centre=((mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2), size=(float(mx[0] - mn[0]), float(mx[1] - mn[1])),
                top=float(mx[2]), chassis_top=chassis_top, bottom=float(mn[2]))


# ---------------------------------------------------------------------------------------------
# Carrier plate (flat load deck) on top of a carter, so every package sits level and on a known height
# ---------------------------------------------------------------------------------------------
def add_carrier_plate(stage, carter_path, size, thick, centre_local, z_top_local, post_from_z, yaw_comp_deg=0.0):
    from pxr import UsdGeom, Gf, UsdPhysics
    cx, cy = centre_local
    base = f"{carter_path}/LoadDeck"
    x = UsdGeom.Xform.Define(stage, base)
    xf = UsdGeom.Xformable(x)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
    xf.AddRotateZOp().Set(float(yaw_comp_deg))

    def cube(name, sz, pos, rgb):
        c = UsdGeom.Cube.Define(stage, f"{base}/{name}")
        c.GetSizeAttr().Set(1.0)
        f = UsdGeom.Xformable(c)
        f.AddTranslateOp().Set(Gf.Vec3d(*pos))
        f.AddScaleOp().Set(Gf.Vec3f(*sz))
        c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        return c

    L, W = size
    cube("plate", (L, W, thick), (0, 0, z_top_local - thick / 2), (0.30, 0.32, 0.36))
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        h = max(z_top_local - thick - post_from_z, 0.02)
        cube(f"post{i}", (0.05, 0.05, h), (sx * (L / 2 - 0.06), sy * (W / 2 - 0.06), post_from_z + h / 2), (0.75, 0.55, 0.1))
    # rubber pads so packages visibly sit on something
    cube("pad", (L * 0.9, W * 0.9, 0.004), (0, 0, z_top_local + 0.002), (0.08, 0.08, 0.09))
