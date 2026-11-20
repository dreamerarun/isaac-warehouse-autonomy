"""Dynamic warehouse people + moving props: NVIDIA SimReady characters that walk routes.

Each person is a referenced RL_BoneRoot character.  At build time the Isaac walk clips (which
match the `biped_demo` source rig 81/81) are retargeted onto that character with
`omni.anim.retarget.core` and bound as an in-place loop, so the mesh animates while a
`nav.Mobile` unicycle model drives its position along waypoint routes planned on their own
occupancy grid (walls, racks, belts, forklift aisles and keep-outs are all inflated).

The shuttle pushcart is returned in the same list as a moving prop, and the traffic cones are
registered as static obstacles in `C.STATIC_RECTS`, so *every* robot sees them.

Contract of each returned dict (consumed by logic.Sim and by step()):
    name, kind      label used in logs ("person" / "prop")
    mob             nav.Mobile (x, y, yaw, v) - the planner position
    path            remaining waypoints, [] while waiting, None when a fresh plan is needed
    goals           looping waypoint route; goal_idx advances ONLY on arrival
    r               planning radius (disk) of the obstacle
    prim            USD prim whose translate/rotateZ is written every step
    ops             (translate_op, rotateZ_op) handles of that prim
    yaw_off         visual yaw correction in degrees
    grid            the shared people occupancy grid
    z               floor offset written to the prim

`step(people, dt, t, obstacles=...)` advances everybody and writes their transforms; logic.Sim
yields to their disks (arbitrate / goto / _hard_blocked), which is what keeps every robot from
ever touching a person.
"""
import math, random
import numpy as np
import config as C
import assets as A
from nav import Grid, Mobile

_RETARGET_OK = None
_STAGE = None
_TL = None
_TL_WARNED = False
_STRIDE_M = 0.8     # metres of travel per stride cycle; re-measured from each baked clip
_CLIP_TC = 80.0             # fallback clip length in time codes; the real one is measured
_CLIP_S = _CLIP_TC / 60.0   # at 60 time codes per second -> seconds per stride cycle


# ---------------------------------------------------------------------------------------------
# retarget plumbing (validated by test_retarget.py)
# ---------------------------------------------------------------------------------------------
def _enable_retarget():
    global _RETARGET_OK
    if _RETARGET_OK is None:
        try:
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension("omni.anim.retarget.core")
            _RETARGET_OK = True
            print("[step3 people] omni.anim.retarget.core enabled", flush=True)
        except Exception as e:
            _RETARGET_OK = False
            print(f"[step3 people] retarget extension unavailable: {e}", flush=True)
    return _RETARGET_OK


def _find(root, tpe):
    from pxr import Usd
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == tpe:
            return p
    return None


def _bake_walk(stage, src_path, tgt_path, clip_path, parent):
    """Retarget `clip_path` from the biped source rig onto the character skeleton; returns the
    baked SkelAnimation path or None."""
    import omni.kit.commands
    from pxr import Usd
    stage.DefinePrim(parent, "Scope")
    res = omni.kit.commands.execute(
        "CreateRetargetAnimationsCommand",
        source_skeleton_path=src_path,
        target_skeleton_path=tgt_path,
        source_animation_paths=[clip_path],
        target_animation_parent_path=parent,
        set_root_identity=True,
    )
    if not ((isinstance(res, tuple) and res[0]) or res is True):
        print(f"[step3 people] bake failed: {res}", flush=True)
        return None
    from pxr import UsdSkel
    for p in Usd.PrimRange(stage.GetPrimAtPath(parent)):
        if p.GetTypeName() == "SkelAnimation":
            # the command can "succeed" yet leave an empty animation (no keyframes) - reject that.
            # Isaac bakes into translations/rotations, not the newer combined `transforms` attribute.
            anim = UsdSkel.Animation(p)
            counts = {}
            for name, meth in (("transforms", "GetTransformsAttr"),
                               ("translations", "GetTranslationsAttr"),
                               ("rotations", "GetRotationsAttr")):
                try:
                    get = getattr(anim, meth, None)     # older OpenUSD lacks `transforms`
                    a = get() if get is not None else None
                    counts[name] = len(a.GetTimeSamples()) if a else 0
                except Exception:
                    counts[name] = -1
            if not any(n > 0 for n in counts.values()):
                print(f"[step3 people] bake produced an empty SkelAnimation under {parent} "
                      f"{counts}", flush=True)
                return None
            return p.GetPath()
    print(f"[step3 people] bake produced no SkelAnimation under {parent}", flush=True)
    return None


def _joint_overlap(skel, clip_anim):
    """Fraction of the clip's joint names that also exist on the character skeleton."""
    from pxr import UsdSkel
    try:
        sj = [str(x) for x in (UsdSkel.Skeleton(skel).GetJointsAttr().Get() or [])]
        aj = [str(x) for x in (UsdSkel.Animation(clip_anim).GetJointsAttr().Get() or [])]
    except Exception:
        return 0.0
    if not aj or not sj:
        return 0.0
    sset = set(sj)
    sset |= {n.rsplit("/", 1)[-1].lstrip("*") for n in sj}
    hit = 0
    for n in aj:
        if n in sset or n.rsplit("/", 1)[-1].lstrip("*") in sset:
            hit += 1
    return hit / float(len(aj))


def _bind_animation(skel, anim_path):
    """Point the character's animation source at `anim_path` - on the Skeleton and, when it has
    one, on the enclosing SkelRoot so every renderer under it evaluates the same clip."""
    from pxr import UsdSkel
    rel = UsdSkel.BindingAPI.Apply(skel).CreateAnimationSourceRel()
    rel.SetTargets([anim_path])
    root = skel.GetParent()
    while root and root.IsValid() and root.GetTypeName() != "SkelRoot":
        root = root.GetParent()
    if root and root.IsValid() and root.GetTypeName() == "SkelRoot":
        try:
            UsdSkel.BindingAPI.Apply(root).CreateAnimationSourceRel().SetTargets([anim_path])
        except Exception:
            pass


def _disable_anim_graphs(asset):
    """The character may ship an AnimationGraph that would fight our directly-bound clip."""
    from pxr import Usd
    for p in Usd.PrimRange(asset.GetPrim()):
        try:
            if p.GetTypeName() == "AnimationGraph" or p.GetName() in ("AnimationGraph",
                                                                     "CharacterAnimation"):
                p.SetActive(False)
        except Exception:
            pass


def _gap_probe(stage, asset_prim, mesh_prim, baked_path):
    """One-off: linear-blend-skin the boot mesh across the clip and report the lowest world z
    so we can measure any gap between the feet and the floor."""
    from pxr import Usd, UsdSkel, UsdGeom, Gf
    sk = UsdSkel.BindingAPI(mesh_prim).GetSkeletonRel().GetTargets()
    if not sk:
        return
    skel = UsdSkel.Skeleton(stage.GetPrimAtPath(sk[0]))
    joints = [str(x) for x in (skel.GetJointsAttr().Get() or [])]
    rest = [Gf.Matrix4d(x) for x in (skel.GetRestTransformsAttr().Get() or [])]
    idx = {j: i for i, j in enumerate(joints)}
    parent = []
    for j in joints:
        pj = j.rsplit("/", 1)[0] if "/" in j else None
        parent.append(idx.get(pj, -1))
    pts = np.array([(p[0], p[1], p[2]) for p in UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()])
    ji = np.array(UsdGeom.PrimvarsAPI(mesh_prim).GetPrimvar("skel:jointIndices").Get()).reshape(-1, 3)
    jw = np.array(UsdGeom.PrimvarsAPI(mesh_prim).GetPrimvar("skel:jointWeights").Get()).reshape(-1, 3)
    try:
        geom_bind = UsdSkel.BindingAPI(mesh_prim).GetGeomBindTransformAttr().Get()
    except Exception:
        geom_bind = None
    gb = np.array(Gf.Matrix4d(geom_bind) if geom_bind else Gf.Matrix4d(1.0)).astype(float)
    # skel root world (constant)
    root = mesh_prim
    while root is not None and root.IsValid() and root.GetTypeName() != "SkelRoot":
        root = root.GetParent()
    if root is not None and root.IsValid():
        root_w = np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode(0))).astype(float)
        print(f"[gap] skelroot {root.GetPath()}", flush=True)
    else:
        root_w = np.eye(4)
        print("[gap] skelroot NOT FOUND", flush=True)

    def globals_from(locals_):
        g = []
        for i in range(len(locals_)):
            g.append(locals_[i] if parent[i] < 0 else g[parent[i]] @ locals_[i])
        return g

    rest_g = globals_from([np.array(m).astype(float) for m in rest])
    anim = UsdSkel.Animation(stage.GetPrimAtPath(baked_path))
    tattr, rattr = anim.GetTranslationsAttr(), anim.GetRotationsAttr()
    print(f"[gap] geomBind={np.round(gb,3).tolist()} rootW_diag={np.round(np.diag(root_w),3).tolist()} rootW_t={np.round(root_w[:3,3],3).tolist()}", flush=True)
    print(f"[gap] rest min world z = {(pts @ root_w[:3,:3].T + root_w[:3,3])[:,2].min():.4f}", flush=True)
    for tc in (0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 159.0):
        ts, rs = tattr.Get(tc), rattr.Get(tc)
        loc = []
        for i in range(len(joints)):
            q = rs[i]
            w, x, y, z = float(q.GetReal()), float(q.GetImaginary()[0]), float(q.GetImaginary()[1]), float(q.GetImaginary()[2])
            rr = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
            m = np.eye(4)
            m[:3, :3] = rr
            m[:3, 3] = (ts[i][0], ts[i][1], ts[i][2])
            loc.append(m)
        ag = globals_from(loc)
        skinX = [ag[i] @ np.linalg.inv(rest_g[i]) for i in range(len(joints))]
        base = (gb[:3,:3] @ pts.T).T + gb[:3,3]
        out = np.zeros_like(base)
        for k in range(3):
            for v in range(len(pts)):
                j = int(ji[v, k]); w = float(jw[v, k])
                if w:
                    out[v] += w * (skinX[j][:3,:3] @ base[v] + skinX[j][:3,3])
        world_z = (out @ root_w[:3,:3].T + root_w[:3,3])[:,2]
        print(f"[gap] tc={int(tc):3d} min world z = {world_z.min():.4f}", flush=True)


def _stride_probe(stage, skel_prim, baked_path):
    """Measure the walk clip's real step length from the foot joints, so the travel speed can be
    matched to the animation instead of guessed."""
    from pxr import Usd, UsdSkel, Gf
    skel = UsdSkel.Skeleton(skel_prim)
    joints = [str(x) for x in (skel.GetJointsAttr().Get() or [])]
    rest = [Gf.Matrix4d(x) for x in (skel.GetRestTransformsAttr().Get() or [])]
    idx = {j: i for i, j in enumerate(joints)}
    parent = []
    for j in joints:
        pj = j.rsplit("/", 1)[0] if "/" in j else None
        parent.append(idx.get(pj, -1))
    feet = [i for i, j in enumerate(joints)
            if any(k in j.lower() for k in ("foot", "ankle", "toe", "ball"))]
    if not feet:
        print("[stride] no foot joints found", flush=True)
        return
    anim = UsdSkel.Animation(stage.GetPrimAtPath(baked_path))
    ta, ra = anim.GetTranslationsAttr(), anim.GetRotationsAttr()
    tcs = [float(x) for x in ta.GetTimeSamples()]
    span = tcs[-1] - tcs[0]
    tracks = {i: [] for i in feet}
    for tc in tcs:
        ts, rs = ta.Get(tc), ra.Get(tc)
        loc = []
        for i in range(len(joints)):
            q = rs[i]
            w, x, y, z = (float(q.GetReal()), float(q.GetImaginary()[0]),
                          float(q.GetImaginary()[1]), float(q.GetImaginary()[2]))
            rr = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
            m = np.eye(4)
            m[:3, :3] = rr
            m[:3, 3] = (ts[i][0], ts[i][1], ts[i][2])
            loc.append(m)
        g = []
        for i in range(len(loc)):
            g.append(loc[i] if parent[i] < 0 else g[parent[i]] @ loc[i])
        for i in feet:
            tracks[i].append((float(g[i][0, 3]), float(g[i][1, 3]), float(g[i][2, 3])))
    global _CLIP_S, _STRIDE_M
    _CLIP_S = span / 60.0
    best = 0.0
    for i in feet:
        ps = tracks[i]
        e = 0.0
        for a in ps:
            for b in ps:
                d = math.hypot(a[0] - b[0], a[1] - b[1])
                if d > e:
                    e = d
        best = max(best, e)
        if e > 0.05 and __import__("os").environ.get("STRIDEDBG"):
            print(f"[stride] {joints[i]}: horizontal swing {e:.3f} m, "
                  f"x {max(p[0] for p in ps) - min(p[0] for p in ps):.3f} "
                  f"y {max(p[1] for p in ps) - min(p[1] for p in ps):.3f}, "
                  f"z dip {min(p[2] for p in ps):.3f}", flush=True)
    _STRIDE_M = 2.0 * best
    if __import__("os").environ.get("STRIDEDBG"):
        print(f"[stride] step length {best:.3f} m -> stride {_STRIDE_M:.3f} m/cycle -> "
              f"speed {_STRIDE_M / _CLIP_S:.3f} m/s (cycle {_CLIP_S:.3f} s)", flush=True)
    return _STRIDE_M
    # which way does the swinging foot travel? (it goes forward while it is lifted)
    if __import__("os").environ.get("STRIDEDBG"):
        probe = max(feet, key=lambda i: max(p[1] for p in tracks[i]) - min(p[1] for p in tracks[i]))
        ps = tracks[probe]
        lift = [i for i, p in enumerate(ps) if p[2] > min(q[2] for q in ps) + 0.04]
        if lift:
            y0, y1 = ps[lift[0]][1], ps[lift[-1]][1]
            print(f"[stride] {joints[probe].split('/')[-1]} in swing: y {y0:+.3f} -> "
                  f"{y1:+.3f} => rig forward is {'+Y' if y1 > y0 else '-Y'}", flush=True)

def _read_clip(stage, path):
    """Pull a baked SkelAnimation out as numpy arrays: (times, translations, rotations, scales)."""
    from pxr import UsdSkel
    anim = UsdSkel.Animation(stage.GetPrimAtPath(path))
    ta, ra = anim.GetTranslationsAttr(), anim.GetRotationsAttr()
    sa = anim.GetScalesAttr()
    tcs = [float(x) for x in ta.GetTimeSamples()]
    T = np.array([[list(v) for v in ta.Get(tc)] for tc in tcs], dtype=float)
    Q = np.array([[(q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2])
                   for q in ra.Get(tc)] for tc in tcs], dtype=float)
    S = None
    if sa and len(sa.GetTimeSamples()):
        stcs = [float(x) for x in sa.GetTimeSamples()]
        if len(stcs) == len(tcs):
            S = np.array([[list(v) for v in sa.Get(tc)] for tc in stcs], dtype=float)
    return tcs, T, Q, S


def _write_clip(stage, path, joints, tcs, T, Q, S=None):
    """Author a new SkelAnimation prim with the given time-sampled joint transforms."""
    from pxr import UsdSkel, Gf, Vt
    anim = UsdSkel.Animation(stage.DefinePrim(path, "SkelAnimation"))
    anim.GetJointsAttr().Set(joints)
    ta, ra = anim.GetTranslationsAttr(), anim.GetRotationsAttr()
    sa = anim.GetScalesAttr() if S is not None else None
    for k, tc in enumerate(tcs):
        ta.Set(Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in T[k]]),
               tc)
        ra.Set(Vt.QuatfArray([Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]),
                                                          float(q[3]))) for q in Q[k]]), tc)
        if sa is not None:
            sa.Set(Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in S[k]]),
                   tc)
    return anim


def _sample_clip(tcs, T, Q, phases):
    """Linearly resample a looping clip at the requested time codes (quaternions nlerp-ed)."""
    n = len(tcs)
    outT = np.empty((len(phases), T.shape[1], 3))
    outQ = np.empty((len(phases), Q.shape[1], 4))
    for k, ph in enumerate(phases):
        i0 = min(max(int(math.floor(ph)), 0), n - 2)
        f = min(max(ph - tcs[i0], 0.0), 1.0)
        outT[k] = T[i0] * (1.0 - f) + T[i0 + 1] * f
        q0, q1 = Q[i0], Q[i0 + 1].copy()
        q1[(q0 * q1).sum(axis=1) < 0.0] *= -1.0         # shortest arc, same hemisphere
        q = q0 * (1.0 - f) + q1 * f
        outQ[k] = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-9)
    return outT, outQ


def _phase_clip(stage, src_path, dst_path, offset_tc):
    """Copy a walk clip with its whole cycle shifted by `offset_tc` time codes.

    Every character shares the one omni-timeline playhead, so an unshifted copy would make all
    of them step in lockstep.  Shifting the samples gives each person their own phase without
    touching the timeline."""
    from pxr import UsdSkel
    src = UsdSkel.Animation(stage.GetPrimAtPath(src_path))
    joints = src.GetJointsAttr().Get()
    tcs, T, Q, S = _read_clip(stage, src_path)
    span = tcs[-1] - tcs[0]
    phases = [((tc - offset_tc - tcs[0]) % span) + tcs[0] for tc in tcs]
    # the wrap sample must land exactly on the first phase (0 % span) so the loop stays closed
    phases[-1] = tcs[0]
    outT, outQ = _sample_clip(tcs, T, Q, phases)
    _write_clip(stage, dst_path, joints, list(tcs), outT, outQ, S)
    return dst_path


def _stand_clip(stage, src_path, dst_path):
    """Build a one-pose 'standing still' clip by averaging the walk cycle.

    Averaging a symmetric gait cancels the stride, so the result stands with the feet under the
    body and the arms hanging - what a person does while waiting for a carter to pass."""
    from pxr import UsdSkel
    src = UsdSkel.Animation(stage.GetPrimAtPath(src_path))
    joints = src.GetJointsAttr().Get()
    tcs, T, Q, S = _read_clip(stage, src_path)
    q0 = Q[0]
    Qa = Q.copy()
    for k in range(len(Qa)):                          # same hemisphere before averaging
        Qa[k][(q0 * Qa[k]).sum(axis=1) < 0.0] *= -1.0
    qm = Qa.mean(axis=0)
    qm /= np.maximum(np.linalg.norm(qm, axis=1, keepdims=True), 1e-9)
    Tm = T.mean(axis=0)
    Sm = S[0] if S is not None else None
    _write_clip(stage, dst_path, joints, [tcs[0], tcs[-1]], Tm[None].repeat(2, 0),
                qm[None].repeat(2, 0), None if Sm is None else Sm[None].repeat(2, 0))
    return dst_path


# ---------------------------------------------------------------------------------------------
# placement helpers
# ---------------------------------------------------------------------------------------------
def _reference(stage, root, rel, path, expect=None):
    """Person/prop prim we own (clean xform ops) + a child prim carrying the asset reference.

    The child prim is scaled to the asset's declared units and shifted so that, inside the
    wrapper, the asset's footprint centre sits on (0, 0) and its lowest point on z = 0.  The
    bounds are measured off-screen exactly like statics._real does, so a cm-authored asset can
    never end up 100x too big and a character never floats or sinks into the floor.

    Returns (prim, asset, measure); all None when the asset could not be measured - the caller
    must then skip it (no geometry, therefore no obstacle either)."""
    from pxr import UsdGeom, Gf
    prim = stage.DefinePrim(path, "Xform")
    asset = stage.DefinePrim(path + "/Asset")
    url = f"{root}/{rel}"
    try:
        m = A.measure(url, expect)
        size = m["max"] - m["min"]
        if not np.all(np.isfinite(size)) or float(size.max()) <= 0.01:
            raise RuntimeError(f"implausible bounds {np.round(size, 3)}")
    except Exception as e:
        print(f"[step3 people] {rel.rsplit('/', 1)[-1]}: cannot measure it ({e}) - skipped", flush=True)
        stage.RemovePrim(path)
        return None, None, None
    asset.GetReferences().AddReference(url)
    try:
        asset.SetInstanceable(False)       # every person needs its own skeleton to retarget onto
    except Exception:
        pass
    c = (m["min"] + m["max"]) / 2.0
    s = float(m["scale"])
    xf = UsdGeom.Xformable(asset)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(float(-c[0]), float(-c[1]), float(-m["min"][2])))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(s, s, s))
    return prim, asset, m


def _place(prim, x, y, yaw_deg, z=0.0):
    """Give the prim fresh translate/rotateZ ops and return them for cheap per-step updates."""
    from pxr import UsdGeom, Gf
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    t_op.Set(Gf.Vec3d(float(x), float(y), float(z)))
    r_op = xf.AddRotateZOp()
    r_op.Set(float(yaw_deg))
    return t_op, r_op


def _free_spot(grid, rng, min_d=0.0, others=()):
    hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
    for _ in range(500):
        x = rng.uniform(-hx + 1.5, hx - 1.5)
        y = rng.uniform(-hy + 1.5, hy - 1.5)
        if not grid.free_point(x, y):
            continue
        if any(math.hypot(x - a, y - b) < min_d for a, b in others):
            continue
        return (x, y)
    return None


# ---------------------------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------------------------
def build(stage, root, keep_out=(), seed=0):
    """Spawn the people, the shuttle pushcart and the traffic cones.
    Returns the dynamic-obstacle list for logic.Sim."""
    global _STAGE, _TL
    _STAGE = stage
    try:
        import omni.timeline
        _TL = omni.timeline.get_timeline_interface()
    except Exception as e:
        _TL = None
        print(f"[step3 people] timeline unavailable, walk animation frozen: {e}", flush=True)

    rng = random.Random(seed)
    hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2

    # ---- occupancy grid for the people: walls + keep-outs + every registered static prop --------
    wall_rects = [(-hx - 1, -hx, -hy - 1, hy + 1), (hx, hx + 1, -hy - 1, hy + 1),
                  (-hx - 1, hx + 1, hy, hy + 1), (-hx - 1, hx + 1, -hy - 1, -hy)]

    def new_grid():
        g = Grid(-hx, hx, -hy, hy, 0.1)
        rects = wall_rects + [tuple(r) for r in keep_out] + [tuple(r) for r in C.STATIC_RECTS]
        g.build(rects, C.PEOPLE_R, C.PEOPLE_R)
        return g

    grid = new_grid()

    def reachable(a, b):
        """A route exists on the static layout (used to reject waypoints in sealed pockets)."""
        return grid.plan(a, b, (), 0.9, 0.6) is not None

    print(f"[step3 people] nav rects: keep-out {len(keep_out)}, static {len(C.STATIC_RECTS)}",
          flush=True)
    for r in list(keep_out) + list(C.STATIC_RECTS):
        print(f"[step3 people]   rect ({r[0]:6.2f},{r[2]:6.2f}) .. ({r[1]:6.2f},{r[3]:6.2f})", flush=True)

    # ---- traffic cones: static obstacles, registered so robots and people both see them ---------
    for i in range(C.PEOPLE_CONES):
        spot = _free_spot(grid, rng, min_d=1.5)
        if spot is None:
            break
        prim, asset, m = _reference(stage, root, C.CONE_ASSET, f"/World/People/Cone_{i}", C.CONE_SIZE)
        if prim is None:
            continue                        # not measurable -> no geometry -> no obstacle either
        A.strip_physics(asset)
        _place(prim, spot[0], spot[1], rng.uniform(0, 360))
        half = 0.5 * float(max(m["max"][0] - m["min"][0], m["max"][1] - m["min"][1]))
        rect = (spot[0] - half, spot[0] + half, spot[1] - half, spot[1] + half)
        C.STATIC_RECTS.append(rect)
        print(f"[step3 people] traffic cone {float(m['max'][2] - m['min'][2]):.2f} m at "
              f"({spot[0]:.1f}, {spot[1]:.1f}) footprint {rect[1] - rect[0]:.2f} x "
              f"{rect[3] - rect[2]:.2f} m", flush=True)
    if C.PEOPLE_CONES:
        grid = new_grid()               # rebuild so the cones become obstacles too

    # ---- characters: retarget a walk clip onto each one ----------------------------------------
    people = []
    ok = _enable_retarget()
    src_skel = clip_anim = None
    if ok:
        src_prim = stage.DefinePrim("/World/People/_Src")
        src_prim.GetReferences().AddReference(f"{root}/{C.BIPED_RIG}")
        clip_prim = stage.DefinePrim("/World/People/_Clip")
        clip_prim.GetReferences().AddReference(f"{root}/{C.WALK_CLIPS[0]}")
        src_skel, clip_anim = _find(src_prim, "Skeleton"), _find(clip_prim, "SkelAnimation")
        if src_skel is None or clip_anim is None:
            ok = False
            print("[step3 people] source rig or walk clip missing - people will stand still",
                  flush=True)
        else:
            from omni.anim.retarget.core import auto_setup
            auto_setup("Human", src_skel.GetPath())      # tags + facing for the source, once

    spawns = []
    for i in range(C.PEOPLE_N):
        spot = _free_spot(grid, rng, min_d=2.0, others=spawns)
        if spot is None:
            print("[step3 people] no free spot left for another person", flush=True)
            break
        spawns.append(spot)
        rel = C.PEOPLE_ASSETS[i % len(C.PEOPLE_ASSETS)]
        prim, asset, m = _reference(stage, root, rel, f"/World/People/Person_{i}", C.PERSON_SIZE)
        if prim is None:
            continue                        # unmeasurable asset: no mesh, no obstacle, no person
        A.strip_physics(asset)                          # pure visuals: no bodies, no ragdoll
        height = float(m["max"][2] - m["min"][2])
        clip_path = None
        stand_path = None
        skel_prim = None
        if ok:
            try:
                skel = _find(asset.GetPrim(), "Skeleton")
                clip_i = stage.DefinePrim(f"/World/People/_Clip{i}")
                # the in-place loop is the clip the retarget recipe was validated on; the
                # stand_walk_1..3 alternates bake but render some rigs in a broken pose
                clip_i.GetReferences().AddReference(f"{root}/{C.WALK_CLIPS[0]}")
                clip_a = _find(clip_i, "SkelAnimation")
                walk_speed = C.PEOPLE_SPEED
                if skel is None or clip_a is None:
                    print(f"[step3 people] person {i}: no skeleton or walk clip - it will stand "
                          f"still", flush=True)
                else:
                    skel_prim = skel
                    from omni.anim.retarget.core import auto_setup
                    auto_setup("Human", skel.GetPath())
                    baked = _bake_walk(stage, str(src_skel.GetPath()), str(skel.GetPath()),
                                       str(clip_a.GetPath()), f"/World/People/Baked_{i}")
                    if baked is not None:
                        _bind_animation(skel, baked)
                        _disable_anim_graphs(asset)
                        clip_path = str(baked)
                        print(f"[step3 people] person {i}: walk clip bound ({baked.name})",
                              flush=True)
                        try:
                            # walk exactly as fast as this clip's own stride, so the feet never
                            # slide on the floor (each rig bakes a slightly different stride)
                            walk_speed = _stride_probe(stage, skel, baked) / _CLIP_S
                        except Exception as e:
                            print(f"[stride] probe failed: {e}", flush=True)
                        # every character gets its own animation sources: a phase-shifted walk
                        # loop (no lockstep marching) and a standing pose for when it is blocked
                        try:
                            pdir = f"/World/People/Baked_{i}"
                            off = (i % max(C.PEOPLE_N, 1)) * _CLIP_S * 60.0 / max(C.PEOPLE_N, 1)
                            clip_path = _phase_clip(stage, str(baked), f"{pdir}/_Walk{i}", off)
                            stand_path = _stand_clip(stage, str(baked), f"{pdir}/_Stand{i}")
                            _bind_animation(skel, clip_path)
                            print(f"[step3 people] person {i}: own walk source {clip_path} "
                                  f"(phase -{off / 60.0:.2f} s) + stand source {stand_path}",
                                  flush=True)
                        except Exception as e:
                            print(f"[step3 people] person {i}: per-person clip failed ({e}); "
                                  f"keeping the shared walk clip", flush=True)
                            stand_path = None
                    else:
                        # retarget can fail silently on an unfamiliar rig: only bind the clip
                        # straight to the skeleton when the joint sets are close enough to match
                        ov = _joint_overlap(skel, clip_a)
                        if ov >= 0.6:
                            _bind_animation(skel, str(clip_a.GetPath()))
                            _disable_anim_graphs(asset)
                            print(f"[step3 people] person {i}: fallback - clip bound directly "
                                  f"({ov:.0%} joint match)", flush=True)
                        else:
                            print(f"[step3 people] person {i}: retarget failed and joint match "
                                  f"only {ov:.0%} - it will stand still", flush=True)
            except Exception as e:
                print(f"[step3 people] person {i}: retarget failed ({e}); it will stand still",
                      flush=True)

        x, y = spot
        yaw = rng.uniform(-math.pi, math.pi)
        t_op, r_op = _place(prim, x, y, math.degrees(yaw) + C.PERSON_YAW_OFFSET_DEG, C.PERSON_Z0)

        goals, tries = [], 0
        while len(goals) < C.PEOPLE_GOALS and tries < 200:
            tries += 1
            g = _free_spot(grid, rng, min_d=2.0, others=[spot] + goals)
            if g is None:
                break
            if math.hypot(g[0] - x, g[1] - y) < 3.0 or not reachable((x, y), g):
                continue
            goals.append(g)
        if not goals:
            goals = [spot]                              # nowhere to go: it just stands there

        people.append(dict(name=f"person{i}", kind="person",
                           mob=Mobile(x, y, yaw, walk_speed, C.PEOPLE_WMAX, amax=1.0),
                           path=None, goals=goals, goal_idx=0, r=C.PEOPLE_R,
                           prim=prim, ops=(t_op, r_op), yaw_off=C.PERSON_YAW_OFFSET_DEG,
                           grid=grid, plan_t=-99.0, z=C.PERSON_Z0, clip_path=clip_path,
                           skel=skel_prim, stand_path=stand_path, anim_state="walk",
                           stop_t=0.0, blocks=0, blocked_s=0.0))
        print(f"[step3 people] person{i}: {rel.rsplit('/', 1)[-1]} at ({x:.1f}, {y:.1f}), "
              f"{len(goals)} waypoint route", flush=True)

    # ---- shuttle pushcart: a moving warehouse prop with the same disk contract -------------------
    lane = _free_spot(grid, rng, min_d=2.0, others=spawns)
    if lane is not None:
        prim, asset, m = _reference(stage, root, C.PUSHCART_ASSET, "/World/People/Pushcart", C.PROP_SIZE)
        if prim is not None:
            A.strip_physics(asset)
            other = None
            for d in (6.0, 5.0, 4.0, 3.0):             # prefer a long straight aisle
                for dx, dy in ((d, 0.0), (-d, 0.0), (0.0, d), (0.0, -d)):
                    cand = (lane[0] + dx, lane[1] + dy)
                    if grid.free_point(*cand) and reachable(lane, cand):
                        other = cand
                        break
                if other is not None:
                    break
            if other is None:
                other = _free_spot(grid, rng, min_d=3.0, others=[lane])
            if other is not None:
                t_op, r_op = _place(prim, lane[0], lane[1], rng.uniform(0, 360))
                rad = 0.5 * math.hypot(float(m["max"][0] - m["min"][0]),
                                       float(m["max"][1] - m["min"][1])) + 0.05
                people.append(dict(name="pushcart", kind="prop",
                                   mob=Mobile(lane[0], lane[1], 0.0, C.PUSHCART_SPEED, 1.2, amax=0.5),
                                   path=None, goals=[other, lane], goal_idx=0, r=rad,
                                   prim=prim, ops=(t_op, r_op), yaw_off=C.PUSHCART_YAW_OFFSET_DEG,
                                   grid=grid, plan_t=-99.0, z=0.0))
                print(f"[step3 people] pushcart shuttles ({lane[0]:.1f}, {lane[1]:.1f}) <-> "
                      f"({other[0]:.1f}, {other[1]:.1f}), disk r={rad:.2f} m", flush=True)
            else:
                print("[step3 people] no shuttle lane free - pushcart skipped", flush=True)

    # ---- clip timing: the timeline plays the walk loop while logic moves the people ----------------
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(_CLIP_S * 60.0)        # measured clip length, not a guessed 160
    if _TL is not None:
        try:
            _TL.set_time_codes_per_second(60.0)
            _TL.set_start_time(0.0)
            _TL.set_end_time(_CLIP_S)
            # keep the app timeline running (world.step needs it) but step() re-pins the
            # playhead to sim time every frame so the walk loop can never drift or stall
            _TL.play(0.0, _CLIP_S, True)
            _TL.set_current_time(0.0)
        except Exception as e:
            print(f"[step3 people] could not set up the walk loop: {e}", flush=True)

    # the retarget source rig + clip prims are build scaffolding, not scenery: hide the source
    # body (rest pose, untextured) so it never lingers in the warehouse looking like a fallen person
    for helper in ("/World/People/_Src",):
        pr = stage.GetPrimAtPath(helper)
        if pr and pr.IsValid():
            try:
                pr.SetActive(False)
            except Exception:
                pass

    n_walk = sum(1 for p in people if p["kind"] == "person")
    print(f"[step3 people] {n_walk} walking people, {len(people) - n_walk} moving props, "
          f"{C.PEOPLE_CONES} cones on the floor", flush=True)
    return people


def step(people, dt, t, obstacles=None):
    """Advance every person/prop along its looping route and write its prim transform.

    A route is (re)planned whenever there is no path yet, after arrival, or when another
    dynamic obstacle is close enough to invalidate the remaining waypoints."""
    if not people:
        return
    if _TL is not None:
        try:
            # pin the playhead to simulated time: the app timeline advances at wall-clock
            # rate and would otherwise cycle the legs faster/slower than the body moves.
            # set_current_time is applied on the next app update, i.e. the next world.step().
            _TL.set_current_time(float(t) % _CLIP_S)
            if __import__("os").environ.get("ANIMDBG") and int(t * 60) % 30 == 0:
                print(f"[anim] t={t:.2f} tl={_TL.get_current_time():.3f} "
                      f"playing={_TL.is_playing()}", flush=True)
        except Exception:
            pass
    from pxr import UsdGeom, Gf
    # Drive the USD playhead from sim time so the bound walk clips actually animate.  In this
    # script-driven loop nothing else advances the omni timeline, so without this the people
    # slide along the floor in their rest pose.

    obs = list(obstacles() if callable(obstacles) else (obstacles or []))
    dyn = [(p["mob"].x, p["mob"].y, p["r"]) for p in people]
    for i, p in enumerate(people):
        mob, goals = p["mob"], p["goals"]
        gi = p["goal_idx"] % len(goals)
        goal = goals[gi]
        # carter disks supplied by the caller plus the other people/props (never ourselves)
        others = obs + [d for j, d in enumerate(dyn) if j != i]
        # a disk that already contains this person cannot be steered around: drop it so the
        # planner can walk out of a passing carter instead of seeing every neighbour blocked
        disks = [(dx, dy, dr + p["r"]) for dx, dy, dr in others
                 if math.hypot(mob.x - dx, mob.y - dy) > dr + p["r"]]
        near = any(math.hypot(mob.x - dx, mob.y - dy) < C.PEOPLE_REPLAN_D for dx, dy, _ in others)
        due = t - p["plan_t"] >= (1.0 if p["path"] else 0.5)
        if due and (not p["path"] or near):
            path = p["grid"].plan((mob.x, mob.y), goal, disks, 0.9, 0.6)
            p["plan_t"] = t
            if path:
                p["path"] = path
                p["plan_fails"] = 0
            elif path is not None:                      # [] -> already standing on the goal
                p["goal_idx"] = (gi + 1) % len(goals)
                p["path"] = None
                p["plan_t"] = -99.0
            else:                                       # no route: retry, then take the next waypoint
                p["plan_fails"] = p.get("plan_fails", 0) + 1
                if p["plan_fails"] in (1, 3):
                    clear = p["grid"].plan((mob.x, mob.y), goal, (), 0.9, 0.6) is not None
                    why = "dynamic disks" if clear else "static layout"
                    print(f"[step3 people] {p['name']}: no route from ({mob.x:.1f}, {mob.y:.1f}) "
                          f"to ({goal[0]:.1f}, {goal[1]:.1f}) via {why}"
                          f"{' - next waypoint' if p['plan_fails'] >= 3 else ', retrying'}", flush=True)
                if p["plan_fails"] >= 3:
                    p["plan_fails"] = 0
                    p["goal_idx"] = (gi + 1) % len(goals)
                    p["path"] = None
                    p["plan_t"] = -99.0
        wps = p["path"]
        old_xy = (mob.x, mob.y)
        reached = bool(wps) and mob.track(wps[0], dt, move=True, tol=0.35)
        # never step *into* a robot/prop disk: the people have right of way, so a stopped
        # carter waits while the walker goes around it instead of overlapping it visually
        blocked = False
        p["block_reason"] = ""
        if not p["grid"].free_point(mob.x, mob.y):     # stepped too close to a rack/wall/cone
            mob.x, mob.y, mob.v = old_xy[0], old_xy[1], 0.0
            blocked = True
            p["block_reason"] = "static"
        for di, (dx, dy, dr) in enumerate(others):
            if blocked:
                break
            rr = dr + p["r"] + 0.03          # keep a small clearance, never exactly touching
            if (math.hypot(mob.x - dx, mob.y - dy) < rr
                    and math.hypot(old_xy[0] - dx, old_xy[1] - dy) >= rr):
                mob.x, mob.y, mob.v = old_xy[0], old_xy[1], 0.0
                blocked = True
                p["block_reason"] = f"dyn{di}@{dx:.1f},{dy:.1f}r{dr:.2f}"
                break
        # people have right-of-way: if this person would overlap another obstacle,
        # stop in place and let carters/other walkers go around.  No sideways detour –
        # carters wait for walkers (priority 99) so walkers never need to swerve.
        if blocked:
            # stop and then advance to the next waypoint instead of staying glued
            mob.x, mob.y, mob.v = old_xy[0], old_xy[1], 0.0
            p["goal_idx"] = (gi + 1) % len(goals)
            p["path"] = None
            p["plan_t"] = -99.0
        else:
            p["stuck_t"] = 0.0
            # keep previous velocity if we were already moving; otherwise stay stopped
            if mob.v == 0.0:
                mob.v = 0.0

        if reached and not blocked:
            wps.pop(0)
            if not wps:
                p["goal_idx"] = (gi + 1) % len(goals)
                p["path"] = None
                p["plan_t"] = -99.0
        t_op, r_op = p["ops"]
        t_op.Set(Gf.Vec3d(float(mob.x), float(mob.y), float(p["z"])))
        r_op.Set(float(math.degrees(mob.yaw) + p["yaw_off"]))
        # ---- animation source per person: the walk loop while it moves, the standing pose while
        # a carter/neighbour blocks it.  Without this the shared timeline keeps cycling its legs
        # and the person moonwalks in place.
        if blocked:
            p["blocks"] = p.get("blocks", 0) + 1
            p["blocked_s"] = p.get("blocked_s", 0.0) + dt
            p["stop_t"] = p.get("stop_t", 0.0) + dt
            if p["stop_t"] > p.get("max_block_s", 0.0):
                p["max_block_s"] = p["stop_t"]
        else:
            p["stop_t"] = 0.0
        if p.get("skel") and p.get("stand_path") and p.get("clip_path"):
            want = "stand" if (p["stop_t"] > 0.03 or p.get("force_stand")) else "walk"
            if want != p.get("anim_state", "walk"):
                try:
                    _bind_animation(p["skel"],
                                    p["stand_path"] if want == "stand" else p["clip_path"])
                    p["anim_state"] = want
                    if __import__("os").environ.get("ANIMDBG"):
                        print(f"[anim] {p['name']}: {p['anim_state'] if False else want} "
                              f"(blocked {p.get('stop_t', 0.0):.2f} s)", flush=True)
                except Exception:
                    p["anim_state"] = want
    # The walk cycle runs on the omni timeline over [0, _CLIP_S] (started in build/after
    # world.reset).  Its natural pace is _STRIDE_M / _CLIP_S m/s, so PEOPLE_SPEED is set to
    # that value in config - the feet then match the ground motion instead of sliding.
