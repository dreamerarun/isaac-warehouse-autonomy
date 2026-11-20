"""Static warehouse dressing: racks (real SimReady-style props, procedural fallback), pallets with cartons,
columns.  Everything placed is reported back as footprint rectangles so the navigation grid avoids it."""
import math, random
import numpy as np
import config as C
import assets as A


def _overlap(r, k, m=0.4):
    return not (r[1] + m < k[0] or r[0] - m > k[1] or r[3] + m < k[2] or r[2] - m > k[3])


def _rect_of(stage, path):
    mn, mx = A.world_bbox(stage.GetPrimAtPath(path))
    return (float(mn[0]), float(mx[0]), float(mn[1]), float(mx[1]))


def _make_static(prim):
    """keep colliders, drop dynamics, so props are solid but never move."""
    from pxr import Usd, UsdPhysics
    for p in Usd.PrimRange(prim):
        try:
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if p.HasAPI(UsdPhysics.MassAPI):
                p.RemoveAPI(UsdPhysics.MassAPI)
        except Exception:
            pass


def _real(stage, root, rel, path, pos, yaw, expect, report):
    """Place one real asset, measured and unit-corrected. Returns (path, size_xyz) or None."""
    url = f"{root}/{rel}"
    try:
        m = A.measure(url, expect)
        size = m["max"] - m["min"]
        if not np.all(np.isfinite(size)) or size.max() <= 0:
            raise RuntimeError("empty bounds")
    except Exception as e:
        report(f"[statics] could not measure {rel.split('/')[-1]}: {e}")
        return None
    # put the asset's footprint centre on (x, y) and its bottom on z=0
    c = (m["min"] + m["max"]) / 2
    th = math.radians(yaw)
    ox = c[0] * math.cos(th) - c[1] * math.sin(th)
    oy = c[0] * math.sin(th) + c[1] * math.cos(th)
    prim, _ = A.place(stage, root, {"x": rel}, path, "x", (pos[0] - ox, pos[1] - oy, pos[2] - m["min"][2]), yaw,
                      scale_override=m["scale"])
    # the analytic offset above can be off when an asset mixes purposes/units; snap the
    # *actual* rendered bottom onto the requested support height so nothing hovers.
    try:
        from pxr import UsdGeom, Gf
        mn = A.world_bbox(prim, render=True)[0]
        dz = float(pos[2]) - float(mn[2])
        if abs(dz) > 1e-4:
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(op.Get() + Gf.Vec3d(0.0, 0.0, dz))
                    break
    except Exception as e:
        report(f"[statics] could not ground {path}: {e}")
    _make_static(prim)
    return path, size


# ---------------------------------------------------------------------------------------------
# procedural fallback (primitives with colliders) - used only if a real asset is missing/implausible
# ---------------------------------------------------------------------------------------------
def _box(stage, path, size, pos, rgb, collide=True):
    from pxr import UsdGeom, UsdPhysics, Gf
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    f = UsdGeom.Xformable(c)
    f.AddTranslateOp().Set(Gf.Vec3d(*pos))
    f.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())


def procedural_rack(stage, path, cx, cy, length, depth, height, levels=3, seed=0):
    from pxr import UsdGeom
    rnd = random.Random(seed)
    UsdGeom.Xform.Define(stage, path)
    bays = max(int(round(length / 2.7)), 1)
    bw = length / bays
    x0 = cx - length / 2
    for i in range(bays + 1):
        for sy in (-1, 1):
            _box(stage, f"{path}/up_{i}_{0 if sy < 0 else 1}", (0.08, 0.08, height), (x0 + i * bw, cy + sy * (depth / 2 - 0.04), height / 2), (0.1, 0.3, 0.7))
    for k in range(levels):
        z = 0.30 + k * (height - 0.5) / levels
        for sy in (-1, 1):
            _box(stage, f"{path}/beam_{k}_{0 if sy < 0 else 1}", (length, 0.06, 0.12), (cx, cy + sy * (depth / 2 - 0.04), z), (0.95, 0.45, 0.05))
        _box(stage, f"{path}/deck_{k}", (length - 0.05, depth - 0.12, 0.03), (cx, cy, z + 0.075), (0.55, 0.55, 0.58))
        for b in range(bays):
            for j in range(rnd.choice((1, 2, 3))):
                w = rnd.uniform(0.45, 0.8)
                h = rnd.uniform(0.3, min(0.9, (height - 0.5) / levels - 0.25))
                _box(stage, f"{path}/ctn_{k}_{b}_{j}", (w, depth - 0.3, h),
                     (x0 + b * bw + 0.4 + j * (bw - 0.8) / 2.2, cy, z + 0.09 + h / 2), (0.72, 0.52, 0.30))


def build(stage, root, keep_out, report=print, seed=3):
    """Returns list of obstacle rects. keep_out: rects that must stay clear (belts, docks, forklift strips...)."""
    rnd = random.Random(seed)
    rects = []
    hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
    S = C.STATIC_ASSETS

    # ---- racks along the north and south walls: real upright frames + real shelves + cartons on them ----------
    def real_rack_row(ysgn, n_bays_max):
        fr = A.measure(f"{root}/{S['rack_frame']}", (0.3, 8.0))
        sh = A.measure(f"{root}/{S['rack_shelf']}", (0.3, 8.0))
        fs, ss = fr["max"] - fr["min"], sh["max"] - sh["min"]
        if not (1.5 <= fs[2] <= 8.0 and 0.3 <= max(fs[0], fs[1]) <= 3.0):
            raise RuntimeError(f"rack frame size {np.round(fs, 2)} implausible")
        if not (1.0 <= max(ss[0], ss[1]) <= 5.0 and ss[2] <= 0.8):
            raise RuntimeError(f"rack shelf size {np.round(ss, 2)} implausible")
        frame_yaw = 0.0 if fs[0] <= fs[1] else 90.0              # thin side along x
        fw, fd = (min(fs[0], fs[1]), max(fs[0], fs[1]))
        shelf_yaw = 0.0 if ss[0] >= ss[1] else 90.0              # long side along x
        L_s = max(ss[0], ss[1])
        pitch = L_s + fw
        n = int(min((C.ROOM_X - 2.0 - fw) // pitch, n_bays_max))
        if n < 1:
            raise RuntimeError("no room for a rack bay")
        total = n * pitch + fw
        x_start = -total / 2 + fw / 2
        ycen = ysgn * (hy - 0.5 - fd / 2)
        tag = "N" if ysgn > 0 else "S"
        foot_y = (ycen - fd / 2, ycen + fd / 2)
        # never dress a bay that sits inside a pickup/drop zone (belt, dock, forklift lane)
        bays = []
        for i in range(n):
            xc = x_start + (i + 0.5) * pitch
            if any(_overlap((xc - pitch / 2, xc + pitch / 2, foot_y[0], foot_y[1]), k) for k in keep_out):
                report(f"[statics] rack row {tag}: bay at x={xc:.1f} skipped (pickup/drop zone)")
                continue
            bays.append(xc)
        if not bays:
            raise RuntimeError("every rack bay overlaps a pickup/drop zone")
        levels = [0.30 + i * (fs[2] - 0.6) / 3.0 for i in range(3)]
        # uprights at both ends of every contiguous run of kept bays
        edges = []
        for xc in bays:
            left, right = xc - pitch / 2, xc + pitch / 2
            if not edges or abs(edges[-1] - left) > 1e-6:
                edges.append(left)
            edges.append(right)
        for k, xe in enumerate(edges):
            _real(stage, root, S["rack_frame"], f"/World/Statics/Rack{tag}_frame{k}", (xe, ycen, 0.0),
                  frame_yaw, (0.3, 8.0), lambda *_: None)
        for i, xc in enumerate(bays):
            for j, z in enumerate(levels):
                sp = _real(stage, root, S["rack_shelf"], f"/World/Statics/Rack{tag}_shelf{i}_{j}", (xc, ycen, z),
                           shelf_yaw, (0.3, 8.0), lambda *_: None)
                if sp and rnd.random() < 0.8:
                    top = z + float(sp[1][2])          # intended shelf surface, not a scene-bbox guess
                    rel = rnd.choice(S["cartons"])
                    _real(stage, root, rel, f"/World/Statics/Rack{tag}_box{i}_{j}", (xc + rnd.uniform(-0.4, 0.4), ycen, top),
                          rnd.choice((0, 90)), (0.15, 1.2), lambda *_: None)
        row_rects, run = [], [bays[0]]
        for xc in bays[1:]:
            if abs(xc - run[-1] - pitch) < 1e-6:
                run.append(xc)
            else:
                row_rects.append((run[0] - pitch / 2, run[-1] + pitch / 2, foot_y[0], foot_y[1]))
                run = [xc]
        row_rects.append((run[0] - pitch / 2, run[-1] + pitch / 2, foot_y[0], foot_y[1]))
        return row_rects, len(edges)

    real_ok, placed = True, 0
    for side, ysgn in (("N", 1), ("S", -1)):
        try:
            if not real_ok:
                raise RuntimeError("skipped after earlier failure")
            row_rects, nfr = real_rack_row(ysgn, max(C.MAX_RACKS // 2, 1))
            rects.extend(row_rects)
            placed += 1
            for rect in row_rects:
                report(f"[statics] rack row {side}: real frames + shelves + cartons, {nfr - 1} frames, "
                       f"x {rect[0]:.1f}..{rect[1]:.1f}")
        except Exception as e:
            real_ok = False
            report(f"[statics] real rack row {side} unavailable ({e}) -> procedural rack")
            ycen = ysgn * (hy - 1.0)
            for i, x in enumerate(np.linspace(-hx + 3.5, hx - 3.5, max(C.MAX_RACKS // 2, 1))):
                procedural_rack(stage, f"/World/Statics/Rack{side}{i}", x, ycen, 4.0, 1.1, 4.0, 3, seed + i)
                rects.append((x - 2.0, x + 2.0, ycen - 0.55, ycen + 0.55))

    # ---- pallets with cartons / crates (floor clutter) ---------------------------------------------
    spots = [(-1.5, 5.6), (3.5, -5.7), (-4.0, -5.6), (1.0, 5.7), (-8.5, 6.0), (-8.5, -6.0)]
    for i, (x, y) in enumerate(spots):
        foot = (x - 0.7, x + 0.7, y - 0.65, y + 0.65)
        if any(_overlap(foot, k) for k in keep_out + rects):
            continue
        pp = _real(stage, root, S["pallet"], f"/World/Statics/Pallet{i}", (x, y, 0.0), rnd.choice((0, 90)), (0.8, 1.6), report)
        if not pp:
            continue
        z = float(pp[1][2])                        # pallet top from its measured height
        for layer in range(rnd.choice((1, 2, 3))):
            rel = rnd.choice(S["cartons"] + S["crates"])
            b = _real(stage, root, rel, f"/World/Statics/Pallet{i}_box{layer}", (x, y, z), rnd.choice((0, 90)), (0.15, 1.2), report)
            if b:
                z += float(b[1][2])                # next layer sits on this one's measured height
        r = _rect_of(stage, pp[0])
        rects.append((min(r[0], x - 0.6), max(r[1], x + 0.6), min(r[2], y - 0.6), max(r[3], y + 0.6)))

    # ---- columns (primitive cylinders with colliders) ---------------------------------------------------
    from pxr import UsdGeom, UsdPhysics, Gf
    for i, (x, y) in enumerate(((-3.0, 0.0), (3.0, 0.0))):
        foot = (x - 0.3, x + 0.3, y - 0.3, y + 0.3)
        if any(_overlap(foot, k) for k in keep_out):
            continue
        cyl = UsdGeom.Cylinder.Define(stage, f"/World/Statics/Column{i}")
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(C.WALL_H)
        cyl.CreateRadiusAttr(0.25)
        UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(x, y, C.WALL_H / 2))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.62, 0.6)])
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        rects.append(foot)
        # yellow safety band
        band = UsdGeom.Cylinder.Define(stage, f"/World/Statics/Column{i}_band")
        band.CreateAxisAttr("Z")
        band.CreateHeightAttr(0.5)
        band.CreateRadiusAttr(0.26)
        UsdGeom.Xformable(band).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.4))
        band.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.8, 0.05)])
    return rects
