"""Reusable warehouse shell (from step1). Call build(stage) after SimulationApp exists."""
import config as C
import random
import numpy as np


def build(stage, pallet_boxes=True, legacy_statics=True):
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf, PhysxSchema

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World/Warehouse")


    def material(path, rgb, rough=0.7, metal=0.0):
        m = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m


    MATS = {
        "floor": material("/World/Looks/floor", (0.35, 0.35, 0.37), 0.9),
        "wall": material("/World/Looks/wall", (0.75, 0.75, 0.72), 0.9),
        "belt": material("/World/Looks/belt", (0.08, 0.08, 0.09), 0.95),
        "frame": material("/World/Looks/frame", (0.55, 0.55, 0.6), 0.4, 0.8),
        "pkg": material("/World/Looks/pkg", (0.72, 0.52, 0.30), 0.9),
        "dock": material("/World/Looks/dock", (0.1, 0.6, 0.2), 0.8),
        "marker": material("/World/Looks/marker", (0.9, 0.7, 0.1), 0.8),
        "ped": material("/World/Looks/ped", (0.25, 0.25, 0.3), 0.6, 0.5),
    }


    def box(path, size, pos, mat=None, collide=True, body=None, mass=None):
        """body: None/'static' | 'kinematic' | 'dynamic'"""
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        prim = cube.GetPrim()
        if mat is not None:
            UsdShade.MaterialBindingAPI(prim).Bind(MATS[mat])
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
        if body in ("kinematic", "dynamic"):
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            if body == "kinematic":
                rb.CreateKinematicEnabledAttr(True)
        if mass:
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        return prim


    # ---------- shell: floor, 4 walls, ceiling -------------------------------
    hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
    box("/World/Warehouse/Floor", (C.ROOM_X + 1, C.ROOM_Y + 1, 0.2), (0, 0, -0.1), "floor")
    wz = C.WALL_H / 2
    box("/World/Warehouse/WallN", (C.ROOM_X + C.WALL_T * 2, C.WALL_T, C.WALL_H), (0, hy + C.WALL_T / 2, wz), "wall")
    box("/World/Warehouse/WallS", (C.ROOM_X + C.WALL_T * 2, C.WALL_T, C.WALL_H), (0, -hy - C.WALL_T / 2, wz), "wall")
    box("/World/Warehouse/WallE", (C.WALL_T, C.ROOM_Y, C.WALL_H), (hx + C.WALL_T / 2, 0, wz), "wall")
    box("/World/Warehouse/WallW", (C.WALL_T, C.ROOM_Y, C.WALL_H), (-hx - C.WALL_T / 2, 0, wz), "wall")
    box("/World/Warehouse/Ceiling", (C.ROOM_X + C.WALL_T * 2, C.ROOM_Y + C.WALL_T * 2, C.WALL_T),
        (0, 0, C.WALL_H + C.WALL_T / 2), "wall")

    # ---------- static warehouse obstacles (racks, crates) -------------------
    # Legacy step-1 dressing: it hangs racks high on the west wall and scatters crates in
    # mid-air, and it knows nothing about pickup/drop zones. Step 3 has its own grounded,
    # keep-out-aware dressing (statics.py), so it switches this off.
    if legacy_statics:
        rack_h, rack_w, rack_d = 2.4, 0.3, 1.8  # height, width (into room), depth
        rack_h_gap = 0.6  # vertical gap between shelves
        for row in range(3):
            for col in range(2):
                rx = -hx + 1.0 + col * (rack_d + 0.2)   # just inside the east wall
                rz = hy - 1.5 - row * (rack_h + rack_h_gap)  # start high, step down
                if rz > 0.5:  # only place racks that fit
                    # rack base frame
                    box(f"/World/Static/RackFrame_{row}_{col}", (rack_d, rack_w, rack_h),
                        (rx, 0, rz + rack_h / 2), "frame")
                    # one shelf per rack
                    shelf_z = rz + rack_h + 0.15
                    box(f"/World/Static/RackShelf_{row}_{col}", (rack_d - 0.2, 0.8, 0.08),
                        (rx + 0.1, 0, shelf_z), "frame")

        # A few scattered crates as obstacles (grounded, so nothing floats)
        for i in range(5):
            cx = random.uniform(-hx + 2, hx - 2)
            cy = random.uniform(-hy + 2, hy - 2)
            box(f"/World/Static/Crate_{i}", (0.8, 0.5, 0.4), (cx, cy, 0.2), "pkg")

    # (dynamic people and moving props are built by people.py and returned to the caller separately)

    # ---------- lighting: a few rect lights under the ceiling (cheap) --------
    n = C.NUM_CEILING_LIGHTS
    cols = 3
    xs = np.linspace(-hx * 0.6, hx * 0.6, cols)
    ys = np.linspace(-hy * 0.5, hy * 0.5, n // cols)
    i = 0
    for y in ys:
        for x in xs:
            L = UsdLux.RectLight.Define(stage, f"/World/Lights/Ceiling_{i}")
            L.CreateWidthAttr(3.0); L.CreateHeightAttr(1.0)
            L.CreateIntensityAttr(9000.0)
            xf = UsdGeom.Xformable(L); xf.AddTranslateOp().Set(Gf.Vec3d(x, y, C.WALL_H - 0.1))
            i += 1  # RectLight faces -Z by default, i.e. down

    # ---------- conveyors ----------------------------------------------------
    def build_conveyor(tag, cfg):
        y = cfg["y"]
        x0, x1 = C.BELT_START_X, C.BELT_START_X + C.BELT_LEN
        cx = (x0 + x1) / 2
        belt_th = 0.08
        belt = box(f"/World/Conveyor{tag}/Belt", (C.BELT_LEN, C.BELT_W, belt_th),
                   (cx, y, C.BELT_H - belt_th / 2), "belt", body="kinematic")
        # real moving surface: PhysX surface velocity (world +X since no rotation)
        sv = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt)
        sv.CreateSurfaceVelocityEnabledAttr(True)
        sv.CreateSurfaceVelocityAttr(Gf.Vec3f(C.BELT_SPEED, 0, 0))
        # side rails keep packages on the belt
        for s, nm in ((+1, "RailL"), (-1, "RailR")):
            box(f"/World/Conveyor{tag}/{nm}", (C.BELT_LEN, 0.03, 0.10),
                (cx, y + s * (C.BELT_W / 2 + 0.015), C.BELT_H + 0.05 - belt_th), "frame")
        # legs
        for k, lx in enumerate(np.linspace(x0 + 0.3, x1 - 0.3, 4)):
            box(f"/World/Conveyor{tag}/Leg{k}", (0.08, C.BELT_W, C.BELT_H - belt_th),
                (lx, y, (C.BELT_H - belt_th) / 2), "frame")
        # stopper: package halts with its centre at PICK_X
        stop_x = C.PICK_X + C.PKG_SIZE[0] / 2 + 0.02
        box(f"/World/Conveyor{tag}/Stopper", (0.04, C.BELT_W, 0.20),
            (stop_x, y, C.BELT_H + 0.10), "marker")
        # arm pedestal (inward side of the belt)
        sgn = -1 if y > 0 else +1
        ped_y = y + sgn * C.ARM_OFFSET_Y
        box(f"/World/Manip_{cfg['arm']}/Pedestal", (0.35, 0.35, C.ARM_PEDESTAL_H),
            (C.PICK_X, ped_y, C.ARM_PEDESTAL_H / 2), "ped")
        # carter dock marker (visual only, no collision): where the arm places
        dock_y = y + sgn * C.DOCK_OFFSET_Y
        box(f"/World/Docks/Dock{tag}", (0.8, 0.6, 0.01), (C.PICK_X, dock_y, 0.005), "dock", collide=False)
        return (C.PICK_X, y)


    for tag, cfg in C.CONVEYORS.items():
        build_conveyor(tag, cfg)

    # ---------- forklift + pallet markers (SimReady forklift comes in Step 5) -
    for name, f in C.FORKLIFTS.items():
        fx, fy = f["pos"]
        box(f"/World/Forklifts/{name}_footprint", (2.6, 1.2, 0.01), (fx, fy, 0.005), "marker", collide=False)
        if pallet_boxes:
            box(f"/World/Forklifts/{name}_pallet", C.PALLET_SIZE,
                (fx + C.PALLET_OFFSET_X, fy, C.PALLET_SIZE[2] / 2), "frame")

    # ---------- package spawner ----------------------------------------------
    class Spawner:
        def __init__(self, tag, y):
            self.tag, self.y, self.t, self.n, self.live = tag, y, 0.0, 0, []

        def tick(self, dt):
            self.t += dt
            # cull anything that fell off the world
            for p in list(self.live):
                if not p.IsValid():
                    self.live.remove(p); continue
                z = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
                if z < -1.0:
                    stage.RemovePrim(p.GetPath()); self.live.remove(p)
            if self.t >= C.SPAWN_PERIOD_S and len(self.live) < C.MAX_ACTIVE_PKGS:
                self.t = 0.0
                self.spawn()

        def spawn(self):
            sx, sy, sz = C.PKG_SIZE
            pos = (C.BELT_START_X + 0.4, self.y, C.BELT_H + sz / 2 + 0.02)
            p = box(f"/World/Packages/Pkg{self.tag}_{self.n}", C.PKG_SIZE, pos, "pkg",
                    body="dynamic", mass=C.PKG_MASS)
            self.n += 1; self.live.append(p)
            return p


    spawners = [Spawner(t, c["y"]) for t, c in C.CONVEYORS.items()]
    return spawners
