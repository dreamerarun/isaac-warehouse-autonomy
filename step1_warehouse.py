"""
Step 1: enclosed warehouse shell, two powered conveyors with stoppers,
package spawners, arm pedestals, carter docks, forklift + pallet markers.

Run (from your Isaac Sim 5.1 folder):
    ./python.sh /path/to/warehouse/step1_warehouse.py
    ./python.sh /path/to/warehouse/step1_warehouse.py --headless --steps 600
Primitives only on purpose: nothing here can fail on a missing asset path.
Robots / forklift USDs are added in later steps after check_assets.py.
"""
import argparse, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--steps", type=int, default=0, help="0 = run until window closed")
args, _ = ap.parse_known_args()

import config as C
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless, "width": C.RENDER_W,
                     "height": C.RENDER_H, "renderer": C.RENDERER})

import random
import numpy as np
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf, PhysxSchema
from isaacsim.core.api import World
import omni.usd

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60.0, rendering_dt=1 / 30.0)
stage = omni.usd.get_context().get_stage()
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
            sx, sy, sz = C.PKG_SIZE
            pos = (C.BELT_START_X + 0.4, self.y + random.uniform(-0.05, 0.05), C.BELT_H + sz / 2 + 0.02)
            p = box(f"/World/Packages/Pkg{self.tag}_{self.n}", C.PKG_SIZE, pos, "pkg",
                    body="dynamic", mass=C.PKG_MASS)
            self.n += 1; self.live.append(p)


spawners = [Spawner(t, c["y"]) for t, c in C.CONVEYORS.items()]

# ---------- viewport camera ----------------------------------------------
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-4.0, -12.0, 8.0]), target=np.array([0.0, 0.0, 1.0]))

world.reset()
step = 0
while app.is_running():
    world.step(render=not args.headless)
    for s in spawners:
        s.tick(1 / 60.0)
    step += 1
    if args.steps and step >= args.steps:
        break
app.close()
