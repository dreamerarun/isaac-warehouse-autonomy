"""
Step 2: Step-1 shell + UR5, Franka, 3 Nova Carters, 2 forklifts, real pallets
placed from verified asset paths. Robots are static here (no control yet);
Step 3 adds articulations / IK / suction.

    ./python.sh ~/warehouse/step2_robots.py
    ./python.sh ~/warehouse/step2_robots.py --headless --steps 300
Needs internet (assets stream from NVIDIA S3). First load is slow.
"""
import argparse, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--steps", type=int, default=0)
args, _ = ap.parse_known_args()

import config as C
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless, "width": C.RENDER_W,
                     "height": C.RENDER_H, "renderer": C.RENDERER})

import numpy as np
sys.stdout.reconfigure(line_buffering=True)
from pxr import UsdGeom, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
import scene

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60.0, rendering_dt=1 / 30.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
spawners = scene.build(stage, pallet_boxes=False)  # real pallet assets replace the boxes

root = get_assets_root_path()
if not root:
    raise SystemExit("No asset root - check internet connection")


_bbox = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])


def _size(prim):
    r = _bbox.ComputeWorldBound(prim).ComputeAlignedRange()
    d = r.GetSize()
    return max(d[0], d[1], d[2]), r


def _asset_mpu(url):
    """metersPerUnit the asset file declares (1.0 if unreadable)."""
    from pxr import Usd
    try:
        st = Usd.Stage.Open(url, Usd.Stage.LoadNone)
        return float(UsdGeom.GetStageMetersPerUnit(st)) or 1.0
    except Exception as e:
        print(f"[step2] could not read units of {url}: {e}")
        return 1.0


def freeze(prim):
    """Make a decorative asset inert (no articulation, kinematic bodies) so it
    can't tip over or get pushed. Steps 4/5 re-enable physics where needed."""
    from pxr import Usd, UsdPhysics
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
        if p.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)


def place(prim_path, key, pos, yaw_deg=0.0, inert=False):
    url = f"{root}/{C.ASSETS[key]}"
    prim = add_reference_to_stage(usd_path=url, prim_path=prim_path)
    mpu = _asset_mpu(url)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
    xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw_deg))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(mpu, mpu, mpu))
    if inert:
        freeze(prim)
    _bbox.Clear()
    size, _ = _size(prim)
    lim = {"forklift": (1.5, 6.0), "pallet": (0.8, 1.6)}.get(key)
    if lim and not (lim[0] <= size <= lim[1]) and size > 0:
        import math
        k = 10.0 ** (-round(math.log10(size / math.sqrt(lim[0] * lim[1]))))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw_deg))
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(mpu * k, mpu * k, mpu * k))
        print(f"[step2] {key} size {size:.2f} m outside {lim}; extra correction x{k:g}")
        _bbox.Clear()
        size, _ = _size(prim)
    print(f"[step2] {prim_path} <- {key} @ {pos} yaw {yaw_deg} | file units {mpu:g} m/unit -> scale x{mpu:g} | longest side {size:.2f} m")
    return prim


# arms: on pedestal top, facing the belt (belt A at +y -> face +y)
for tag, cfg in C.CONVEYORS.items():
    sgn = -1 if cfg["y"] > 0 else +1
    ped_y = cfg["y"] + sgn * C.ARM_OFFSET_Y
    yaw = 90.0 if cfg["y"] > 0 else -90.0
    place(f"/World/Manip_{cfg['arm']}/Robot", cfg["arm"], (C.PICK_X, ped_y, C.ARM_PEDESTAL_H), yaw)

# carters at start parking
for name, (x, y) in C.CARTER_START.items():
    place(f"/World/Carters/{name}", "carter", (x, y, 0.05), 0.0, inert=True)

# forklifts + real pallets in front of them
for name, f in C.FORKLIFTS.items():
    fx, fy = f["pos"]
    place(f"/World/Forklifts/{name}", "forklift", (fx, fy, 0.0), f["yaw_deg"], inert=True)
    place(f"/World/Forklifts/{name}_pallet_asset", "pallet", (fx + C.PALLET_OFFSET_X, fy, 0.0), 0.0, inert=True)

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
    if step in (1, 120, 600):
        for p in ("/World/Carters/C1", "/World/Carters/C2", "/World/Carters/C3",
                  "/World/Forklifts/F1", "/World/Forklifts/F2"):
            pr = stage.GetPrimAtPath(p)
            _bbox.Clear()
            sz, r = _size(pr)
            c = r.GetMidpoint()
            print(f"[diag step {step}] {p} valid={pr.IsValid()} children={len(pr.GetChildren())} "
                  f"center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f}) longest={sz:.2f}")
    if args.steps and step >= args.steps:
        break
app.close()
