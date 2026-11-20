"""Verify walk clip playback on biped_demo: bind, step sim, screenshot + measure joint motion.
    ./python.sh ~/warehouse/shot_people.py"""
import sys, os
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdSkel, Gf

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
CLIP = f"{ROOT}/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd"
BIPED = f"{ROOT}/Isaac/People/Characters/biped_demo/biped_demo_meters.usd"

import omni.usd
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

floor = UsdGeom.Cube.Define(stage, "/World/Floor")
floor.GetSizeAttr().Set(1.0)
UsdGeom.Xformable(floor).AddScaleOp().Set(Gf.Vec3f(40, 40, 0.1))
UsdGeom.Xformable(floor).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))

# lights so the frame is not black
from pxr import UsdLux
d = UsdLux.DistantLight.Define(stage, "/World/Sun")
d.CreateIntensityAttr(3000.0)
d.CreateAngleAttr(1.0)
UsdGeom.Xformable(d.GetPrim()).AddRotateXOp().Set(-50.0)
fl = UsdLux.SphereLight.Define(stage, "/World/Fill")
fl.CreateIntensityAttr(3000.0)
fl.CreateRadiusAttr(4.0)
UsdGeom.Xformable(fl.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(-3, -3, 5))

p = stage.DefinePrim("/World/Person")
p.GetReferences().AddReference(BIPED)
clip = stage.DefinePrim("/World/PersonClip")
clip.GetReferences().AddReference(CLIP)

skel = anim = None
for pr in Usd.PrimRange(p, Usd.TraverseInstanceProxies()):
    if pr.GetTypeName() == "Skeleton" and skel is None:
        skel = pr
for pr in Usd.PrimRange(clip, Usd.TraverseInstanceProxies()):
    if pr.GetTypeName() == "SkelAnimation" and anim is None:
        anim = pr
print(f"[bind] skel={skel.GetPath() if skel else None}")
print(f"[bind] anim={anim.GetPath() if anim else None}")
if not (skel and anim):
    print("MISSING"); app.close(); sys.exit(1)

UsdSkel.BindingAPI.Apply(skel).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
js = [str(x) for x in skel.GetAttribute("joints").Get()]
print(f"[bind] animationSource={list(UsdSkel.BindingAPI(skel).GetAnimationSourceRel().GetTargets())}")

# camera
cam = UsdGeom.Camera.Define(stage, "/World/Cam")
cam.GetFocalLengthAttr().Set(24.0)
cam.GetHorizontalApertureAttr().Set(36.0)
xf = UsdGeom.Xformable(cam.GetPrim())
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(-6.0, 0, 1.1))
xf.AddRotateYOp().Set(-90.0)     # look +X
xf.AddRotateXOp().Set(-2.0)

from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0)
world.reset()
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
vp = get_active_viewport()
vp.camera_path = "/World/Cam"
try:
    vp.set_resolution((1280, 720))
except Exception:
    pass
OUT = "/home/arun/warehouse/shots"
os.makedirs(OUT, exist_ok=True)

# progress check: how far does a toe joint move over 1s of animation?
def find_joint_fn(c):
    for m in dir(c):
        if "oint" in m and not m.startswith("_"):
            return getattr(c, m)
    return None


cache = UsdSkel.Cache()
JFN = find_joint_fn(cache)
print(f"[cache] joint fn = {JFN.__name__ if JFN else None}; attrs={[m for m in dir(cache) if not m.startswith('_')][:20]}")

toe_idx = next((i for i, n in enumerate(js) if n.endswith("L_Ball") or n.endswith("L_ToeBase") or n.endswith("L_Foot")), None)
if toe_idx is None:
    toe_idx = len(js) // 2
print(f"[progress] tracking joint {js[toe_idx]}")


def toe_pos(t):
    try:
        b = cache.ComputeSkelBinding(skel)
        q = cache.GetSkelQuery(b)
        fn = getattr(q, "ComputeJointWorldTransforms", None) or getattr(q, "ComputeJointsWorldTransforms", None)
        if fn is None:
            return None
        try:
            ok, mats = fn(Usd.TimeCode(t))
        except TypeError:
            ok, mats = fn(time=Usd.TimeCode(t))
        if not ok:
            return None
    except Exception as e:
        return f"err:{e}"
    m = mats[toe_idx]
    return (float(m[3][0]), float(m[3][1]), float(m[3][2]))


for i in range(5):
    for _ in range(8):
        world.step(render=True)
    t = world.current_time
    capture_viewport_to_file(vp, f"{OUT}/walk_{i}.png")
    pos = toe_pos(t)
    print(f"[progress] t={t:.3f}s toe_world={pos} shot={OUT}/walk_{i}.png", flush=True)
print("done")
app.close()
