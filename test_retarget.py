"""Test: retarget the Isaac walk clip onto a realistic RL_BoneRoot character, bind, and render.
    ./python.sh ~/warehouse/test_retarget.py"""
import sys, os
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.anim.retarget.core")

from pxr import Usd, UsdGeom, UsdSkel, UsdLux, Gf
import omni.usd
import omni.kit.commands

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
BIPED = f"{ROOT}/Isaac/People/Characters/biped_demo/biped_demo_meters.usd"
MALE = f"{ROOT}/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
CLIP = f"{ROOT}/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd"

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# --- source rig (biped: matches clip 81/81) and target rig (realistic character)
src = stage.DefinePrim("/World/Src"); src.GetReferences().AddReference(BIPED)
tgt = stage.DefinePrim("/World/Tgt"); tgt.GetReferences().AddReference(MALE)
clip = stage.DefinePrim("/World/Clip"); clip.GetReferences().AddReference(CLIP)


def find(root, tpe, first_only=True):
    out = []
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == tpe:
            out.append(p)
            if first_only:
                break
    return out[0] if out else None


src_skel = find(src, "Skeleton")
tgt_skel = find(tgt, "Skeleton")
clip_anim = find(clip, "SkelAnimation")
print(f"[setup] src={src_skel.GetPath()}")
print(f"[setup] tgt={tgt_skel.GetPath()}")
print(f"[setup] clip={clip_anim.GetPath()}")
assert src_skel and tgt_skel and clip_anim

# --- auto retarget setup (tags + facing + rest pose) for both rigs
from omni.anim.retarget.core import auto_setup, has_retarget_setup
import omni.anim.retarget.core as rc

ok1 = auto_setup("Human", src_skel.GetPath())
ok2 = auto_setup("Human", tgt_skel.GetPath())
print(f"[auto_setup] src={ok1} tgt={ok2} "
      f"src_tags={has_retarget_setup(src_skel)} tgt_tags={has_retarget_setup(tgt_skel)}")

# --- bake retargeted animation onto the target skeleton
tgt_anim_parent = "/World/Tgt/Retargeted"
stage.DefinePrim(tgt_anim_parent, "Scope")
res = omni.kit.commands.execute(
    "CreateRetargetAnimationsCommand",
    source_skeleton_path=str(src_skel.GetPath()),
    target_skeleton_path=str(tgt_skel.GetPath()),
    source_animation_paths=[str(clip_anim.GetPath())],
    target_animation_parent_path=tgt_anim_parent,
    set_root_identity=True,
)
print(f"[bake] result={res}")
baked = None
for p in Usd.PrimRange(stage.GetPrimAtPath(tgt_anim_parent)):
    if p.GetTypeName() == "SkelAnimation":
        baked = p
        break
if baked is None:
    print("[bake] NO ANIMATION PRODUCED")
else:
    a = UsdSkel.Animation(baked)
    js = a.GetJointsAttr().Get()
    ts = a.GetTranslationsAttr().GetTimeSamples() if a.GetTranslationsAttr() else []
    print(f"[bake] baked={baked.GetPath()} joints={len(js)} samples={len(ts)} t=[{ts[0] if ts else '-'}..{ts[-1] if ts else '-'}]")

    # --- bind to the target skeleton and render
    UsdSkel.BindingAPI.Apply(tgt_skel).CreateAnimationSourceRel().SetTargets([baked.GetPath()])
    print(f"[bind] animationSource={list(UsdSkel.BindingAPI(tgt_skel).GetAnimationSourceRel().GetTargets())}")

# light + floor + camera
floor = UsdGeom.Cube.Define(stage, "/World/Floor")
floor.GetSizeAttr().Set(1.0)
UsdGeom.Xformable(floor).AddScaleOp().Set(Gf.Vec3f(40, 40, 0.1))
UsdGeom.Xformable(floor).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXOp().Set(-50.0)
cam = UsdGeom.Camera.Define(stage, "/World/Cam")
cam.GetFocalLengthAttr().Set(24.0)
cam.GetHorizontalApertureAttr().Set(36.0)
xf = UsdGeom.Xformable(cam.GetPrim())
xf.ClearXformOpOrder()
# explicit camera basis (row-vector Gf convention): look +X, up +Z, right -Y
xf.AddTransformOp().Set(Gf.Matrix4d(
    0, -1, 0, 0,
    0, 0, 1, 0,
    -1, 0, 0, 0,
    -6, 0, 1.1, 1))

# separate the two figures along y so both are visible
for prim, y in ((src, -1.2), (tgt, 1.2)):
    xfo = UsdGeom.Xformable(prim)
    xfo.ClearXformOpOrder()
    xfo.AddTranslateOp().Set(Gf.Vec3d(0, y, 0))

from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0)
world.reset()
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
vp = get_active_viewport()
vp.camera_path = "/World/Cam"
OUT = "/home/arun/warehouse/shots"
os.makedirs(OUT, exist_ok=True)
for i in range(3):
    for _ in range(10):
        world.step(render=True)
    capture_viewport_to_file(vp, f"{OUT}/retarget_{i}.png")
    print(f"[shot] {OUT}/retarget_{i}.png", flush=True)
print("done")
app.close()
