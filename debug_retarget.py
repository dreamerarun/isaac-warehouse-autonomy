"""Debug retarget orientation: inspect root joint rest vs baked transforms and world joint positions.
    ./python.sh ~/warehouse/debug_retarget.py"""
import sys, math, os
sys.stdout.reconfigure(line_buffering=True)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.anim.retarget.core")
from pxr import Usd, UsdGeom, UsdSkel, Gf
import omni.usd, omni.kit.commands

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
BIPED = f"{ROOT}/Isaac/People/Characters/biped_demo/biped_demo_meters.usd"
MALE = f"{ROOT}/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
CLIP = f"{ROOT}/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd"

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
src = stage.DefinePrim("/World/Src"); src.GetReferences().AddReference(BIPED)
tgt = stage.DefinePrim("/World/Tgt"); tgt.GetReferences().AddReference(MALE)
clip = stage.DefinePrim("/World/Clip"); clip.GetReferences().AddReference(CLIP)


def find(root, tpe):
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == tpe:
            return p
    return None


src_skel, tgt_skel, clip_anim = find(src, "Skeleton"), find(tgt, "Skeleton"), find(clip, "SkelAnimation")
src_skel, tgt_skel = UsdSkel.Skeleton(src_skel), UsdSkel.Skeleton(tgt_skel)


def describe(tag, skel):
    js = [str(x) for x in skel.GetJointsAttr().Get()]
    rest = skel.GetRestTransformsAttr().Get()
    print(f"[{tag}] joints={len(js)} root={js[0]}")
    m0 = rest[0]
    print(f"[{tag}] rest root: trans={[round(float(v),3) for v in m0.ExtractTranslation()]} "
          f"rot={m0.ExtractRotationQuat()}")
    # which joint is hips / head / toe
    for name in js:
        leaf = name.split("/")[-1]
        if leaf in ("Hip", "Pelvis", "Head", "L_Foot", "L_ToeBase", "RL_BoneRoot"):
            i = js.index(name)
            t = rest[i].ExtractTranslation()
            print(f"[{tag}] rest {leaf}: {[round(float(v),3) for v in t]}")
    # axes metadata
    for a in ("controlRig:upAxis", "controlRig:forwardAxis", "controlRig:retargetTags"):
        attr = skel.GetPrim().GetAttribute(a)
        if attr and attr.HasAuthoredValue():
            v = attr.Get()
            print(f"[{tag}] {a} = {list(v)[:12] if hasattr(v,'__iter__') else v}")


describe("src", src_skel)
describe("tgt", tgt_skel)

from omni.anim.retarget.core import auto_setup
print(f"[auto] src={auto_setup('Human', src_skel.GetPath())} tgt={auto_setup('Human', tgt_skel.GetPath())}")
describe("tgt-after", tgt_skel)

stage.DefinePrim("/World/Tgt/Retargeted", "Scope")
res = omni.kit.commands.execute(
    "CreateRetargetAnimationsCommand",
    source_skeleton_path=str(src_skel.GetPath()),
    target_skeleton_path=str(tgt_skel.GetPath()),
    source_animation_paths=[str(clip_anim.GetPath())],
    target_animation_parent_path="/World/Tgt/Retargeted",
    set_root_identity=True,
)
baked = None
for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Tgt/Retargeted")):
    if p.GetTypeName() == "SkelAnimation":
        baked = p
print(f"[bake] res={res} baked={baked.GetPath() if baked else None}")
a = UsdSkel.Animation(baked)
js = [str(x) for x in tgt_skel.GetJointsAttr().Get()]
rots = a.GetRotationsAttr().Get(0.0)
tr = a.GetTranslationsAttr().Get(0.0)
print(f"[bake] anim root joint token={a.GetJointsAttr().Get()[0]}")
print(f"[bake] root trans={tuple(round(float(v),3) for v in tr[0])} rot={rots[0]}")

UsdSkel.BindingAPI.Apply(tgt_skel.GetPrim()).CreateAnimationSourceRel().SetTargets([baked.GetPath()])

# world joint transforms at a few times
cache = UsdSkel.Cache()
q = cache.GetSkelQuery(tgt_skel)
xfc = UsdGeom.XformCache()
print(f"[q] methods={[m for m in dir(q) if 'ompute' in m]}")
for t in (0.0, 40.0, 80.0, 120.0):
    try:
        mats = q.ComputeJointSkelTransforms(Usd.TimeCode(t))
    except Exception as e:
        print(f"[q] call failed: {e}")
        break
    if not mats:
        print(f"[q] t={t} empty")
        continue
    for want in ("RL_BoneRoot", "Head", "L_ToeBase", "L_Foot"):
        idx = next((i for i, n in enumerate(js) if n.split("/")[-1] == want), None)
        if idx is None:
            continue
        m = mats[idx]
        p = m.ExtractTranslation()
        print(f"[q] t={t:5.1f} {want}: world=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")
# xform ops present on the target prim (asset may carry its own orientation fix)
tgt_xf = UsdGeom.Xformable(tgt)
print(f"[xform] tgt ops={[o.GetOpName() for o in tgt_xf.GetOrderedXformOps()]}")
lt = tgt_xf.GetLocalTransformation()
print(f"[xform] tgt local translation={[round(float(v),2) for v in lt.ExtractTranslation()]} "
      f"rot={lt.ExtractRotationQuat()}")

# skinning transforms (what the renderer applies to the mesh) at animated times
skq = cache.GetSkelQuery(tgt_skel)
for t in (0.0, 40.0):
    mats = skq.ComputeSkinningTransforms(Usd.TimeCode(t))
    i_hip = next((i for i, n in enumerate(js) if n.split("/")[-1] == "Hip"), 1)
    m = mats[i_hip]
    p = m.ExtractTranslation()
    q_ = m.ExtractRotationQuat()
    print(f"[skin] t={t} Hip skin trans=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) "
          f"rot=({q_.GetReal():.2f},{q_.GetImaginary()[0]:.2f},{q_.GetImaginary()[1]:.2f},{q_.GetImaginary()[2]:.2f})")

# world bbox of the *geometry* at explicit times (what the renderer draws)
from pxr import UsdGeom
for tt in (0.0, 40.0, 80.0):
    bc = UsdGeom.BBoxCache(Usd.TimeCode(tt), [UsdGeom.Tokens.default_], useExtentsHint=False)
    bbox = bc.ComputeWorldBound(tgt).ComputeAlignedRange()
    mn, mx = bbox.GetMin(), bbox.GetMax()
    print(f"[bbox] t={tt} min={[round(float(v),2) for v in mn]} max={[round(float(v),2) for v in mx]}")
print("done")
app.close()
