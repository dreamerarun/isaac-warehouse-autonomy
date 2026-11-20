"""
Inspect character skeleton joints + walk-clip joint names (for animation binding).
    ./python.sh ~/warehouse/inspect_skel.py
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.stdout.reconfigure(line_buffering=True)
from pxr import Usd, UsdGeom, UsdSkel
from isaacsim.storage.native import get_assets_root_path

root = get_assets_root_path()
CHAR = f"{root}/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
ANIM = f"{root}/Isaac/People/Animations/stand_walk_loop.skelanim.usd"

st = Usd.Stage.Open(CHAR, Usd.Stage.LoadAll)
skel_prim = None
for p in Usd.PrimRange(st.GetPseudoRoot()):
    if p.GetTypeName() == "Skeleton":
        skel_prim = p
        break
print(f"character skeleton: {skel_prim.GetPath() if skel_prim else None}")
if skel_prim:
    sk = UsdSkel.Skeleton(skel_prim)
    order = sk.GetJointsAttr().Get()
    print(f"  joints ({len(order)}): {list(order)[:12]} ...")
    # animationSource rel on skeleton or its parent
    bind = UsdSkel.BindingAPI(skel_prim)
    rel = bind.GetAnimationSourceRel()
    print(f"  animationSource authored={rel.HasAuthoredTargets()} targets={list(rel.GetTargets())}")
    # rest joint local transforms: find toe vs ankle to infer facing
    import numpy as np
    pts = sk.GetRestTransformsAttr().Get()
    names = [str(j).split("/")[-1] for j in order]
    for i, n in enumerate(names):
        if any(k in n.lower() for k in ("toe", "foot", "ankle", "head", "nose", "eye")):
            t = pts[i]
            print(f"    joint {n}: trans=({t[3][0]:.3f}, {t[3][1]:.3f}, {t[3][2]:.3f})")
    # skelroot binding
    for p in Usd.PrimRange(st.GetPseudoRoot()):
        if p.GetTypeName() == "SkelRoot":
            b = UsdSkel.BindingAPI(p)
            print(f"  SkelRoot {p.GetPath()} skeleton rel={list(b.GetSkeletonRel().GetTargets())} "
                  f"animSource={list(b.GetAnimationSourceRel().GetTargets())}")

ast = Usd.Stage.Open(ANIM, Usd.Stage.LoadAll)
anim_prim = None
for p in Usd.PrimRange(ast.GetPseudoRoot()):
    if p.GetTypeName() == "SkelAnimation":
        anim_prim = p
        break
print(f"\nanim clip: {anim_prim.GetPath() if anim_prim else None}")
if anim_prim:
    a = UsdSkel.SkelAnimation(anim_prim)
    jn = a.GetJointsAttr().Get()
    print(f"  clip joints ({len(jn)}): {[str(x).split('/')[-1] for x in list(jn)[:8]]}")
    print(f"  stage tps={ast.GetTimeCodesPerSecond()} start={ast.GetStartTimeCode()} end={ast.GetEndTimeCode()}")
    # do joint name sets match?
    if skel_prim:
        sset = set(str(x) for x in skel_prim.GetAttribute("joints").Get())
        cset = set(str(x) for x in jn)
        print(f"  name match: {len(sset & cset)}/{len(sset)} character joints covered by clip")

app.close()
