"""
Inspect human / prop USD structure: units, bbox orientation, skeleton + animation availability.
    ./python.sh ~/warehouse/inspect_assets.py
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.stdout.reconfigure(line_buffering=True)
from pxr import Usd, UsdGeom, UsdSkel
from isaacsim.storage.native import get_assets_root_path

root = get_assets_root_path()
URLS = [
    "Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd",
    "Isaac/People/Characters/F_Business_02/F_Business_02.usd",
    "Isaac/People/Characters/biped_demo/biped_demo_meters.usd",
    "Isaac/People/Animations/stand_walk_loop.skelanim.usd",
    "Isaac/Environments/Simple_Warehouse/Props/SM_PushcartA_02.usd",
    "Isaac/Props/Dolly/dolly.usd",
]

for rel in URLS:
    url = f"{root}/{rel}"
    print(f"\n===== {rel}")
    try:
        st = Usd.Stage.Open(url, Usd.Stage.LoadNone)
    except Exception as e:
        print(f"  OPEN FAILED: {e}")
        continue
    mpu = float(UsdGeom.GetStageMetersPerUnit(st)) or 1.0
    print(f"  metersPerUnit={mpu}  defaultPrim={st.GetDefaultPrim().GetPath() if st.GetDefaultPrim() else None}")
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rootp = st.GetDefaultPrim() or st.GetPseudoRoot()
    try:
        r = cache.ComputeWorldBound(rootp).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"  bbox native min={ [round(v,3) for v in mn] } max={ [round(v,3) for v in mx] }"
              f"  -> metres size={ [round((mx[i]-mn[i])*mpu,3) for i in range(3)] }")
    except Exception as e:
        print(f"  bbox failed: {e}")
    skel = anim = graph = 0
    samples = []
    for p in Usd.PrimRange(rootp, Usd.TraverseInstanceProxies()):
        t = p.GetTypeName()
        if t == "Skeleton":
            skel += 1
        elif t == "SkelAnimation":
            anim += 1
            attr = p.GetAttribute("translations")
            if attr and attr.HasAuthoredValueOpinion():
                times = attr.GetTimeSamples()
                if times:
                    samples.append((p.GetPath().pathString, len(times), times[0], times[-1]))
        if "AnimationGraph" in p.GetName() or t == "UsdAnimationGraph" or "Graph" in p.GetName():
            graph += 1
            if graph <= 3:
                print(f"  graph prim: {p.GetPath()} type={t}")
    print(f"  Skeletons={skel} SkelAnimations={anim} graph-ish={graph}")
    for s in samples[:4]:
        print(f"  anim samples: {s[0]} n={s[1]} t=[{s[2]:.3f}..{s[3]:.3f}]")
    # interesting attributes on root / graph prims
    for p in Usd.PrimRange(rootp, Usd.TraverseInstanceProxies()):
        if p.GetName() in ("AnimationGraph", "root", "Character", "humanoid") or "Graph" in p.GetName():
            attrs = [a.GetName() for a in p.GetAttributes()][:25]
            print(f"  attrs of {p.GetPath()}: {attrs}")
            break

app.close()
