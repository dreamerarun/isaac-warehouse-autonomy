"""Bind Isaac walk clips to a character skeleton and evaluate joint motion.
    ./python.sh ~/warehouse/test_walk.py"""
import sys
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdSkel, Gf

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
CHARS = {
    "male_con_01": f"{ROOT}/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd",
    "f_business": f"{ROOT}/Isaac/People/Characters/F_Business_02/F_Business_02.usd",
}
CLIPS = {
    "walk_in_place": f"{ROOT}/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd",
    "walk_loop": f"{ROOT}/Isaac/People/Animations/stand_walk_loop.skelanim.usd",
    "walk_1": f"{ROOT}/Isaac/People/Animations/stand_walk_1.skelanim.usd",
}

# ---- 1. inspect clip contents ------------------------------------------------
for cname, curl in CLIPS.items():
    st = Usd.Stage.Open(curl, Usd.Stage.LoadAll)
    print(f"[clip {cname}] mpu={UsdGeom.GetStageMetersPerUnit(st)} tps={st.GetTimeCodesPerSecond()} "
          f"range=[{st.GetStartTimeCode()}..{st.GetEndTimeCode()}]")
    for p in Usd.PrimRange(st.GetPseudoRoot()):
        if p.GetTypeName() == "SkelAnimation":
            a = UsdSkel.Animation(p)
            tr = a.GetTranslationsAttr()
            ts = tr.GetTimeSamples() if tr else []
            v0 = tr.Get(ts[0]) if ts else None
            # root joint = index 0
            print(f"   anim {p.GetPath()} joints={len(a.GetJointsAttr().Get() or [])} "
                  f"trans_samples={len(ts)} t0={ts[0] if ts else '-'}")
            if v0:
                import math
                mags = [round(math.sqrt(sum(c * c for c in v0[i])), 4) for i in range(min(6, len(v0)))]
                print(f"   translations[0..5] magnitudes at t0: {mags}")
                # hip-ish biggest?
                allm = [round(math.sqrt(sum(c * c for c in v)), 4) for v in v0]
                print(f"   max joint translation mag: {max(allm)} at idx {allm.index(max(allm))}")

# ---- 2. build composed stage: character + clip bound --------------------------
print("\n[compose]")
st = Usd.Stage.CreateNew("/tmp/walk_test.usda")
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
from pxr import Sdf
st.SetStartTimeCode(0); st.SetEndTimeCode(240); st.SetTimeCodesPerSecond(60)

def ref(url, path):
    prim = st.DefinePrim(path)
    prim.GetReferences().AddReference(url)
    return prim

char = ref(CHARS["male_con_01"], "/World/Person")
clip = ref(CLIPS["walk_in_place"], "/World/PersonClip")

skel_prim = None
for p in Usd.PrimRange(char, Usd.TraverseInstanceProxies()):
    if p.GetTypeName() == "Skeleton":
        skel_prim = p
        break
anim_prim = None
for p in Usd.PrimRange(clip, Usd.TraverseInstanceProxies()):
    if p.GetTypeName() == "SkelAnimation":
        anim_prim = p
        break
print(f"skel={skel_prim.GetPath()} anim={anim_prim.GetPath()}")
if skel_prim is None or anim_prim is None:
    print("MISSING prims"); sys.exit(1)

# joints match?
sj = set(str(x) for x in skel_prim.GetAttribute("joints").Get())
aj = set(str(x) for x in UsdSkel.Animation(anim_prim).GetJointsAttr().Get())
print(f"joint name overlap: {len(sj & aj)}/{len(sj)} char, {len(aj)} clip")
if not (sj & aj):
    print("NO OVERLAP - sample char:", list(sj)[:3], " clip:", list(aj)[:3])

# author animationSource on skeleton AND skelroot (try skeleton first)
bind_skel = UsdSkel.BindingAPI.Apply(skel_prim)
bind_skel.CreateAnimationSourceRel().SetTargets([anim_prim.GetPath()])
print(f"authored animationSource on {skel_prim.GetPath()}")

# find skelroot too, author there as well (harmless duplicate? may conflict -> skip if skeleton worked)
# ---- 3. evaluate joint world transforms over time -----------------------------
cache = UsdSkel.Cache()
for t in (0.0, 10.0, 20.0, 40.0):
    xform = UsdGeom.Xformable(char).ComputeLocalToWorldTransform(Usd.TimeCode(t))
    ok, joints = cache.ComputeJointsWorldTransforms(skel_prim, time=Usd.TimeCode(t))
    if not ok:
        print(f"t={t}: ComputeJointsWorldTransforms FAILED")
        continue
    # find a toe joint index
    order = [str(x) for x in skel_prim.GetAttribute("joints").Get()]
    ti = next((i for i, n in enumerate(order) if n.endswith("L_ToeBase")), None)
    if ti is None:
        ti = len(order) // 2
    m = joints[ti]
    p = Gf.Vec3d(m[3][0], m[3][1], m[3][2])
    wp = xform.Transform(p)
    print(f"t={t:5.1f}  {order[ti].split('/')[-1]}: local=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
          f"world=({wp[0]:.3f},{wp[1]:.3f},{wp[2]:.3f})")
print("done")
app.close()
