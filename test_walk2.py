"""Compare walk-clip rig vs character rigs: leaf-name overlap + biped_demo binding test.
    ./python.sh ~/warehouse/test_walk2.py"""
import sys, math
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdSkel, Gf

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
CHAR = f"{ROOT}/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
BIPED = f"{ROOT}/Isaac/People/Characters/biped_demo/biped_demo_meters.usd"
CLIP = f"{ROOT}/Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd"


def joints_of(url):
    st = Usd.Stage.Open(url, Usd.Stage.LoadAll)
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == "Skeleton":
            return st, p, [str(x) for x in p.GetAttribute("joints").Get()]
    return st, None, []


def clip_of(url):
    st = Usd.Stage.Open(url, Usd.Stage.LoadAll)
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == "SkelAnimation":
            a = UsdSkel.Animation(p)
            return st, p, [str(x) for x in a.GetJointsAttr().Get()]
    return st, None, []


def leaf(p):
    return p.split("/")[-1]


cst, csk, cj = joints_of(CHAR)
bst, bsk, bj = joints_of(BIPED)
ast, ap, aj = clip_of(CLIP)
print(f"[char] {len(cj)} joints, first={cj[0]}")
print(f"[biped] {len(bj)} joints, first={bj[0]}")
print(f"[clip] {len(aj)} joints, first={aj[0]}")

cset, bset, aset = set(map(leaf, cj)), set(map(leaf, bj)), set(map(leaf, aj))
print(f"[leaf-overlap] char vs clip: {len(cset & aset)}/{len(aset)} clip leaves matched; "
      f"unmatched clip leaves = {sorted(aset - cset)}")
print(f"[leaf-overlap] biped vs clip: {len(bset & aset)}/{len(aset)}; unmatched = {sorted(bset - aset)}")
print(f"[fullpath] char vs clip: {len(set(cj) & set(aj))}, biped vs clip: {len(set(bj) & set(aj))}")

# full-path match for biped means direct binding works
print("\n[bind biped + clip, evaluate]")
st = Usd.Stage.CreateNew("/tmp/walk_biped.usda")
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
st.SetStartTimeCode(0); st.SetEndTimeCode(240); st.SetTimeCodesPerSecond(60)
b = st.DefinePrim("/World/Biped"); b.GetReferences().AddReference(BIPED)
c = st.DefinePrim("/World/Clip"); c.GetReferences().AddReference(CLIP)
sk = ap_ = None
for p in Usd.PrimRange(b, Usd.TraverseInstanceProxies()):
    if p.GetTypeName() == "Skeleton":
        sk = p
for p in Usd.PrimRange(c, Usd.TraverseInstanceProxies()):
    if p.GetTypeName() == "SkelAnimation":
        ap_ = p
print(f"  skel={sk.GetPath()} anim={ap_.GetPath()}")
sj = [str(x) for x in sk.GetAttribute("joints").Get()]
aj2 = [str(x) for x in UsdSkel.Animation(ap_).GetJointsAttr().Get()]
print(f"  exact path overlap: {len(set(sj) & set(aj2))}/{len(sj)}")
UsdSkel.BindingAPI.Apply(sk).CreateAnimationSourceRel().SetTargets([ap_.GetPath()])

cache = UsdSkel.Cache()
methods = [m for m in dir(cache) if "Joint" in m or "World" in m]
print(f"  cache methods: {methods}")
order = sj
try:
    fn = getattr(cache, "ComputeJointsWorldTransforms")
except AttributeError:
    fn = None
ti = next((i for i, n in enumerate(order) if n.endswith("L_ToeBase") or n.endswith("L_Foot") or n.endswith("LeftFoot")), len(order)//2)
for t in (0.0, 10.0, 20.0, 40.0):
    if fn:
        ok, mats = fn(sk, time=Usd.TimeCode(t))
        if not ok:
            print(f"  t={t}: FAILED"); continue
        m = mats[ti]
        p = Gf.Vec3d(m[3][0], m[3][1], m[3][2])
        print(f"  t={t:5.1f} {order[ti]}: local=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
print("done")
app.close()
