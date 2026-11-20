"""Find which human characters match which walk clips (joint naming) and which ship embedded animations.
    ./python.sh ~/warehouse/check_match.py"""
import sys
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from pxr import Usd, UsdGeom, UsdSkel
from isaacsim.storage.native import get_assets_root_path

root = get_assets_root_path()

# ---- full listing of People folders --------------------------------------
def listdir(rel):
    res, entries = omni.client.list(f"{root}/{rel}")
    if res != omni.client.Result.OK:
        return []
    return [e.relative_path.rstrip("/") for e in entries]

chars = [c for c in listdir("Isaac/People/Characters") if not c.startswith(".")]
anims = [a for a in listdir("Isaac/People/Animations") if not a.startswith(".")]
print(f"[chars] {len(chars)}: {chars}")
print(f"[anims] {len(anims)}: {anims}")

# ---- for each character: skeleton joint root, embedded anims, animSource ----
info = {}
for c in chars:
    # find the main usd inside
    res, entries = omni.client.list(f"{root}/Isaac/People/Characters/{c}")
    usds = [e.relative_path for e in entries if e.relative_path.lower().endswith((".usd", ".usda", ".usdc"))
            and not e.relative_path.startswith(".")]
    if not usds:
        print(f"  {c}: no usd"); continue
    main = sorted(usds, key=len)[0]
    url = f"{root}/Isaac/People/Characters/{c}/{main}"
    try:
        st = Usd.Stage.Open(url, Usd.Stage.LoadAll)
    except Exception as e:
        print(f"  {c}: OPEN FAIL {e}"); continue
    skel = anim = 0; jroot = None; anims_in = []; asrc = False
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        t = p.GetTypeName()
        if t == "Skeleton" and skel == 0:
            skel += 1
            js = p.GetAttribute("joints").Get()
            if js:
                jroot = str(list(js)[0]).split("/")[0]
            b = UsdSkel.BindingAPI(p)
            asrc = b.GetAnimationSourceRel().HasAuthoredTargets()
        elif t == "SkelAnimation":
            anim += 1
            a = UsdSkel.SkelAnimation(p)
            n = a.GetJointsAttr().Get()
            anims_in.append((p.GetPath().pathString, len(n or [])))
    info[c] = (jroot, anim, asrc, main, anims_in)
    print(f"  {c}: skel_root={jroot} embedded_anims={anim} animSource={asrc} file={main} {anims_in[:2]}")

# ---- clip joint roots -----------------------------------------------------
print("\n[clip joint roots]")
cliproots = {}
for a in anims:
    if not a.endswith((".usd", ".usda", ".usdc")):
        continue
    try:
        st = Usd.Stage.Open(f"{root}/Isaac/People/Animations/{a}", Usd.Stage.LoadAll)
    except Exception:
        continue
    for p in Usd.PrimRange(st.GetPseudoRoot()):
        if p.GetTypeName() == "SkelAnimation":
            js = UsdSkel.Animation(p).GetJointsAttr().Get()
            r = str(list(js)[0]).split("/")[0] if js else None
            cliproots[a] = (r, len(js or []))
            break
for k, v in cliproots.items():
    print(f"  {k}: root={v[0]} n={v[1]}")

# ---- matching characters to clips ----------------------------------------
print("\n[MATCH] characters whose skel root matches a clip root:")
croots = set(v[0] for v in cliproots.values())
for c, v in info.items():
    if v[0] in croots:
        print(f"  {c} (root={v[0]}) matches clips")
    if v[1] > 0:
        print(f"  {c} has {v[1]} EMBEDDED animations (usable directly)")

app.close()
