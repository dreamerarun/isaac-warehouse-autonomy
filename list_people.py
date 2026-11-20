"""Full recursive listing of Isaac/People + biped_demo folder, and resolve biped_demo animationSource.
    ./python.sh ~/warehouse/list_people.py"""
import sys
sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from pxr import Usd, UsdSkel
from isaacsim.storage.native import get_assets_root_path

root = get_assets_root_path()
OUT = "/home/arun/warehouse/people_tree.txt"
lines = []


def walk(url, depth=0):
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        return
    for e in entries:
        name = e.relative_path
        full = f"{url}/{name}".rstrip("/")
        is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
        rel = full.replace(root + "/", "")
        lines.append(("D " if is_dir else "  ") + rel)
        if is_dir and depth < 6:
            walk(full, depth + 1)


walk(f"{root}/Isaac/People")
with open(OUT, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"[tree] wrote {len(lines)} entries to {OUT}")

# biped_demo details
url = f"{root}/Isaac/People/Characters/biped_demo/biped_demo_meters.usd"
st = Usd.Stage.Open(url, Usd.Stage.LoadAll)
for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if p.GetTypeName() in ("Skeleton", "SkelRoot", "SkelAnimation") or "Graph" in p.GetName():
        print(f"  biped prim {p.GetPath()} type={p.GetTypeName()}")
        if p.GetTypeName() == "Skeleton":
            js = p.GetAttribute("joints").Get()
            print(f"    joints={len(js)} first={list(js)[:4]}")
            b = UsdSkel.BindingAPI(p)
            print(f"    animSource targets={list(b.GetAnimationSourceRel().GetTargets())}")
            print(f"    skeleton rel={list(b.GetSkeletonRel().GetTargets())}")
        if p.GetTypeName() == "SkelRoot":
            b = UsdSkel.BindingAPI(p)
            print(f"    skeleton rel={list(b.GetSkeletonRel().GetTargets())} "
                  f"animSource={list(b.GetAnimationSourceRel().GetTargets())}")
print("[done]")
app.close()
