"""
Finds the real USD paths for robots/props on your Isaac Sim 5.1 asset server.
Writes a short clean list to assets_found.txt next to this script (no log noise).

    cd ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh ~/warehouse/check_assets.py
    cat ~/warehouse/assets_found.txt
Needs internet (assets are served from NVIDIA's S3 bucket).
"""
import os
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from isaacsim.storage.native import get_assets_root_path

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets_found.txt")
KEYS = ("ur5", "ur10", "franka", "carter", "forklift", "pallet", "conveyor",
        "cardboard", "box", "shelf", "rack", "crate")
ROOTS = (("Isaac/Robots", 4), ("Isaac/Props", 3), ("Isaac/Environments/Simple_Warehouse", 3))
SKIP = ("materials", "textures", "configuration", "payloads", "sublayers", "meshes", "icons", ".thumbs")

root = get_assets_root_path()
lines = [f"ASSET ROOT: {root}"]
found = []


def walk(url, depth, max_depth):
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        return
    for e in entries:
        name = e.relative_path
        full = f"{url}/{name}".rstrip("/")
        is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
        if is_dir:
            if depth < max_depth and name.lower().strip("/") not in SKIP:
                walk(full, depth + 1, max_depth)
        elif name.lower().endswith((".usd", ".usda", ".usdc")):
            if any(k in full.lower() for k in KEYS):
                found.append(full.replace(root + "/", ""))


if not root:
    lines.append("NO ASSET ROOT - check internet connection")
else:
    for r, d in ROOTS:
        walk(f"{root}/{r}", 0, d)
    lines += sorted(set(found))

with open(OUT, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"WROTE {len(found)} paths to {OUT}")
app.close()
