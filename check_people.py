"""
Discover human/pedestrian and dynamic warehouse prop assets on your Isaac Sim 5.1 asset server.
Writes people_assets.txt next to this script.

    cd ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh ~/warehouse/check_people.py
Needs internet (assets are served from NVIDIA's S3 bucket).
"""
import os
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from isaacsim.storage.native import get_assets_root_path

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "people_assets.txt")

# folders worth scanning for people / moving props
ROOTS = (
    ("Isaac/People", 6),
    ("Isaac/Props", 3),
    ("Isaac/Environments/Simple_Warehouse/Props", 2),
)
# keep only files whose path mentions these (lowercased)
KEEP = ("human", "pedestrian", "person", "people", "character", "man", "woman", "worker",
        "cart", "trolley", "jack", "pallettruck", "dolly", "barrier", "cone", "forklift",
        "wagon", "handtruck", "truck", "bin", "drum", "barrel", "barrow")
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
            if any(k in full.lower() for k in KEEP):
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
