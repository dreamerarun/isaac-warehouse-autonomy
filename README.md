# Isaac Sim warehouse autonomy

A small warehouse "micro-world" built on **Isaac Sim / Isaac Sim 5.1** with pure Python + USD:
two conveyor belts feed crates to a UR5 and a Franka, Nova Carters carry them to docks, forklifts
put them on pallets, and three animated people walk their own looping routes between the racks
while everything else keeps out of their way.

The cell stands on the open floor of NVIDIA's `full_warehouse` hall (racked aisles, 9 m ceiling,
ceiling lamps, painted floor), marked out with yellow tape, and the whole plant is on **ROS 2**:
every robot publishes odometry / tf / joint states, cameras stream as compressed images, a separate
WMS node releases crates with orders, and a web dashboard (through rosbridge) shows and controls it.

Everything is driven by one simulation loop (`logic.py` -> `Sim.step()`), physics is real
(articulations, rigid bodies, contacts), and every asset is measured off-screen before it is
placed, so a centimetre-authored asset can never end up 100x too big or floating above the floor.

## Run it

```bash
# interactive GUI
./python.sh ~/warehouse/step3_autonomy.py

# headless, 600 physics steps, no images
./python.sh ~/warehouse/step3_autonomy.py --headless --steps 600
```

Or let `run.sh` find the interpreter (Isaac Sim 5.1 `python.sh`, or a pip-installed Isaac Sim 6.x
venv such as the one on the DGX Spark / GB10, where it also preloads libgomp as aarch64 needs):

```bash
./run.sh --headless --steps 9000
ISAAC_PY=~/isaacsim/python.sh ./run.sh          # pick a specific install
```

Built on Isaac Sim 5.1 (RTX 4050 laptop); also runs on Isaac Sim 6.2 (pip) on aarch64 (DGX Spark / GB10).

Useful flags:

| flag | meaning |
| --- | --- |
| `--steps N` | run N physics steps (0 = until the window is closed) |
| `--shots N` | save a viewport frame every N steps into `shots/` |
| `--track N` | cinematic follow-cam on person N (side tracking shot) |
| `--record DIR` | write a numbered PNG frame sequence of that shot |
| `--every N` | one recorded frame every N physics steps (2 = 30 fps) |
| `--view ex,ey,ez,tx,ty,tz` | fixed camera eye + target instead of the overview (e.g. `8.6,0.6,1.7,8.6,4.2,0.6` = side view of the F1 hand-off) |
| `--focal MM` | focal length of that fixed camera (default 9) |
| `--ros` | ROS 2 interface (see below) |
| `--orders` | belts release a crate only against an order on `/warehouse/orders` |
| `--realtime` | pace the simulation to wall-clock time |
| `--room` | the old plain 26 x 16 m shell instead of the warehouse building |

## Whole stack: ROS 2 + WMS + web dashboard

```bash
./warehouse.sh start          # sim (headless, real time) + WMS + rosbridge :9090 + dashboard :8090
./warehouse.sh status
./warehouse.sh logs sim       # or wms | rosbridge | web
./warehouse.sh stop
./warehouse.sh start --gui    # Isaac Sim window on the local display as well
```

Open `http://<host>:8090/` (the cell needs a few minutes to load its assets). Everything runs on
ROS 2 Jazzy, `ROS_DOMAIN_ID=42` by default (`ROS_DOMAIN_ID=.. ./warehouse.sh start` to change), so
any ROS 2 machine on the LAN with the same domain sees the topics (RViz: fixed frame `map`,
add `/warehouse/markers`, `/map`, `/tf`).

| interface | type | what |
| --- | --- | --- |
| `/clock` | rosgraph_msgs/Clock | simulation time |
| `/tf`, `/tf_static` | tf2 | `map` -> `carter1..3/base_link`, `forklift1..2/base_link`, `ur5/base_link`, `franka/base_link`, people |
| `/carter{1,2,3}/odom`, `/forklift{1,2}/odom` | nav_msgs/Odometry | 30 Hz |
| `/forklift{1,2}/lift` | std_msgs/Float64 | fork height |
| `/ur5/joint_states`, `/franka/joint_states` | sensor_msgs/JointState | 30 Hz |
| `/warehouse/people`, `/warehouse/packages` | geometry_msgs/PoseArray | walkers + props, every crate |
| `/warehouse/markers` | visualization_msgs/MarkerArray | for RViz / Foxglove |
| `/map` | nav_msgs/OccupancyGrid (latched) | 0.1 m grid of the cell |
| `/warehouse/layout` | std_msgs/String JSON (latched) | belts, docks, pallets, obstacles |
| `/warehouse/state` | std_msgs/String JSON, 5 Hz | live state of every actor + KPIs |
| `/warehouse/events` | std_msgs/String | event log |
| `/warehouse/camera/{overview,follow}/compressed` | sensor_msgs/CompressedImage | 1280x720 jpeg, 4 Hz |
| `/warehouse/camera/follow_target` | std_msgs/String (in) | `C1`, `F2`, `ur5`, `person1`, `overview` ... |
| `/warehouse/orders` | std_msgs/String JSON (in) | `{"line": "A", "qty": 1}` releases crates |
| `/warehouse/pause`, `/warehouse/resume` | std_srvs/Trigger | freeze / continue the physics |
| `/warehouse/estop` | std_srvs/SetBool | machines and conveyors stop, people keep walking |
| `/wms/status` | std_msgs/String JSON | orders queued -> released -> on carter -> delivered, throughput |
| `/wms/new_order`, `/wms/config` | std_msgs/String JSON (in) | manual orders; `{"auto": true, "period_s": 45}` |

`tools/ros_smoke.py` checks all of it against the running cell (rates, orders, e-stop, pause):

```bash
source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 tools/ros_smoke.py
```

`CARTER_DIAG=10 ./run.sh --headless ...` prints every carter's and walker's position and state every
10 s (how the people/carter gridlocks below were found).
Full pallets (3 crates high) are shipped and replaced, so the cell can run indefinitely.

## People

The characters are NVIDIA SimReady `RL_BoneRoot` assets. Each one gets the Isaac in-place walk
clip retargeted onto its own skeleton at build time (`omni.anim.retarget.core`), and the
retargeted clip is measured, not assumed:

* **stride and cadence** - `_stride_probe()` walks the baked joint chain and measures how far each
  foot travels horizontally over the cycle. The clips are 1.333 s long with a 1.67-1.83 m stride,
  i.e. a natural 1.25-1.37 m/s. Each person is driven at `stride / cycle`, so the feet stay planted
  instead of sliding.
* **heading** - the rigs walk along their local **-Y**, the planner heading is +X, so the visuals
  are rotated by `PERSON_YAW_OFFSET_DEG = 90`. Without it they crab-walked sideways.
* **independent animation sources** - all characters share one omni-timeline playhead, so each one
  gets its own phase-shifted copy of the clip (`_phase_clip`). Otherwise they step in lockstep.
* **blocked legs** - while a person is blocked by a carter or another walker it switches its
  animation source to an averaged *standing* pose (`_stand_clip`), so it no longer walks in place.

`sim.time` pins the playhead to simulated time every frame (`people.step`), which keeps the gait
locked to ground speed even when rendering and physics run at different rates.

## Videos

`videos/person{0,1,2}.mp4` - 10 s side-tracking cinematic of each character walking its route,
captured with `--track`:

```bash
for i in 0 1 2; do
  ./python.sh ~/warehouse/step3_autonomy.py --headless --steps 600 --track $i \
      --record /tmp/frames_$i --every 2
  ffmpeg -framerate 30 -pattern_type glob -i "/tmp/frames_$i/*.png" \
      -c:v libx264 -pix_fmt yuv420p -crf 20 videos/person$i.mp4
done
```

`videos/warehouse_overview.mp4` - 150 s real-time overview of one full operation cycle from a
fixed wide-angle corner camera (the whole floor in one frame; both forklifts complete a delivery
in the last ten seconds):

```bash
./python.sh ~/warehouse/step3_autonomy.py --headless --steps 9000 --record /tmp/ov --every 3
ffmpeg -framerate 20 -pattern_type glob -i "/tmp/ov/*.png" \
    -c:v libx264 -pix_fmt yuv420p -crf 18 videos/warehouse_overview.mp4
```

## Dressing and lighting

`scene.py` lights the room with a 4x3 grid of ceiling fixtures (each a `RectLight` plus a visible
emissive diffuser panel) and a low-intensity dome for ambient fill, so no corner of the floor is
black. `statics.py` stacks pallets, plastic crates and cartons in the four corners, along the east
wall and along the bare north/south wall stretches west of the rack rows. Every proposed spot is
checked against the keep-out rectangles (belts, docks, forklift strips, carter parking) and the
footprints already placed, and is skipped if it would intrude on a lane; every stack that is placed
also registers its own footprint, so the navigation grid routes people and robots around it.

## Layout

```
config.py            every dimension, speed, asset path and tuning constant
scene.py             room shell, lights, conveyors
statics.py           racks, cartons, columns (measured and registered as obstacles)
assets.py            asset measuring/placing helpers (units, footprint, floor)
nav.py               occupancy grid + unicycle Mobile model
people.py            characters, retargeting, stride measurement, walk animation
logic.py             Sim: belts, arms, carters, forklifts, traffic arbitration
hw_isaac.py          hardware abstraction for the two arms
usd_kin.py           arm kinematics + collision filtering
step3_autonomy.py    builds the scene and runs the loop (CLI above)
ros_bridge.py        ROS 2 node /warehouse_sim inside the Isaac Sim process (topics, services, cameras)
wms_node.py          ROS 2 WMS node: auto/manual orders, tracks each order to the pallet
warehouse.sh         starts/stops sim + WMS + rosbridge + web dashboard
web/                 dashboard (static; roslib over rosbridge)
tools/ros_smoke.py   end-to-end ROS 2 check
check_*.py, test_*.py, debug_retarget.py, inspect_*.py   asset/retarget probes used during bring-up
```

## Traffic between people and robots

People and props have right of way; robots yield. Three gridlocks showed up only in 20-minute runs
and are fixed:

* a walker that ended a route inside an obstacle's comfort margin was refused every step and
  re-planned forever - it now keeps its route, finishes turning, and may cross the margin (never the
  obstacle) after 1.5 s;
* a carter inside a person's clearance ring had its speed zeroed every frame, even when backing away,
  while the person waited for the carter - carters may now move away, only closing in is refused;
* the roamer carter (C3) gave up a must-reach goal and fell through to a blind straight drive -
  must-reach goals are never abandoned; and two walkers meeting head-on now pass shoulder to shoulder.

## Known limitations

* The omni timeline is shared, so a person switches to a static standing pose when blocked rather
  than blending into it; the pose change is a hard cut, not a cross-fade.
* `MAX_RACKS` is deliberately small: the scene is tuned for a 6 GB laptop GPU (RTX 4050).
