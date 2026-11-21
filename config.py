"""Single source of truth for warehouse layout (meters). Later steps import this."""

# ---- room -------------------------------------------------------------
ROOM_X = 26.0          # length  (x from -13 to +13)
ROOM_Y = 16.0          # width   (y from -8 to +8)
WALL_H = 5.0
WALL_T = 0.3

# ---- conveyors (two feeds, mirrored in y) -----------------------------
BELT_LEN = 6.0
BELT_W = 0.6
BELT_H = 0.75          # belt top surface height
BELT_SPEED = 0.25      # m/s
BELT_START_X = -11.5   # package spawn end
PICK_X = -5.5          # stopper sits just after this; package rests here
CONVEYORS = {
    "A": {"y": +5.0, "arm": "ur5"},
    "B": {"y": -5.0, "arm": "franka"},
}

# ---- manipulators: base sits beside the pick point --------------------
ARM_PEDESTAL_H = 0.90
ARM_OFFSET_Y = 0.55    # base distance from belt centreline (toward room centre)
# Reach (UR5 0.85 m, Franka 0.855 m) is verified in Step 3, not assumed.

# ---- carter docks: where an arm places a package ----------------------
DOCK_OFFSET_Y = 1.15   # from belt centreline, toward room centre
CARTER_DECK_H = 0.50   # approximate top height of the carrier; Step 4 measures it

# ---- packages ---------------------------------------------------------
PKG_SIZE = (1.0, 0.5, 0.30)   # (length along belt, depth, total height incl. skids) - recomputed from the forklift forks at start
PKG_MASS = 2.0
SPAWN_PERIOD_S = 12.0
MAX_ACTIVE_PKGS = 6    # per belt; keeps physics and VRAM small

# ---- forklifts + drop pallets (east wall) ------------------------------
FORKLIFTS = {
    "F1": {"pos": (10.5, +4.0), "yaw_deg": 180.0},
    "F2": {"pos": (10.5, -4.0), "yaw_deg": 180.0},
}
PALLET_SIZE = (1.2, 1.0, 0.15)
PALLET_OFFSET_X = -4.2  # pallet sits well clear of the forks (Step 5 moves it in)

# ---- carter start parking (Step 4 uses these) -------------------------
CARTER_START = {
    "C1": (-1.0, +1.5),
    "C2": (-1.0, -1.5),
    "C3": (0.0, 0.0),    # supervisor / dynamic obstacle
}

# ---- low-VRAM profile (RTX 4050 6 GB) ---------------------------------
RENDER_W, RENDER_H = 1280, 720
RENDERER = "RaytracedLighting"   # not PathTracing
NUM_CEILING_LIGHTS = 6
CEIL_LIGHT_COLS, CEIL_LIGHT_ROWS = 4, 3     # fixture grid across the ceiling (corners get one)
CEIL_LIGHT_SIZE = (2.6, 1.3)      # light emitting area
CEIL_PANEL_SIZE = (1.7, 0.85)     # visible diffuser; smaller than the light so a panel close
                                  # to the camera does not fill half the frame when seen edge-on
CEIL_LIGHT_INTENSITY = 11000.0
DOME_INTENSITY = 450.0                      # soft ambient fill so the far corners are not black

# ---- verified asset paths (from your assets_found.txt, Isaac Sim 5.1) ----
ASSETS = {
    "ur5":      "Isaac/Robots/UniversalRobots/ur5/ur5.usd",
    "franka":   "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    "carter":   "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    "forklift": "Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
    "pallet":   "Isaac/Props/Pallet/pallet.usd",
}


# ======================= Step 3: autonomy ===============================
CARTER_Z0 = 0.05            # carter spawn height (matches step2)
CARTER_DECK_H = 0.45        # WORLD height of the carter's top deck; printed bbox top helps tune
CARTER_SLOT = (-0.15, 0.0)  # package slot in carter frame (x fwd, y left)
CARTER_VMAX, CARTER_WMAX = 0.40, 1.3
CARTER_PRIMARY = {"A": "C1", "B": "C2"}   # which carter serves which belt
ROAMER = "C3"                              # roams, and backs up a busy primary
BACKUP_AFTER_S = 25.0

DOCK_YAW = 0.0              # carter heading while docked (parallel to belt)
DOCK_APPROACH = 2.0         # metres west of dock where carters line up

FORK_HOME_X = 10.5
FORK_SURF = 0.04            # fork top surface above fork lift height 0
FORK_CARRY_BACK = 0.55      # package centre this far behind the fork tip
FORK_CARRY_LIFT = 0.15
FORKLIFT_VISUAL_YAW_FIX = 0.0   # set 180 if the forks point away from the pallet
CARTER_TRANSFER_X = 7.65    # x where a carter hands the package to a forklift
PALLET_GRID = (3, 3, 4)     # packages per row, per column, layers
PALLET_PITCH = 0.30

SUCTION_LEN = 0.105         # cup length beyond the flange
CARTER_VISUAL_YAW_OFFSET_DEG = 0.0   # set 90/180/-90 if the carter drives sideways/backwards


# ---- runtime-derived values (filled in by step3_autonomy.py before the scene is built) -----------
PKG_SKID_H = 0.09           # clearance under the crate body for the forks
PKG_SKID_W = 0.12
CARTER_RADIUS = 0.65        # planning radius of a carter incl. its load deck
CARTER_PLATE = None         # dict(size=(L, W), thick, centre=(cx, cy), top_local) set at runtime
FORK_GEOM = {}              # per forklift: dict(tip=, surf=, thk=, width=)
STATIC_RECTS = []           # (x0, x1, y0, y1) footprints of racks / props, filled at runtime

# ---- real warehouse props (paths found by check_assets.py on your Isaac Sim 5.1 server) ------------
_SW = "Isaac/Environments/Simple_Warehouse/Props/"
STATIC_ASSETS = {
    "rack_frame": _SW + "SM_RackFrame_03.usd",
    "rack_shelf": _SW + "SM_RackShelf_01.usd",
    "rack_pile": [_SW + "SM_RackPile_03.usd", _SW + "SM_RackPile_04.usd", _SW + "SM_RackPile_06.usd"],
    "rack_shield": _SW + "SM_Rackshield_02.usd",
    "crates": [_SW + "SM_CratePlasticNote_A_01.usd", _SW + "SM_CratePlasticNote_B_03.usd"],
    "cartons": [_SW + "SM_CardBoxD_01.usd", _SW + "SM_CardBoxC_01.usd", _SW + "SM_CardBoxA_01.usd"],
    "pallet": "Isaac/Props/Pallet/pallet.usd",
}
MAX_RACKS = 10              # keeps the 6 GB GPU happy

# ---- corner / side dressing (only ever at the walls, never in a lane) ----
_PT = "Isaac/Props/PackingTable/props/"
STATIC_ASSETS["crate_plastic"] = [_SW + n for n in (
    "SM_CratePlastic_A_01.usd", "SM_CratePlastic_B_01.usd",
    "SM_CratePlastic_C_01.usd", "SM_CratePlastic_E_01.usd")]
STATIC_ASSETS["pack_crate"] = [_PT + n for n in (
    "SM_Crate_A07_Yellow_01.usd", "SM_Crate_A08_Blue_01.usd")]
STATIC_ASSETS["carton_white"] = [_PT + "sm_whitecorrugatedbox_b/sm_whitecorrugatedbox_b07_brown_01.usd",
                                _PT + "sm_whitecorrugatedbox_b/sm_whitecorrugatedbox_b08_brown_01.usd",
                                _PT + "sm_whitecorrugatedbox_b/sm_whitecorrugatedbox_b09_white_01.usd"]

# (tag, x, y, yaw, kind, half-extent) - every spot is validated against the keep-out zones at build time
CLUTTER_SPOTS = [
    ("NE", +11.90, +6.50, 0.0, "boxes", 0.55),
    ("SE", +11.90, -6.50, 0.0, "boxes", 0.55),
    ("NW", -10.40, +7.20, 0.0, "crates", 0.50),
    ("SW", -10.40, -7.20, 0.0, "crates", 0.50),
    ("E", +12.20, +0.80, 0.0, "crates", 0.60),
    ("E2", +12.20, -1.45, 0.0, "boxes", 0.60),
]

# ---- dynamic people + moving props (NVIDIA SimReady, wired in by step3_autonomy) ---------------
_PEOPLE = "Isaac/People/Characters/"
PEOPLE_ASSETS = [           # RL_BoneRoot characters validated end-to-end with the retargeted walk clip
    _PEOPLE + "male_adult_construction_01_new/male_adult_construction_01_new.usd",
    _PEOPLE + "F_Business_02/F_Business_02.usd",
    # NOTE: male_adult_construction_03/05 bake 101/101 joints but their rest-root orientation
    # does not match the identity-root clip, so they render lying flat - kept out on purpose.
]

BIPED_RIG = _PEOPLE + "biped_demo/biped_demo_meters.usd"   # source rig the clips match 81/81
WALK_CLIPS = [              # in-place walk loops (81 joints, t 0..80 @30 tps) - one per person so
    "Isaac/People/Animations/stand_walk_loop_in_place.skelanim.usd",   # they do not walk in lockstep
    "Isaac/People/Animations/stand_walk_1.skelanim.usd",
    "Isaac/People/Animations/stand_walk_2.skelanim.usd",
    "Isaac/People/Animations/stand_walk_3.skelanim.usd",
]
PEOPLE_N = 3                # walking people
PEOPLE_SPEED = 1.30         # m/s; the clips' measured natural pace is 1.25-1.37 m/s (1.7-1.8 m stride / 1.333 s)
PEOPLE_WALK_SPEED_SCALE = 0.70   # slow the walk a bit (animation + ground speed scaled together so feet stay planted)
PEOPLE_WMAX = 2.2           # rad/s turn-in-place limit
PEOPLE_GOALS = 5            # waypoints per looping route
PEOPLE_R = 0.45             # planning disk radius of a person
PEOPLE_REPLAN_D = 1.3       # replan a route when an obstacle gets this close (people_r + carter_r = 1.10 + margin)
PERSON_Z0 = -0.10           # floor offset of the character root (sinks the walk-clip's lifted feet onto the floor)
PERSON_YAW_OFFSET_DEG = 90.0 # the rigs walk along local -Y; +90 maps that onto the planner's +X heading
PUSHCART_ASSET = _SW + "SM_PushcartA_02.usd"   # shuttle cart: a moving warehouse prop obstacle
PUSHCART_SPEED = 0.10
PEOPLE_CONES = 2            # traffic cones: static nav obstacles registered for every robot
CONE_ASSET = _SW + "S_TrafficCone.usd"
PUSHCART_YAW_OFFSET_DEG = 0.0   # set 90/180/-90 if the cart visual faces sideways/backwards

# every person/prop is measured off-screen before it is spawned (same trick as statics._real):
# the size window fixes a wrongly declared metersPerUnit and the bounds place feet/wheels on z=0.
PERSON_SIZE = (1.2, 2.2)    # expected standing height in metres
CONE_SIZE = (0.15, 1.2)     # traffic cone extent in metres
PROP_SIZE = (0.30, 3.0)     # moving warehouse prop extent in metres

DOLLY_ASSET = "Isaac/Props/Dolly/dolly.usd"   # shelf cart (verified on your Isaac 5.1 asset server)
DOLLY_SPEED = 0.10
MOVING_PROPS = [            # name, asset, speed m/s, yaw offset deg, measured-size window
    ("pushcart", PUSHCART_ASSET, PUSHCART_SPEED, PUSHCART_YAW_OFFSET_DEG, PROP_SIZE),
    ("dolly", DOLLY_ASSET, DOLLY_SPEED, 0.0, PROP_SIZE),
]

# ---- round operation (Step 3b) ---------------------------------------------------------------------------
SUPPORT_BELT = "A"          # the belt whose line is shared with the support carter (ROAMER = C3)
PARK_SPOTS = {"C1": (-11.0, 2.3), "C3": (-11.0, 0.7), "C2": (-11.0, -2.3)}   # idle carters wait here, out of every lane
ROAM_WHEN_IDLE = False      # idle carters stay parked; no free roaming
