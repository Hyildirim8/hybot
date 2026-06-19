"""
ecza-robotu Isaac Sim 4.5 — standalone simulation launcher.

Run from terminal on the Isaac Sim machine (mucahit@10.42.101.217):

    cd ~/isaacsim
    ./python.sh ~/isaac_sim_standalone.py

This script is now self-contained — it sets LD_LIBRARY_PATH before Isaac Sim
initialises, so the ROS2 bridge always loads correctly regardless of whether
you used launch_isaac_sim.sh.
"""

# ─── 0. ENVIRONMENT — must be set BEFORE importing Isaac Sim ─────────────────
import os

_BRIDGE_LIB = os.path.expanduser(
    "~/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
)
if os.path.isdir(_BRIDGE_LIB):
    # Prepend to LD_LIBRARY_PATH so dlopen finds librmw_fastrtps_cpp.so
    os.environ["LD_LIBRARY_PATH"] = (
        _BRIDGE_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    )
    print(f"LD_LIBRARY_PATH prepended: {_BRIDGE_LIB}")
else:
    print(f"WARNING: bridge lib dir not found: {_BRIDGE_LIB}")

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ.setdefault("ROS_DISTRO",   "humble")
os.environ.setdefault("ROS_DOMAIN_ID", "0")

# ─── 1. START ISAAC SIM ──────────────────────────────────────────────────────
from isaacsim import SimulationApp

sim_app = SimulationApp({
    "headless":      False,
    "width":         1920,
    "height":        1080,
    "renderer":      "RayTracedLighting",
    "anti_aliasing": 1,
})

# ─── 2. IMPORTS ──────────────────────────────────────────────────────────────
import omni
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, PhysxSchema, Usd

# ─── 3. CONFIGURATION ────────────────────────────────────────────────────────

# Your saved USD scene (seen in logs: /home/mucahit/Downloads/warehouse_slam.usd)
EXISTING_USD = "/home/mucahit/Downloads/warehouse_slam.usd"

# Leave EXISTING_USD = "" to import from URDF instead:
URDF_PATH = "/home/master/Workspace/ecza-robotu/src/ecza_description/urdf/rover.urdf"

# Robot prim (confirmed from logs: /World/ecza_rover)
ROBOT_PRIM_HINT = "/World/ecza_rover"

# Wheel joints (confirmed found at /World/ecza_rover/joints/*)
JOINT_NAMES   = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]
JOINT_DAMPING = 1e5
JOINT_STIFF   = 0.0

# Geometry (rover_params.yaml)
WHEEL_RADIUS   = 0.04
WHEEL_BASE     = 0.38
WHEEL_WIDTH    = 0.26
MAX_SPEED      = 1.5
MECANUM_ANGLES = [45.0, -45.0, -45.0, 45.0]

# ROS2 topics
SCAN_TOPIC    = "scan_sim"
ODOM_TOPIC    = "odom"
CMDVEL_TOPIC  = "cmd_vel"
SCAN_FRAME    = "laser_frame"
ODOM_FRAME    = "world"
CHASSIS_FRAME = "base_footprint"
GRAPH_PATH    = "/World/ActionGraph"

# ─── 4. LOAD SCENE ───────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

if EXISTING_USD:
    add_reference_to_stage(EXISTING_USD, "/World")
    print(f"Loaded: {EXISTING_USD}")
else:
    print(f"Importing URDF: {URDF_PATH}")
    ok, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=URDF_PATH, import_config=None
    )
    if not ok:
        print("  URDF import failed — set EXISTING_USD to your .usd scene file")

world.reset()

# ─── 5. AUTO-DETECT ROBOT PRIM ───────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

def find_robot_prim():
    if ROBOT_PRIM_HINT and stage.GetPrimAtPath(ROBOT_PRIM_HINT).IsValid():
        return ROBOT_PRIM_HINT
    for candidate in ["/World/ecza_rover", "/World/ecza_robot", "/World/rover"]:
        if stage.GetPrimAtPath(candidate).IsValid():
            return candidate
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            p = str(prim.GetPath())
            if p.startswith("/World/"):
                return p
    return "/World/ecza_rover"

ROBOT_PRIM = find_robot_prim()
print(f"Robot prim: {ROBOT_PRIM}")

# ─── 6. WHEEL JOINT DRIVES (velocity mode) ───────────────────────────────────
print("Setting wheel joints → velocity drive...")

def configure_joints():
    root = stage.GetPrimAtPath(ROBOT_PRIM)
    if not root.IsValid():
        print(f"  ERROR: {ROBOT_PRIM} not found"); return
    name_map = {p.GetName(): p for p in Usd.PrimRange(root)}
    for jname in JOINT_NAMES:
        prim = name_map.get(jname)
        if prim is None:
            print(f"  WARNING: '{jname}' not found")
            continue
        d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateTypeAttr("velocity")
        d.CreateStiffnessAttr(JOINT_STIFF)
        d.CreateDampingAttr(JOINT_DAMPING)
        print(f"  ✓ {prim.GetPath()}")

configure_joints()

# ─── 7. ENABLE EXTENSIONS ────────────────────────────────────────────────────
ext_mgr = omni.kit.app.get_app().get_extension_manager()

def try_enable(eid):
    if not ext_mgr.is_extension_enabled(eid):
        try:
            ext_mgr.set_extension_enabled_immediate(eid, True)
            print(f"  enabled: {eid}")
        except Exception as e:
            print(f"  could not enable {eid}: {e}")

print("Enabling extensions...")
try_enable("isaacsim.ros2.bridge")
try_enable("omni.isaac.ros2_bridge")
try_enable("omni.isaac.wheeled_robots")
try_enable("omni.isaac.core_nodes")
try_enable("omni.isaac.range_sensor")

# Let extensions finish registering their OmniGraph node types
print("Flushing extension registrations (10 update ticks)...")
for _ in range(10):
    sim_app.update()

# ─── 8. PATCH OMNIGRAPH ──────────────────────────────────────────────────────
# Strategy:
#   The existing USD graph already has all the ROS2 publishing/subscribing nodes.
#   We only need to ADD HolonomicController + ArticulationController and connect
#   them to the existing OnPlaybackTick and ROS2SubscribeTwist nodes.
#   We do NOT need to probe type names for existing nodes — only for the two new ones.

hb = WHEEL_BASE  / 2.0
hw = WHEEL_WIDTH / 2.0
wheel_positions   = [[ hb, hw, 0.0], [ hb, -hw, 0.0],
                     [-hb, hw, 0.0], [-hb, -hw, 0.0]]
wheel_orientations = [[0.0, 0.0, 0.0, 1.0]] * 4

keys = og.Controller.Keys

drive_values = [
    ("HolonomicController.inputs:wheelRadius",       [WHEEL_RADIUS] * 4),
    ("HolonomicController.inputs:wheelPositions",    wheel_positions),
    ("HolonomicController.inputs:wheelOrientations", wheel_orientations),
    ("HolonomicController.inputs:mecanum",           True),
    ("HolonomicController.inputs:mecanumAngles",     MECANUM_ANGLES),
    ("HolonomicController.inputs:maxLinearSpeed",    MAX_SPEED),
    ("ArticulationController.inputs:robotPath",      ROBOT_PRIM),
    ("ArticulationController.inputs:usePath",        True),
    ("ArticulationController.inputs:jointNames",     JOINT_NAMES),
]

existing_graph = og.get_graph_by_path(GRAPH_PATH)
graph_exists   = existing_graph is not None and existing_graph.is_valid()

if graph_exists:
    print(f"\nPatching existing graph at {GRAPH_PATH}...")

    # Inventory existing nodes
    existing_type_to_path = {}
    for node in existing_graph.get_nodes():
        existing_type_to_path[node.get_type_name()] = node.get_node_path()
        print(f"  node: {node.get_node_path().split('/')[-1]}  [{node.get_type_name()}]")

    def find_path(*fragments):
        for t, p in existing_type_to_path.items():
            if any(f.lower() in t.lower() for f in fragments):
                return p
        return None

    tick_path  = find_path("OnPlaybackTick", "playback_tick")
    twist_path = find_path("SubscribeTwist", "subscribe_twist")
    print(f"\n  Tick node  : {tick_path}")
    print(f"  Twist node : {twist_path}")

    holo_exists  = any("Holonomic" in t for t in existing_type_to_path)
    artic_exists = any("ArticulationController" in t for t in existing_type_to_path)

    if holo_exists and artic_exists:
        print("  Drive nodes already present — updating values only.")
        og.Controller.edit(existing_graph, {keys.SET_VALUES: drive_values})
    else:
        # Build connection list using discovered existing node paths
        connects = []
        if tick_path and not holo_exists:
            connects.append((f"{tick_path}.outputs:tick",
                             "HolonomicController.inputs:execIn"))
        if twist_path:
            connects.append((f"{twist_path}.outputs:linearVelocity",
                             "HolonomicController.inputs:linearVelocity"))
            connects.append((f"{twist_path}.outputs:angularVelocity",
                             "HolonomicController.inputs:angularVelocity"))
        connects.append(("HolonomicController.outputs:execOut",
                         "ArticulationController.inputs:execIn"))
        connects.append(("HolonomicController.outputs:jointVelocityCommand",
                         "ArticulationController.inputs:velocityCommand"))

        # Try both old (omni.isaac.*) and new (isaacsim.*) namespaces
        HOLO_TYPES  = ["omni.isaac.wheeled_robots.HolonomicController",
                       "isaacsim.robot.wheeled_robots.HolonomicController"]
        ARTIC_TYPES = ["omni.isaac.core_nodes.IsaacArticulationController",
                       "isaacsim.core.nodes.IsaacArticulationController"]

        patched = False
        for holo_t in HOLO_TYPES:
            for artic_t in ARTIC_TYPES:
                nodes_to_add = []
                if not holo_exists:
                    nodes_to_add.append(("HolonomicController",    holo_t))
                if not artic_exists:
                    nodes_to_add.append(("ArticulationController", artic_t))

                try:
                    og.Controller.edit(existing_graph, {
                        keys.CREATE_NODES: nodes_to_add,
                        keys.CONNECT:      connects,
                        keys.SET_VALUES:   drive_values,
                    })
                    print(f"  ✓ HolonomicController    : {holo_t}")
                    print(f"  ✓ ArticulationController : {artic_t}")
                    patched = True
                    break
                except Exception as e:
                    print(f"  failed {holo_t} + {artic_t}: {type(e).__name__}: {str(e)[:100]}")
                    # Clean up any partially added prims
                    for name in ["HolonomicController", "ArticulationController"]:
                        pp = f"{GRAPH_PATH}/{name}"
                        if stage.GetPrimAtPath(pp).IsValid():
                            stage.RemovePrim(pp)
            if patched:
                break

        if not patched:
            print("\n  ERROR: Could not add drive nodes.")
            print("  Check that omni.isaac.wheeled_robots extension loaded above.")

else:
    # ── FULL BUILD (no existing graph) ────────────────────────────────────────
    print(f"\nNo existing graph — building complete OmniGraph at {GRAPH_PATH}...")

    LIDAR_PRIM = f"{ROBOT_PRIM}/base_link/Lidar"
    for lp in [f"{ROBOT_PRIM}/base_link/Lidar", f"{ROBOT_PRIM}/lidar",
               f"{ROBOT_PRIM}/laser_frame/Lidar"]:
        if stage.GetPrimAtPath(lp).IsValid():
            LIDAR_PRIM = lp
            break

    # Try isaacsim.* namespace first (Isaac Sim 4.5), fall back to omni.isaac.*
    BUILD_CONFIGS = [
        {   # Isaac Sim 4.5 namespaces
            "ctx":   "isaacsim.ros2.bridge.ROS2Context",
            "clock": "omni.isaac.core_nodes.IsaacReadSystemTime",
            "lidar": "omni.isaac.range_sensor.IsaacReadLidarBeams",
            "scan":  "isaacsim.ros2.bridge.ROS2PublishLaserScan",
            "odomc": "omni.isaac.core_nodes.IsaacComputeOdometry",
            "odomp": "isaacsim.ros2.bridge.ROS2PublishOdometry",
            "tf":    "isaacsim.ros2.bridge.ROS2PublishTransformTree",
            "twist": "isaacsim.ros2.bridge.ROS2SubscribeTwist",
            "holo":  "omni.isaac.wheeled_robots.HolonomicController",
            "artic": "omni.isaac.core_nodes.IsaacArticulationController",
        },
        {   # legacy omni.isaac.* namespaces
            "ctx":   "omni.isaac.ros2_bridge.ROS2Context",
            "clock": "omni.isaac.core_nodes.IsaacReadSystemTime",
            "lidar": "omni.isaac.range_sensor.IsaacReadLidarBeams",
            "scan":  "omni.isaac.ros2_bridge.ROS2PublishLaserScan",
            "odomc": "omni.isaac.core_nodes.IsaacComputeOdometry",
            "odomp": "omni.isaac.ros2_bridge.ROS2PublishOdometry",
            "tf":    "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
            "twist": "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
            "holo":  "omni.isaac.wheeled_robots.HolonomicController",
            "artic": "omni.isaac.core_nodes.IsaacArticulationController",
        },
    ]

    for cfg in BUILD_CONFIGS:
        try:
            og.Controller.edit(
                {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick",         "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context",            cfg["ctx"]),
                        ("IsaacReadSystemTime",    cfg["clock"]),
                        ("IsaacReadLidar",         cfg["lidar"]),
                        ("ROS2PublishLaserScan",   cfg["scan"]),
                        ("IsaacComputeOdometry",   cfg["odomc"]),
                        ("ROS2PublishOdometry",    cfg["odomp"]),
                        ("ROS2PublishTF",          cfg["tf"]),
                        ("ROS2SubscribeTwist",     cfg["twist"]),
                        ("HolonomicController",    cfg["holo"]),
                        ("ArticulationController", cfg["artic"]),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick",         "IsaacReadLidar.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick",         "IsaacComputeOdometry.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick",         "ROS2SubscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick",         "HolonomicController.inputs:execIn"),
                        ("ROS2Context.outputs:context",         "ROS2PublishLaserScan.inputs:context"),
                        ("ROS2Context.outputs:context",         "ROS2PublishOdometry.inputs:context"),
                        ("ROS2Context.outputs:context",         "ROS2PublishTF.inputs:context"),
                        ("ROS2Context.outputs:context",         "ROS2SubscribeTwist.inputs:context"),
                        ("IsaacReadSystemTime.outputs:systemTime","ROS2PublishLaserScan.inputs:timeStamp"),
                        ("IsaacReadSystemTime.outputs:systemTime","ROS2PublishOdometry.inputs:timeStamp"),
                        ("IsaacReadSystemTime.outputs:systemTime","ROS2PublishTF.inputs:timeStamp"),
                        ("IsaacReadLidar.outputs:execOut",              "ROS2PublishLaserScan.inputs:execIn"),
                        ("IsaacReadLidar.outputs:azimuthRange",         "ROS2PublishLaserScan.inputs:azimuthRange"),
                        ("IsaacReadLidar.outputs:depthRange",           "ROS2PublishLaserScan.inputs:depthRange"),
                        ("IsaacReadLidar.outputs:horizontalFov",        "ROS2PublishLaserScan.inputs:horizontalFov"),
                        ("IsaacReadLidar.outputs:horizontalResolution", "ROS2PublishLaserScan.inputs:horizontalResolution"),
                        ("IsaacReadLidar.outputs:intensitiesData",      "ROS2PublishLaserScan.inputs:intensitiesData"),
                        ("IsaacReadLidar.outputs:linearDepthData",      "ROS2PublishLaserScan.inputs:linearDepthData"),
                        ("IsaacReadLidar.outputs:numCols",              "ROS2PublishLaserScan.inputs:numCols"),
                        ("IsaacReadLidar.outputs:numRows",              "ROS2PublishLaserScan.inputs:numRows"),
                        ("IsaacReadLidar.outputs:rotationRate",         "ROS2PublishLaserScan.inputs:rotationRate"),
                        ("ROS2PublishLaserScan.outputs:execOut",        "ROS2PublishTF.inputs:execIn"),
                        ("IsaacComputeOdometry.outputs:execOut",        "ROS2PublishOdometry.inputs:execIn"),
                        ("IsaacComputeOdometry.outputs:position",       "ROS2PublishOdometry.inputs:position"),
                        ("IsaacComputeOdometry.outputs:orientation",    "ROS2PublishOdometry.inputs:orientation"),
                        ("IsaacComputeOdometry.outputs:linearVelocity", "ROS2PublishOdometry.inputs:linearVelocity"),
                        ("IsaacComputeOdometry.outputs:angularVelocity","ROS2PublishOdometry.inputs:angularVelocity"),
                        ("ROS2SubscribeTwist.outputs:linearVelocity",   "HolonomicController.inputs:linearVelocity"),
                        ("ROS2SubscribeTwist.outputs:angularVelocity",  "HolonomicController.inputs:angularVelocity"),
                        ("HolonomicController.outputs:execOut",         "ArticulationController.inputs:execIn"),
                        ("HolonomicController.outputs:jointVelocityCommand","ArticulationController.inputs:velocityCommand"),
                    ],
                    keys.SET_VALUES: [
                        ("IsaacReadLidar.inputs:lidarPrim",          [LIDAR_PRIM]),
                        ("ROS2PublishLaserScan.inputs:topicName",    SCAN_TOPIC),
                        ("ROS2PublishLaserScan.inputs:frameId",      SCAN_FRAME),
                        ("IsaacComputeOdometry.inputs:chassisPrim",  ROBOT_PRIM),
                        ("ROS2PublishOdometry.inputs:topicName",     ODOM_TOPIC),
                        ("ROS2PublishOdometry.inputs:odomFrameId",   ODOM_FRAME),
                        ("ROS2PublishOdometry.inputs:chassisFrameId",CHASSIS_FRAME),
                        ("ROS2PublishTF.inputs:targetPrims",         [ROBOT_PRIM]),
                        ("ROS2SubscribeTwist.inputs:topicName",      CMDVEL_TOPIC),
                    ] + drive_values,
                }
            )
            print(f"  ✓ Complete graph created (namespace: {cfg['ctx'].split('.')[0]})")
            break
        except Exception as e:
            print(f"  Config {cfg['ctx'][:30]}: {type(e).__name__}: {str(e)[:100]}")
            if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
                stage.RemovePrim(GRAPH_PATH)

# ─── 9. DONE ─────────────────────────────────────────────────────────────────
print()
print("═══════════════════════════════════════════════════════════════")
print("  OmniGraph ready — press ▶ Play in Isaac Sim GUI")
print("═══════════════════════════════════════════════════════════════")
print(f"  Robot  : {ROBOT_PRIM}")
print(f"  Topics : /{CMDVEL_TOPIC}(in)  /{ODOM_TOPIC} /{SCAN_TOPIC}(out)")
print()
print("  On RPi : bash scripts/sim.sh --nav --rviz")
print("═══════════════════════════════════════════════════════════════")

# ─── 10. SIM LOOP ────────────────────────────────────────────────────────────
world.reset()
world.play()
print("\nSimulation running — close window or Ctrl+C to quit.\n")

try:
    while sim_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    pass
finally:
    world.stop()
    sim_app.close()
