"""
ecza-robotu Isaac Sim 4.5 OmniGraph setup.

STEP 1 — run the DIAGNOSTIC block first (scroll down, un-comment it).
         It prints which extensions are loaded and which node types are available.

STEP 2 — fix ROBOT_PRIM / LIDAR_PRIM if needed, then run the BUILD block.

Run in Isaac Sim: Window → Script Editor → paste whole file → Ctrl+Enter.
"""

import omni.kit.app
import omni.graph.core as og
import omni.usd

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG — adjust to match your Stage panel
# ═══════════════════════════════════════════════════════════════════════════════
ROBOT_PRIM  = "/World/ecza_robot"          # right-click in Stage → Copy Prim Path
LIDAR_PRIM  = "/World/ecza_robot/base_link/Lidar"  # find your lidar prim in Stage
JOINT_NAMES = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]

WHEEL_RADIUS   = 0.04
WHEEL_BASE     = 0.38
WHEEL_WIDTH    = 0.26
MECANUM_ANGLES = [45.0, -45.0, -45.0, 45.0]  # FL FR RL RR roller angles (deg)

SCAN_TOPIC    = "scan_sim"
ODOM_TOPIC    = "odom"
CMDVEL_TOPIC  = "cmd_vel"
SCAN_FRAME    = "laser_frame"
ODOM_FRAME    = "world"
CHASSIS_FRAME = "base_footprint"
GRAPH_PATH    = "/World/ActionGraph"
# ═══════════════════════════════════════════════════════════════════════════════


# ───────────────────────────────────────────────────────────────────────────────
# DIAGNOSTIC — run this first to find correct node type names on YOUR Isaac Sim
# Un-comment the block below, run, check the output, then re-comment it.
# ───────────────────────────────────────────────────────────────────────────────
# ext_manager = omni.kit.app.get_app().get_extension_manager()
# print("\n── Extension status ──────────────────────────────────────────────────")
# for eid in ["omni.isaac.ros2_bridge", "isaacsim.ros2.bridge",
#             "omni.isaac.wheeled_robots", "isaacsim.robot.wheeled_robots",
#             "omni.isaac.core_nodes",   "isaacsim.core.nodes",
#             "omni.isaac.range_sensor", "isaacsim.sensors.physx"]:
#     state = "LOADED" if ext_manager.is_extension_enabled(eid) else "not loaded"
#     print(f"  {eid}: {state}")
# print("\n── Available OmniGraph node types (ROS2 / drive / lidar / odom) ────")
# registry = og.get_node_type_registry()
# for t in sorted(registry.get_all_node_types()):
#     kw = ["ros2", "holonomic", "articulation", "lidar", "odometry", "wheeled"]
#     if any(k in t.lower() for k in kw):
#         print(f"  {t}")
# raise SystemExit("Diagnostic done — check output above, then re-comment this block.")


# ───────────────────────────────────────────────────────────────────────────────
# ENABLE EXTENSIONS — loads the ROS2 bridge and robot control extensions if
# they are present but not yet enabled.  This may fail if AMENT_PREFIX_PATH
# is not set (see Prerequisites below).
# ───────────────────────────────────────────────────────────────────────────────
# Prerequisites: on mucahit@10.42.101.217, before launching Isaac Sim:
#   export AMENT_PREFIX_PATH=/home/mucahit/isaacsim/exts/isaacsim.ros2.bridge/humble
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mucahit/isaacsim/exts/isaacsim.ros2.bridge/humble/lib

ext_manager = omni.kit.app.get_app().get_extension_manager()

def try_enable(ext_id):
    if not ext_manager.is_extension_enabled(ext_id):
        try:
            ext_manager.set_extension_enabled_immediate(ext_id, True)
            print(f"  enabled: {ext_id}")
        except Exception as e:
            print(f"  FAILED to enable {ext_id}: {e}")
    else:
        print(f"  already loaded: {ext_id}")

print("Enabling extensions...")
# Try both old (4.1) and new (4.5) namespaces — only one will succeed
try_enable("omni.isaac.ros2_bridge")
try_enable("isaacsim.ros2.bridge")
try_enable("omni.isaac.wheeled_robots")
try_enable("isaacsim.robot.wheeled_robots")
try_enable("omni.isaac.core_nodes")
try_enable("isaacsim.core.nodes")
try_enable("omni.isaac.range_sensor")


# ───────────────────────────────────────────────────────────────────────────────
# NODE TYPE RESOLUTION — picks the correct name for this Isaac Sim version
# ───────────────────────────────────────────────────────────────────────────────
registry = og.get_node_type_registry()
all_types = set(registry.get_all_node_types())

def pick(candidates):
    """Return the first candidate that exists in the registry."""
    for c in candidates:
        if c in all_types:
            return c
    raise RuntimeError(
        f"None of {candidates} found in OmniGraph registry.\n"
        "Run the DIAGNOSTIC block above to see what IS available."
    )

T_TICK       = pick(["omni.graph.action.OnPlaybackTick"])
T_CONTEXT    = pick(["omni.isaac.ros2_bridge.ROS2Context",
                      "isaacsim.ros2.bridge.ROS2Context"])
T_CLOCK      = pick(["omni.isaac.core_nodes.IsaacReadSystemTime",
                      "isaacsim.core.nodes.IsaacReadSystemTime"])
T_LIDAR      = pick(["omni.isaac.range_sensor.IsaacReadLidarBeams",
                      "isaacsim.sensors.physx.IsaacReadLidarBeams",
                      "isaacsim.sensors.lidar.IsaacReadLidarBeams"])
T_SCAN       = pick(["omni.isaac.ros2_bridge.ROS2PublishLaserScan",
                      "isaacsim.ros2.bridge.ROS2PublishLaserScan"])
T_ODOM_COMP  = pick(["omni.isaac.core_nodes.IsaacComputeOdometry",
                      "isaacsim.core.nodes.IsaacComputeOdometry"])
T_ODOM_PUB   = pick(["omni.isaac.ros2_bridge.ROS2PublishOdometry",
                      "isaacsim.ros2.bridge.ROS2PublishOdometry"])
T_TF         = pick(["omni.isaac.ros2_bridge.ROS2PublishTransformTree",
                      "isaacsim.ros2.bridge.ROS2PublishTransformTree"])
T_TWIST      = pick(["omni.isaac.ros2_bridge.ROS2SubscribeTwist",
                      "isaacsim.ros2.bridge.ROS2SubscribeTwist"])
T_HOLONOMIC  = pick(["omni.isaac.wheeled_robots.HolonomicController",
                      "isaacsim.robot.wheeled_robots.HolonomicController"])
T_ARTICULATE = pick(["omni.isaac.core_nodes.IsaacArticulationController",
                      "isaacsim.core.nodes.IsaacArticulationController"])

print("\nUsing node types:")
for label, t in [("Tick", T_TICK), ("ROS2Context", T_CONTEXT),
                  ("Holonomic", T_HOLONOMIC), ("Articulation", T_ARTICULATE),
                  ("SubscribeTwist", T_TWIST)]:
    print(f"  {label:15s}: {t}")


# ───────────────────────────────────────────────────────────────────────────────
# BUILD GRAPH
# ───────────────────────────────────────────────────────────────────────────────
hb = WHEEL_BASE  / 2.0
hw = WHEEL_WIDTH / 2.0
wheel_positions   = [[ hb, hw, 0.0], [ hb, -hw, 0.0], [-hb, hw, 0.0], [-hb, -hw, 0.0]]
wheel_orientations = [[0.0, 0.0, 0.0, 1.0]] * 4

# Remove old graph for a clean rebuild
stage = omni.usd.get_context().get_stage()
old   = stage.GetPrimAtPath(GRAPH_PATH)
if old.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print(f"\nRemoved old graph at {GRAPH_PATH}")

keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick",         T_TICK),
            ("ROS2Context",            T_CONTEXT),
            ("IsaacReadSystemTime",    T_CLOCK),
            ("IsaacReadLidar",         T_LIDAR),
            ("ROS2PublishLaserScan",   T_SCAN),
            ("IsaacComputeOdometry",   T_ODOM_COMP),
            ("ROS2PublishOdometry",    T_ODOM_PUB),
            ("ROS2PublishTF",          T_TF),
            ("ROS2SubscribeTwist",     T_TWIST),
            ("HolonomicController",    T_HOLONOMIC),
            ("ArticulationController", T_ARTICULATE),
        ],
        keys.CONNECT: [
            # Tick fans out to all exec heads
            ("OnPlaybackTick.outputs:tick",         "IsaacReadLidar.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",         "IsaacComputeOdometry.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",         "ROS2SubscribeTwist.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",         "HolonomicController.inputs:execIn"),
            # Context → all publishers/subscribers
            ("ROS2Context.outputs:context",         "ROS2PublishLaserScan.inputs:context"),
            ("ROS2Context.outputs:context",         "ROS2PublishOdometry.inputs:context"),
            ("ROS2Context.outputs:context",         "ROS2PublishTF.inputs:context"),
            ("ROS2Context.outputs:context",         "ROS2SubscribeTwist.inputs:context"),
            # Wall-clock timestamps
            ("IsaacReadSystemTime.outputs:systemTime", "ROS2PublishLaserScan.inputs:timeStamp"),
            ("IsaacReadSystemTime.outputs:systemTime", "ROS2PublishOdometry.inputs:timeStamp"),
            ("IsaacReadSystemTime.outputs:systemTime", "ROS2PublishTF.inputs:timeStamp"),
            # Lidar chain
            ("IsaacReadLidar.outputs:execOut",               "ROS2PublishLaserScan.inputs:execIn"),
            ("IsaacReadLidar.outputs:azimuthRange",          "ROS2PublishLaserScan.inputs:azimuthRange"),
            ("IsaacReadLidar.outputs:depthRange",            "ROS2PublishLaserScan.inputs:depthRange"),
            ("IsaacReadLidar.outputs:horizontalFov",         "ROS2PublishLaserScan.inputs:horizontalFov"),
            ("IsaacReadLidar.outputs:horizontalResolution",  "ROS2PublishLaserScan.inputs:horizontalResolution"),
            ("IsaacReadLidar.outputs:intensitiesData",       "ROS2PublishLaserScan.inputs:intensitiesData"),
            ("IsaacReadLidar.outputs:linearDepthData",       "ROS2PublishLaserScan.inputs:linearDepthData"),
            ("IsaacReadLidar.outputs:numCols",               "ROS2PublishLaserScan.inputs:numCols"),
            ("IsaacReadLidar.outputs:numRows",               "ROS2PublishLaserScan.inputs:numRows"),
            ("IsaacReadLidar.outputs:rotationRate",          "ROS2PublishLaserScan.inputs:rotationRate"),
            ("ROS2PublishLaserScan.outputs:execOut",         "ROS2PublishTF.inputs:execIn"),
            # Odometry chain
            ("IsaacComputeOdometry.outputs:execOut",         "ROS2PublishOdometry.inputs:execIn"),
            ("IsaacComputeOdometry.outputs:position",        "ROS2PublishOdometry.inputs:position"),
            ("IsaacComputeOdometry.outputs:orientation",     "ROS2PublishOdometry.inputs:orientation"),
            ("IsaacComputeOdometry.outputs:linearVelocity",  "ROS2PublishOdometry.inputs:linearVelocity"),
            ("IsaacComputeOdometry.outputs:angularVelocity", "ROS2PublishOdometry.inputs:angularVelocity"),
            # cmd_vel → mecanum drive → joints  ← THE MISSING CHAIN
            ("ROS2SubscribeTwist.outputs:linearVelocity",       "HolonomicController.inputs:linearVelocity"),
            ("ROS2SubscribeTwist.outputs:angularVelocity",      "HolonomicController.inputs:angularVelocity"),
            ("HolonomicController.outputs:execOut",             "ArticulationController.inputs:execIn"),
            ("HolonomicController.outputs:jointVelocityCommand","ArticulationController.inputs:velocityCommand"),
        ],
        keys.SET_VALUES: [
            ("IsaacReadLidar.inputs:lidarPrim",              [LIDAR_PRIM]),
            ("ROS2PublishLaserScan.inputs:topicName",        SCAN_TOPIC),
            ("ROS2PublishLaserScan.inputs:frameId",          SCAN_FRAME),
            ("IsaacComputeOdometry.inputs:chassisPrim",      ROBOT_PRIM),
            ("ROS2PublishOdometry.inputs:topicName",         ODOM_TOPIC),
            ("ROS2PublishOdometry.inputs:odomFrameId",       ODOM_FRAME),
            ("ROS2PublishOdometry.inputs:chassisFrameId",    CHASSIS_FRAME),
            ("ROS2PublishTF.inputs:targetPrims",             [ROBOT_PRIM]),
            ("ROS2SubscribeTwist.inputs:topicName",          CMDVEL_TOPIC),
            ("HolonomicController.inputs:wheelRadius",       [WHEEL_RADIUS] * 4),
            ("HolonomicController.inputs:wheelPositions",    wheel_positions),
            ("HolonomicController.inputs:wheelOrientations", wheel_orientations),
            ("HolonomicController.inputs:mecanum",           True),
            ("HolonomicController.inputs:mecanumAngles",     MECANUM_ANGLES),
            ("HolonomicController.inputs:maxLinearSpeed",    1.5),
            ("ArticulationController.inputs:robotPath",      ROBOT_PRIM),
            ("ArticulationController.inputs:usePath",        True),
            ("ArticulationController.inputs:jointNames",     JOINT_NAMES),
        ],
    }
)

print("\n" + "=" * 60)
print("  OmniGraph rebuilt — press ▶ Play to start simulation")
print("=" * 60)
print(f"  Robot : {ROBOT_PRIM}")
print(f"  Lidar : {LIDAR_PRIM}")
print(f"  Topics: /{SCAN_TOPIC}  /{ODOM_TOPIC}  /{CMDVEL_TOPIC}")
print()
print("  If robot still doesn't move after Play:")
print("  • Stage panel → Copy Prim Path → fix ROBOT_PRIM at top")
print("  • Physics > each wheel joint > Drive: Stiffness=0, Damping=1e5")
print("=" * 60)
