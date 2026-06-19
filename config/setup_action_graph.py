# ecza-robotu — Isaac Sim Script Editor setup
# Window → Script Editor → paste → Ctrl+Enter
#
# Prerequisites:
#   1. Isaac Sim launched from terminal:
#        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mucahit/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
#        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#        ~/isaacsim/isaac-sim.sh
#   2. Scene open: File → Open → warehouse_slam.usd

import omni.graph.core as og
import omni.usd
from pxr import UsdPhysics, Usd

# ── CONFIG ─────────────────────────────────────────────────────────────────────
ROBOT_PRIM  = "/World/ecza_rover"
GRAPH_PATH  = "/World/ActionGraph"
JOINT_NAMES = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]
WHEEL_RADIUS   = 0.04
WHEEL_BASE     = 0.38
WHEEL_WIDTH    = 0.26
MECANUM_ANGLES = [45.0, -45.0, -45.0, 45.0]
MAX_SPEED      = 1.5
JOINT_DAMPING  = 1e5
HOLO_TYPE  = "isaacsim.robot.wheeled_robots.HolonomicController"
ARTIC_TYPE = "isaacsim.core.nodes.IsaacArticulationController"
# ──────────────────────────────────────────────────────────────────────────────

stage = omni.usd.get_context().get_stage()
keys  = og.Controller.Keys

# ── 1. WHEEL JOINTS ───────────────────────────────────────────────────────────
print("\n── Wheel joints ──")
robot = stage.GetPrimAtPath(ROBOT_PRIM)
if robot.IsValid():
    name_map = {p.GetName(): p for p in Usd.PrimRange(robot)}
    for jname in JOINT_NAMES:
        prim = name_map.get(jname)
        if prim:
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
            d.CreateTypeAttr("velocity")
            d.CreateStiffnessAttr(0.0)
            d.CreateDampingAttr(JOINT_DAMPING)
            print(f"  ✓ {prim.GetPath()}")
        else:
            print(f"  MISS: {jname}")

# ── 2. GRAPH ──────────────────────────────────────────────────────────────────
print("\n── ActionGraph ──")
graph = og.get_graph_by_path(GRAPH_PATH)
if not graph or not graph.is_valid():
    print(f"ERROR: no graph at {GRAPH_PATH}")
    raise SystemExit(1)

existing = {}
for node in graph.get_nodes():
    t = node.get_type_name()
    p = node.get_prim_path()
    existing[t] = p
    print(f"  {p.split('/')[-1]}  [{t}]")

def find_path(*frags):
    for t, p in existing.items():
        if any(f.lower() in t.lower() for f in frags):
            return p
    return None

tick_path  = find_path("OnPlaybackTick", "playback_tick")
twist_path = find_path("SubscribeTwist", "subscribe_twist")
print(f"\n  tick  → {tick_path}")
print(f"  twist → {twist_path}")

# ── 3. REMOVE LEFTOVER NODES FROM PREVIOUS RUNS ───────────────────────────────
for nname in ["HolonomicController", "ArticulationController"]:
    pp = f"{GRAPH_PATH}/{nname}"
    if stage.GetPrimAtPath(pp).IsValid():
        stage.RemovePrim(pp)
        print(f"  removed leftover: {pp}")

# ── 4. CREATE NODES ───────────────────────────────────────────────────────────
print(f"\n── Create nodes ──")
try:
    og.Controller.edit(graph, {
        keys.CREATE_NODES: [
            ("HolonomicController",    HOLO_TYPE),
            ("ArticulationController", ARTIC_TYPE),
        ],
    })
    print(f"  ✓ {HOLO_TYPE}")
    print(f"  ✓ {ARTIC_TYPE}")
except Exception as e:
    print(f"  ERROR: {e}")
    raise SystemExit(1)

# ── 5. SHOW PORTS ─────────────────────────────────────────────────────────────
print("\n── HolonomicController inputs ──")
try:
    n = graph.get_node(f"{GRAPH_PATH}/HolonomicController")
    if n and n.is_valid():
        for a in n.get_attributes():
            if "INPUT" in str(a.get_port_type()).upper():
                print(f"  inputs:{a.get_name()}  [{a.get_resolved_type()}]")
except Exception as e:
    print(f"  (skipped: {e})")

print("\n── ROS2SubscribeTwist outputs ──")
try:
    n = graph.get_node(twist_path)
    if n and n.is_valid():
        for a in n.get_attributes():
            if "OUTPUT" in str(a.get_port_type()).upper():
                print(f"  outputs:{a.get_name()}  [{a.get_resolved_type()}]")
except Exception as e:
    print(f"  (skipped: {e})")

# ── 6. CONNECT ────────────────────────────────────────────────────────────────
print("\n── Connect ──")
connects = []
if tick_path:
    connects.append((f"{tick_path}.outputs:tick",              "HolonomicController.inputs:execIn"))
if twist_path:
    connects.append((f"{twist_path}.outputs:linearVelocity",   "HolonomicController.inputs:linearVelocity"))
    connects.append((f"{twist_path}.outputs:angularVelocity",  "HolonomicController.inputs:angularVelocity"))
connects += [
    ("HolonomicController.outputs:execOut",              "ArticulationController.inputs:execIn"),
    ("HolonomicController.outputs:jointVelocityCommand", "ArticulationController.inputs:velocityCommand"),
]

ok = True
for src, dst in connects:
    try:
        og.Controller.edit(graph, {keys.CONNECT: [(src, dst)]})
        print(f"  ✓ {src.split('/')[-1]} → {dst.split('.')[-1]}")
    except Exception as e:
        print(f"  ✗ {src.split('/')[-1]} → {dst.split('.')[-1]}")
        print(f"    {e}")
        ok = False

# ── 7. VALUES ─────────────────────────────────────────────────────────────────
print("\n── Values ──")
hb, hw = WHEEL_BASE / 2, WHEEL_WIDTH / 2
vals = [
    ("HolonomicController.inputs:wheelRadius",       [WHEEL_RADIUS] * 4),
    ("HolonomicController.inputs:wheelPositions",    [[hb,hw,0],[hb,-hw,0],[-hb,hw,0],[-hb,-hw,0]]),
    ("HolonomicController.inputs:wheelOrientations", [[0,0,0,1]] * 4),
    ("HolonomicController.inputs:mecanum",           True),
    ("HolonomicController.inputs:mecanumAngles",     MECANUM_ANGLES),
    ("HolonomicController.inputs:maxLinearSpeed",    MAX_SPEED),
    ("ArticulationController.inputs:robotPath",      ROBOT_PRIM),
    ("ArticulationController.inputs:usePath",        True),
    ("ArticulationController.inputs:jointNames",     JOINT_NAMES),
]
for k, v in vals:
    try:
        og.Controller.edit(graph, {keys.SET_VALUES: [(k, v)]})
        print(f"  ✓ {k.split(':')[-1]}")
    except Exception as e:
        print(f"  ✗ {k.split(':')[-1]}: {e}")

print()
if ok:
    print("✓ Done — press ▶ Play")
else:
    print("⚠ Some connections failed — paste the output above and I will fix it")
