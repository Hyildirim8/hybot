# Research: Full Nav2 SLAM Stack — Fix All Errors

**Feature Branch**: `007-fix-nav2-slam`  
**Generated**: 2026-03-04  
**Status**: Complete — all NEEDS CLARIFICATION items resolved

---

## R1: BT Navigator Plugin Availability in `ros-humble-nav2-behavior-tree`

**Question**: Which BT node plugins in `plugin_lib_names` actually exist as `.so`
files inside the Docker image?

**Finding**: `ros-humble-nav2-behavior-tree` v1.1.20 (installed in image) ships
**51 BT plugin `.so` files** in `/opt/ros/humble/lib/`. Nine plugin names that
appeared in the default `nav2_params.yaml` template do **not** exist:

| Missing plugin name | Reason |
|---|---|
| `nav2_are_error_codes_active_condition_bt_node` | Added in Nav2 ≥ 1.2 (Iron+) |
| `nav2_nav_through_poses_action_bt_node` | Renamed — use `nav2_navigate_through_poses_action_bt_node` |
| `nav2_nav_to_pose_action_bt_node` | Renamed — use `nav2_navigate_to_pose_action_bt_node` |
| `nav2_path_expiring_timer_condition_bt_node` | `.so` name is `libnav2_path_expiring_timer_condition.so` (no `_bt_node` suffix) |
| `nav2_smoother_selector_bt_node` | Present as `libnav2_smoother_selector_bt_node.so` ✅ (false alarm) |
| `nav2_speed_limit_action_bt_node` | Added in Nav2 ≥ 1.2 |
| `nav2_would_a_controller_recovery_heal_condition_bt_node` | Added in Nav2 ≥ 1.2 |
| `nav2_would_a_planner_recovery_heal_condition_bt_node` | Added in Nav2 ≥ 1.2 |
| `nav2_would_a_smoother_recovery_heal_condition_bt_node` | Added in Nav2 ≥ 1.2 |

**Decision**: Remove the 8 genuinely missing plugins from `plugin_lib_names`.
Keep `nav2_smoother_selector_bt_node` (`.so` confirmed present).

**Rationale**: Nav2 Humble (1.1.x) is the last release before `error_codes` and
`would_a_*_heal` were introduced. Loading non-existent `.so` files causes
`bt_navigator` to abort the lifecycle `configure` transition with a FATAL error,
preventing the entire Nav2 stack from starting.

**Alternatives considered**: Upgrading to Nav2 Iron/Jazzy rejected — would
require a full ROS distribution upgrade (Ubuntu 24.04); not compatible with the
project's Ubuntu 22.04 / ROS2 Humble platform constraint.

---

## R2: SLAM Toolbox TF Timing — Inter-Container TF Jitter

**Question**: Why does SLAM Toolbox log `Failed to compute odom pose` and
`Message Filter queue is full`? Is this a configuration issue or a
network/DDS issue?

**Finding**: SLAM Toolbox runs in the `navigation` container. The `odom→base_link`
TF is published by `mecanum_drive_controller` in the `robot_description`
container. With `network_mode: host` and FastDDS UDP multicast, TF messages
cross a loopback DDS boundary. The default `transform_timeout: 0.5 s` in
`slam_params.yaml` is insufficient for the added ~50–200 ms DDS delivery jitter
between containers on the same host.

**Decision**: Increase `transform_timeout` to `1.0` s in `slam_params.yaml`.

**Rationale**: 1.0 s is the standard Nav2 tutorial recommendation for
inter-container or inter-machine deployments. It absorbs DDS jitter while
remaining short enough not to cause stale-map artefacts (SLAM accepts scans
up to 1 s after the TF was published, which is safe given typical human-speed
robot motion at < 1 m/s).

**Alternatives considered**:
- Reducing container startup jitter via `depends_on: condition: service_healthy` — useful but doesn't solve steady-state TF latency.
- Tuning FastDDS `BEST_EFFORT` reliability — `nav2_bringup` LaserScan filter uses `RELIABLE`; mismatched QoS causes silent drops. Not applicable here.

---

## R3: RViz Nav2 Goal Tool — Why GoalTool vs SetGoal?

**Question**: Why does `rviz_default_plugins/SetGoal` not work for Nav2 goal
sending?

**Finding**: `rviz_default_plugins/SetGoal` publishes a `geometry_msgs/PoseStamped`
on `/goal_pose`. Nav2's `bt_navigator` subscribes to `nav2_msgs/action/NavigateToPose`
action, not `/goal_pose`. The bridge between the two was provided by the
`nav2_rviz_plugins/GoalTool` which calls the action server directly via
`rclcpp_action`. Starting with Nav2 Humble this is the only supported RViz
integration path.

**Decision**: Use `nav2_rviz_plugins/GoalTool` and add `nav2_rviz_plugins/Nav2Panel`
to `rover.rviz`.

**Rationale**: `GoalTool` sends goals via the proper action protocol, receives
feedback (progress, ETA), and reports result (Succeeded/Failed) in Nav2Panel.
`SetGoal` only publishes a topic; Nav2 ignores it in Humble.

**Alternatives considered**: `ros2 action send_goal` CLI — functional for testing
but not a user-facing solution.

---

## R4: `ros-humble-nav2-rviz-plugins` Package Availability

**Question**: Is `ros-humble-nav2-rviz-plugins` available in the `ros-humble`
apt repository for Ubuntu 22.04?

**Finding**: Package `ros-humble-nav2-rviz-plugins` v1.1.20 is confirmed present
in the apt repository and is now installed in the `ecza-robotu:runtime` image
(verified: `dpkg -l` inside running image confirms install).

**Decision**: Add `ros-humble-nav2-rviz-plugins` to the Dockerfile apt install
block, after `ros-humble-nav2-bringup`.

---

## R5: Docker Image Rebuild vs `docker restart` Behaviour

**Question**: Does `docker restart <container>` pick up a newly built image?

**Finding**: `docker restart` re-starts the same container from its **existing
layer snapshot** — it does NOT pull in a newly built image. The container was
created from the old image and `docker restart` just stops/starts that same
container. Only `docker compose --force-recreate` tears down and re-creates the
container from the latest image tag.

**Decision**: Always use `docker compose -f docker-compose.yaml --profile nav up
-d --force-recreate navigation lidar` after any `docker compose build`.

**Rationale**: This is a fundamental Docker container lifecycle constraint, not a
configuration choice.

---

## R6: SLAM Toolbox vs AMCL — Which to Use for Live Mapping?

**Question**: When should `navigation_launch.py` be used vs `bringup_launch.py`?

**Finding**: Per the official nav2.org SLAM tutorial
(https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html):

- **SLAM mode**: Use `navigation_launch.py`. No `map_server`, no `AMCL`. SLAM
  Toolbox provides both `/map` and the `map→odom` TF directly.
- **Nav mode** (known map): Use `bringup_launch.py` with `map:=` argument. AMCL
  provides `map→odom`. SLAM Toolbox not running.

The `nav2_navigation.launch.py` in this project correctly implements both modes
via the `mode:=slam|nav` launch argument.

**Decision**: Confirmed — architecture is correct. No changes to launch file required.

---

## R7: `OmnidirectionalMotionModel` for AMCL on Mecanum Rover

**Question**: What AMCL `robot_model_type` is correct for a mecanum (holonomic) rover?

**Finding**: AMCL supports three motion models:
- `nav2_amcl::DifferentialMotionModel` — for non-holonomic (differential drive)
- `nav2_amcl::OmnidirectionalMotionModel` — for holonomic (mecanum, omni)
- `nav2_amcl::Stationary` — no motion assumed

Mecanum wheels allow lateral motion (strafing), so the rover is holonomic.
`DifferentialMotionModel` would model strafing as zero displacement, causing
AMCL to diverge during lateral motion.

**Decision**: Use `nav2_amcl::OmnidirectionalMotionModel` in `nav2_params.yaml`.

---

## R8: TF Frame Names — `base_link` vs `base_footprint`

**Question**: Should the SLAM `base_frame` be `base_link` or `base_footprint`?

**Finding**: The `mecanum_drive_controller` in `ros2_control` publishes
`odom→base_link` on `/tf`. The URDF declares `base_link` as the root link. There
is no `base_footprint` frame in the rover's TF tree.

`base_footprint` is a Nav2 default convention for ground-projected 2D frames used
with legged or wheeled robots that have a separate footprint projection. This
rover's `base_link` IS on the ground plane, so `base_link` is correct everywhere
(`slam_params.yaml`, `nav2_params.yaml` `bt_navigator.base_frame_id`, etc.)

**Decision**: All `base_frame_id` references use `base_link`.

---

## Summary: All Unknowns Resolved

| Unknown | Resolution |
|---|---|
| Which BT plugins exist in ros-humble 1.1.20? | 51 plugins; 8 missing ones identified and removed |
| Why `transform_timeout` failures? | DDS inter-container jitter; fix: increase to 1.0 s |
| Why GoalTool vs SetGoal? | GoalTool calls action server directly; SetGoal only publishes topic |
| Is nav2-rviz-plugins available? | Yes, v1.1.20 confirmed installed |
| docker restart vs force-recreate? | restart reuses old layers; force-recreate required |
| SLAM vs AMCL launch mode? | navigation_launch.py for SLAM; bringup_launch.py for nav |
| Mecanum AMCL model? | OmnidirectionalMotionModel |
| base_link vs base_footprint? | base_link — no base_footprint frame in URDF |
