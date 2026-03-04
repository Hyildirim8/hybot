# Feature Specification: Full Nav2 SLAM Stack — Fix All Errors

**Feature Branch**: `007-fix-nav2-slam`  
**Created**: 2026-03-04  
**Status**: Draft  
**Reference**: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 — SLAM maps the environment during navigation (Priority: P1)

An operator starts the rover, drives it (or sends Nav2 goals) around a room, and
watches a live occupancy-grid map build in RViz in real time. The map persists
correctly and the robot does not get lost.

**Why this priority**: Without a working map→odom transform and a running Nav2
stack, no other navigation feature is possible. This is the foundational
capability.

**Independent Test**: Power on rover, launch stack (`--nav` flag), open RViz.
Drive the robot with the joystick. Map tiles should appear and expand as the
robot explores. Robot pose (arrow) should track correctly on the map.

**Acceptance Scenarios**:

1. **Given** the full stack is running in SLAM mode, **When** the robot moves
   forward 1 m, **Then** the occupancy-grid map in RViz shows the newly scanned
   area within 5 seconds and the robot pose is within ±0.1 m of actual position.

2. **Given** SLAM mode is active, **When** the lidar scans a wall, **Then** the
   wall appears as occupied cells in the `/map` topic and in the RViz Map
   display.

3. **Given** the navigation container starts, **When** the lifecycle manager
   brings up all Nav2 nodes, **Then** no ERROR log lines appear for
   `bt_navigator`, `controller_server`, `planner_server`, or `behavior_server`.

---

### User Story 2 — Operator sends autonomous Nav2 goals from RViz (Priority: P2)

After some exploration, the operator clicks the Nav2 Goal tool in RViz, places a
goal pose on the map, and the rover navigates to that pose autonomously while
continuously updating the map.

**Why this priority**: This is the primary value of the autonomous mode feature.
Without it, the Start-button toggle has no observable effect.

**Independent Test**: With SLAM mode running and a partial map built, use the
RViz Nav2 Panel "Navigate to Pose" button and place a goal. Robot should start
moving.

**Acceptance Scenarios**:

1. **Given** a partial map has been built and Nav2 is active, **When** the
   operator places a Nav2 Goal in RViz, **Then** the global plan appears as a
   blue line and the robot begins moving toward the goal within 3 seconds.

2. **Given** the robot is navigating to a goal, **When** the robot reaches the
   goal, **Then** the Nav2 Panel reports success and the robot stops.

3. **Given** the Nav2 Goal tool is selected, **When** the operator clicks an
   unknown (grey) area of the map, **Then** Nav2 either navigates to the nearest
   known free cell or reports that no plan could be found.

---

### User Story 3 — Map saved and reloaded for repeated navigation (Priority: P3)

After fully mapping a room, the operator saves the map and on a subsequent
launch uses it for AMCL-based localisation without re-mapping.

**Why this priority**: Valuable for production use but not required for the
immediate bug-fix scope. Can be deferred until SLAM works reliably.

**Independent Test**: Run `ros2 run nav2_map_server map_saver_cli -f /maps/room`,
restart stack with `NAV_MODE=nav MAP=/maps/room.yaml`, verify robot localises.

**Acceptance Scenarios**:

1. **Given** a map has been saved to `/maps/room.yaml`, **When** the stack
   restarts in nav mode, **Then** the saved map appears in RViz and AMCL
   publishes a valid `map→odom` transform within 10 seconds.

2. **Given** nav mode is running, **When** the operator drives the robot 0.5 m,
   **Then** AMCL updates the pose estimate without the `Failed to compute odom
   pose` warning.

---

### Edge Cases

- What happens when the ESP32 is not connected on startup? SLAM should still
  start; odometry will be zero but no crash.
- What happens if `/dev/ttyLIDAR` is missing? The lidar container should restart
  with a clear error; Nav2 should degrade gracefully (costmaps empty).
- What if the robot starts in a featureless area? SLAM Toolbox may report poor
  scan matching; robot should not hallucinate obstacles.
- What if `docker restart` is used instead of `docker compose --force-recreate`?
  Old image layers may be reused; the correct restart procedure must be
  documented.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The Nav2 lifecycle manager MUST bring up all nodes
  (`controller_server`, `smoother_server`, `planner_server`, `behavior_server`,
  `bt_navigator`, `waypoint_follower`, `velocity_smoother`) without ERROR log
  lines on a fresh start.

- **FR-002**: The `bt_navigator` MUST load only BT node plugins that exist in
  the installed `ros-humble-nav2-behavior-tree` apt package. Plugins from newer
  Nav2 releases MUST be removed from `plugin_lib_names`.

- **FR-003**: SLAM Toolbox MUST NOT log `Failed to compute odom pose`
  continuously. The `odom→base_link` TF MUST be available in the SLAM
  container's TF buffer within 2 seconds of startup.

- **FR-004**: The `/odom` topic MUST have exactly 1 publisher (remapped from
  `/controller_manager/odometry`) so that SLAM Toolbox and Nav2 receive
  odometry.

- **FR-005**: In SLAM mode, Nav2 MUST be launched via `navigation_launch.py`
  (no `map_server`, no AMCL). SLAM Toolbox provides `map→odom` TF and `/map`.

- **FR-006**: The `map→odom` TF published by SLAM Toolbox MUST NOT have a
  future timestamp. Offset MUST be within ±0.1 s of wall-clock time across all
  containers.

- **FR-007**: The SLAM scan message filter MUST NOT log `queue is full`
  continuously. Scans MUST match TF lookups within `transform_timeout`.

- **FR-008**: RViz MUST display: live `/scan`, `/map`, global costmap, local
  costmap, global plan, local plan, robot footprint, and odometry trail — all
  in the `map` fixed frame.

- **FR-009**: The Nav2 Goal tool in RViz MUST use `nav2_rviz_plugins/GoalTool`.
  The Nav2 Panel MUST be present and functional.

- **FR-010**: `nav2_rviz_plugins` MUST be installed in the Docker image so the
  Nav2 Panel and GoalTool plugins load without errors.

- **FR-011**: After rebuilding the Docker image, the `nav` profile containers
  MUST be restarted with the new image using `docker compose --force-recreate`.

### Key Entities

- **SLAM Toolbox node**: Subscribes to `/scan` and `/tf`, publishes `/map` and
  `map→odom` TF. In SLAM mode this replaces AMCL entirely.
- **Nav2 lifecycle group**: All Nav2 servers managed by
  `lifecycle_manager_navigation`.
- **BT plugin list**: `plugin_lib_names` in `nav2_params.yaml` must exactly
  match `.so` files present in `/opt/ros/humble/lib/`.
- **TF chain**: `map → odom → base_link → laser_frame` — all links must be
  present and current-timestamped in every container.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Nav2 lifecycle brings up all nodes with zero ERROR lines within
  15 seconds of container start.

- **SC-002**: SLAM Toolbox stops logging `Failed to compute odom pose` within
  5 seconds of the `robot_description` container being healthy.

- **SC-003**: The `/map` topic receives its first message within 10 seconds of
  the lidar publishing `/scan`.

- **SC-004**: A Nav2 goal placed in RViz results in visible robot motion within
  3 seconds on a map with at least one known obstacle.

- **SC-005**: The full TF chain `map→odom→base_link→laser_frame` is intact and
  current (delay < 0.5 s) in all containers simultaneously.

- **SC-006**: Zero `Could not load library` errors in navigation container logs
  on a clean start.

- **SC-007**: RViz displays map, costmaps, laser scan, path, and robot model
  simultaneously without `Fixed Frame does not exist` warnings.

---

## Assumptions

- Installed Nav2 is from the `ros-humble` apt repository; plugin availability is
  constrained to that package set.
- `network_mode: host` is used for all containers; DDS multicast works across
  container process boundaries on the same host.
- ESP32 firmware and wheel odometry are working; `controller_manager` publishes
  valid `/odom` at ~50 Hz.
- `docker compose --force-recreate` (not `docker restart`) is required to apply
  a newly built image to running containers.
- The systemd service starts only default-profile services; `nav` profile
  containers must be explicitly force-recreated when the image changes.
