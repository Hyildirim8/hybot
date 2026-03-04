# Implementation Plan: Full Nav2 SLAM Stack — Fix All Errors

**Branch**: `007-fix-nav2-slam` | **Date**: 2026-03-04 | **Spec**: [spec.md](spec.md)  
**Input**: Feature specification from `specs/007-fix-nav2-slam/spec.md`

## Summary

Fix all errors preventing Nav2 + SLAM Toolbox from running on the ecza-robotu
mecanum rover. The stack fails to start due to: (a) 8 BT node plugins missing
from the `ros-humble-nav2-behavior-tree` 1.1.x apt package, (b) SLAM Toolbox
TF timeout too short for inter-container DDS jitter, (c) missing
`ros-humble-nav2-rviz-plugins` in the Docker image preventing the Nav2 Goal
tool from loading in RViz.

All configuration fixes have been applied to source files. The Docker image and
running containers need to be rebuilt and force-recreated to deploy them.

## Technical Context

**Language/Version**: Python 3.10 (ROS2 launch files, scripts), C++17
(firmware — not modified in this feature)  
**Primary Dependencies**:
- `ros-humble-nav2-bringup` v1.1.20 — `navigation_launch.py` and `bringup_launch.py`
- `ros-humble-nav2-behavior-tree` v1.1.20 — 51 BT plugin `.so` files
- `ros-humble-nav2-rviz-plugins` v1.1.20 — `Nav2Panel` + `GoalTool`
- `ros-humble-slam-toolbox` v2.6.10 — `async_slam_toolbox_node`
- `ros-humble-rplidar-ros` — RPLidar A2M12 driver, publishes `/scan`
- `ros2_control` + `mecanum_drive_controller` — publishes `/odom` + `odom→base_link` TF

**Storage**: `/maps` bind-mount volume (host `./maps/`) for saved map files; no database  
**Testing**: Manual hardware test + `ros2 topic`/`tf2_echo` spot checks (see
tasks.md); `pytest` and `launch_testing` not applicable to this config-fix feature  
**Target Platform**: Raspberry Pi 4, Ubuntu 22.04 LTS, Docker Compose,
`network_mode: host`, FastDDS UDP multicast  
**Project Type**: Robotics navigation config fix (no new packages; modifies
`ecza_navigation` config, `ecza_description` launch/rviz, Dockerfile)  
**Performance Goals**: Nav2 lifecycle reaches `active` within 15 s; SLAM
`map→odom` TF latency < 0.5 s; `/map` first message < 10 s after lidar ready  
**Constraints**: Must use `ros-humble` apt packages only (Ubuntu 22.04 / ROS2
Humble — no Iron/Jazzy). `network_mode: host` required for DDS multicast.
Docker image < 4 GB target.  
**Scale/Scope**: Single-robot, single-operator. 3 user stories. Config changes
only — no new ROS2 packages introduced.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-checked after Phase 1 design.*

### Pre-Research Gate (Initial)

| Principle | Status | Assessment |
|---|---|---|
| **I. ROS2-First Architecture** | ✅ PASS | All Nav2/SLAM nodes are standard ROS2 nodes from apt. No inter-process bypass. Each package independently buildable. Launch files provided for all modes (`navigation_launch.py`, `bringup_launch.py`). |
| **II. Mecanum Kinematics Correctness** | ✅ N/A | This feature does not touch kinematics. `wheel_separation_width`, `wheel_base`, wheel velocities are unchanged. `robot_model_type: OmnidirectionalMotionModel` in AMCL is consistent with holonomic kinematics (mecanum). |
| **III. Joystick Input via joy** | ✅ PASS | Teleop node unchanged. US1 testing uses joystick via the existing `joy → teleop_node → /cmd_vel` chain. No new input path introduced. |
| **IV. Hardware Abstraction Layer** | ✅ PASS | Nav2 and SLAM Toolbox only see `/cmd_vel` (output) and `/odom`, `/tf`, `/scan` (inputs). No hardware-specific code in navigation layer. ESP32 firmware untouched. micro-ROS transport unchanged. |
| **V. Observability & Diagnostics** | ⚠️ PARTIAL | Nav2 nodes publish `/diagnostics` unconditionally via `nav2_bringup`. `docker-compose.yaml` rosbag records `/diagnostics`. However, no success criterion explicitly verified this. **Mitigation**: tasks.md includes a step to verify `ros2 topic hz /diagnostics > 0` during active navigation. Sufficient for Humble-era Nav2 where diagnostics publication is unconditional. |
| **VI. Simplicity & Incremental Delivery** | ⚠️ CONDITIONAL PASS | Nav2 + SLAM Toolbox are complex dependencies. Justified — see Complexity Tracking section below. Manual teleoperation was proven stable on `feature/lidar` (50 Hz odometry, TF chain working, RPLidar scanning confirmed). This feature builds on a proven-stable teleop baseline as required by §VI. |

### Post-Design Re-Check (Phase 1)

| Principle | Status | Notes |
|---|---|---|
| **I. ROS2-First** | ✅ PASS | No changes to architecture. `nav2_navigation.launch.py` uses standard `IncludeLaunchDescription` to call `nav2_bringup` launch files. |
| **II. Mecanum Kinematics** | ✅ N/A | Unchanged. |
| **III. Joystick** | ✅ PASS | Unchanged. |
| **IV. HAL** | ✅ PASS | Unchanged. |
| **V. Observability** | ✅ PASS | `/diagnostics` covered via existing Nav2 publishers + explicit verification task in tasks.md. |
| **VI. Simplicity** | ✅ PASS | This feature is config-fix only (no new ROS2 packages). Incremental: fixes blocking errors first (US1 → US2 → US3), each independently deliverable. US3 (AMCL nav mode) explicitly deferred as P3. |

**Gate decision**: ✅ PASS — no constitution violations remain unmitigated.

## Project Structure

### Documentation (this feature)

```text
specs/007-fix-nav2-slam/
├── plan.md          ← this file
├── research.md      ← Phase 0 output (generated)
├── spec.md          ← feature specification
└── tasks.md         ← task list (speckit.tasks output)
```

### Source Code (modified by this feature)

```text
docker/
└── Dockerfile                              ← add ros-humble-nav2-rviz-plugins

src/ecza_navigation/
├── config/
│   ├── nav2_params.yaml                    ← remove 8 missing BT plugins,
│   │                                          fix base_frame_id, robot_model_type
│   └── slam_params.yaml                    ← transform_timeout: 1.0
└── launch/
    └── nav2_navigation.launch.py           ← verified correct, no changes needed

src/ecza_description/
├── launch/
│   └── hardware.launch.py                  ← /odom + /tf remaps (already applied)
└── rviz/
    └── rover.rviz                          ← GoalTool, Nav2Panel, map fixed frame

docker-compose.yaml                         ← navigation service restart docs (T024)
scripts/launch.sh                           ← --nav path force-recreate (T025)
```

**Structure Decision**: Single-project layout. No new packages. All changes are
parameter files, a Dockerfile apt install line, and a RViz config — within the
existing `ecza_navigation` and `ecza_description` packages.

## Complexity Tracking

> Filled because Constitution §VI requires justification for complex dependency
> introduction before Nav2 + SLAM Toolbox can be accepted into the codebase.

| Dependency | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| **Nav2 full stack** (`controller_server`, `planner_server`, `bt_navigator`, `behavior_server`, `velocity_smoother`) | Required for autonomous path planning and execution from RViz Nav2 Goal. Without Nav2 the rover has only manual teleoperation — no autonomous mode despite the hardware supporting it. | A custom ROS2 action server sending direct `/cmd_vel` commands would lack obstacle avoidance, dynamic replanning, and recovery behaviours. This would require ~3000 lines of robotics infrastructure to replicate a fraction of Nav2's capability. |
| **SLAM Toolbox** (`async_slam_toolbox_node`) | Builds a live occupancy-grid map while the robot moves and provides the `map→odom` TF that Nav2 requires for global planning. Without SLAM, Nav2 has no map to plan against. | Static map + AMCL requires the map to be pre-built — a chicken-and-egg problem for first exploration. SLAM Toolbox is the official ROS2 Humble recommendation for simultaneous mapping + navigation per the nav2.org tutorial. |
| **`ros-humble-nav2-rviz-plugins`** | Provides `GoalTool` (sends `NavigateToPose` action goals from RViz) and `Nav2Panel` (shows navigation state). Without it, the operator cannot send goals from the standard RViz interface. | `rviz_default_plugins/SetGoal` publishes `/goal_pose` topic only; Nav2 Humble does not bridge this to the `NavigateToPose` action server. CLI `ros2 action send_goal` is not a user-facing operator interface. |

**Justification rationale**: At the point this feature begins, manual
teleoperation is confirmed stable (odometry at 50 Hz, `odom→base_link` TF at
50 Hz, RPLidar scanning at 10 Hz with valid scan data) as established by the
`feature/lidar` session. Constitution §VI permits introducing navigation
complexity when: (a) the hardware baseline is stable, and (b) the autonomy layer
cannot be further deferred without blocking the primary project value proposition.
Both conditions are met.

---

## Architecture: Nav2 + SLAM Toolbox (per nav2.org tutorial)

```
[robot_description container]
  mecanum_drive_controller  →  /tf (odom → base_link)  +  /odom
  robot_state_publisher     →  /tf_static (base_link → laser_frame, …)

[lidar container]
  rplidar_node              →  /scan

[navigation container]
  async_slam_toolbox_node   ←  /scan, /tf (odom → base_link)
                            →  /map,  /tf (map → odom)

  nav2 via navigation_launch.py (SLAM mode — no map_server, no AMCL)
    controller_server        ←  /odom, local costmap
    planner_server           ←  /map, global costmap
    bt_navigator             →  /cmd_vel → mecanum_drive_controller → ESP32
    lifecycle_manager_navigation
```

TF chain: `map → odom → base_link → laser_frame`  
All four links must be present and current-timestamped (delay < 0.5 s) in all containers.  
In Nav mode (saved map), AMCL replaces SLAM Toolbox as the `map→odom` TF provider.

---

## Known Errors and Fixes Applied

| # | Error | Root Cause | Fix | Status |
|---|---|---|---|---|
| E1 | `bt_navigator` FATAL: `Cannot load lib…are_error_codes…bt_node.so` | 8 plugins in `plugin_lib_names` don't exist in ros-humble 1.1.20 | Removed 8 missing plugins from `nav2_params.yaml` | ✅ Applied |
| E2 | YAML parse error line 76 (`nav2_params.yaml`) | `multi_replace` left comment embedded inline with key | Fixed inline comment; confirmed `python3 -c "import yaml; yaml.safe_load(…)"` → OK | ✅ Applied |
| E3 | `Failed to compute odom pose` (SLAM continuous spam) | `transform_timeout: 0.5` too tight for DDS inter-container jitter | `transform_timeout: 1.0` in `slam_params.yaml` | ✅ Applied |
| E4 | `Message Filter queue is full` (SLAM) | Follows from E3 — scan arrives before TF buffer has `odom→base_link` | Fixed by E3 fix | ✅ Applied |
| E5 | Nav2 Goal tool in RViz doesn't send action goal | Used `rviz_default_plugins/SetGoal` (publishes topic, not action) | Replaced with `nav2_rviz_plugins/GoalTool`; added `Nav2Panel` in `rover.rviz` | ✅ Applied |
| E6 | Nav2 Panel / GoalTool crash on load | `nav2_rviz_plugins` not installed in Docker image | Added `ros-humble-nav2-rviz-plugins` to Dockerfile apt install | ✅ Applied |
| E7 | After image rebuild, containers still show old behaviour | `docker restart` reuses old container layer | Must use `docker compose --force-recreate` — documented in T009, T024, T025 | ✅ Documented |
| E8 | `map→odom` future timestamp warning (~0.34 s) | SLAM publishes with slightly future stamp vs wall clock | Cosmetic — absorbed by `transform_tolerance: 1.0` in `nav2_params.yaml` | ✅ Mitigated |

---

## Implementation Strategy

**Critical path** (blocking all user stories):  
T005 + T006 + T007 (verify fixes) → T008 (rebuild image) → T009 (force-recreate containers)

**US1 (P1 — MVP)**: SLAM maps the environment. Unlocked after T009. Verified by T010–T014.  
**US2 (P2)**: Nav2 goal navigation from RViz. Unlocked after US1 passes. Verified by T015–T018.  
**US3 (P3)**: Map save and AMCL reload. Independent of US2, runs after US1. Verified by T019–T023.

All Phase 1–3 fixes (E1–E8) are applied to working-tree source files but are **uncommitted**.
The Docker image predates some fixes and **must be rebuilt** before runtime verification.
