# Tasks: Full System Launch & Integration

**Feature branch**: `005-system-launch`
**Input**: `specs/005-system-launch/spec.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.
**Note**: This feature composes all previous packages (001–004) into a single
bringup package. It produces no new ROS2 node source code — only launch files,
config, and documentation.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the bringup package skeleton.

- [ ] T001 Create ROS2 package `ecza_bringup` with `ros2 pkg create --build-type ament_cmake ecza_bringup` in `src/ecza_bringup/`
- [ ] T002 [P] Add `package.xml` with `<exec_depend>` on `ecza_kinematics`, `ecza_teleop`, `ecza_hardware_bridge`, `joy_linux`, `rosbag2`, `diagnostic_aggregator` in `src/ecza_bringup/package.xml` per FR-008
- [ ] T003 [P] Create directory structure: `src/ecza_bringup/launch/`, `src/ecza_bringup/config/`, `src/ecza_bringup/doc/`

**Checkpoint**: `colcon build --packages-select ecza_bringup` succeeds with empty package.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The single system config YAML and its validation logic must exist
before any user story launch can be built on top of it.

- [ ] T004 Create `src/ecza_bringup/config/rover_params.yaml` with all required fields: `wheel_separation_width`, `wheel_base`, `wheel_radius`, `max_linear_speed`, `max_angular_speed`, `joy_deadzone`, `axis_linear_x`, `axis_linear_y`, `axis_angular_z`, `enable_button`, `agent_port`, `agent_bind_address`, `firmware_watchdog_timeout_ms`, `joy_watchdog_timeout_ms` per FR-003
- [ ] T005 Implement config file existence check in the main launch file (Phase 3): use `LaunchConfiguration` + `OnShutdown` guard that logs ERROR and prevents node startup if `rover_params.yaml` is not found at the expected path per FR-006
- [ ] T006 [P] Add `install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})` in `src/ecza_bringup/CMakeLists.txt` so config and launch files are installed into the ament share path

**Checkpoint**: `colcon build` installs config and launch files; missing config file produces a clear launch error before any node starts.

---

## Phase 3: User Story 1 — One-Command Full Stack Startup (Priority: P1) 🎯 MVP

**Goal**: `ros2 launch ecza_bringup rover.launch.py` starts all five components
(joy_linux, teleop, kinematics, micro-ros-agent, diagnostics aggregator) with
parameters from the single config YAML. All nodes appear in `ros2 node list`
within 15 seconds.

**Independent Test**: From a clean terminal with no nodes running and the ESP32
on the same WiFi network, run `ros2 launch ecza_bringup rover.launch.py`. Within
15 seconds, `ros2 node list` shows joy, teleop, kinematics, micro-ros-agent, and
the ESP32 node. Moving the left stick produces `/cmd_vel` messages.

- [ ] T007 [US1] Create `src/ecza_bringup/launch/rover.launch.py` composing: `joy_linux` node (device auto-detect), `ecza_teleop` node, `ecza_kinematics` node, `micro_ros_agent` node (UDP, port from config), `diagnostic_aggregator` node — all loading `rover_params.yaml` per FR-001, FR-002
- [ ] T008 [US1] Set `respawn=True` and `respawn_delay=2.0` on each node action in `rover.launch.py` (default behaviour; configurable via launch argument `respawn:=true/false`) per FR-005, US1-AC3. For the `micro_ros_agent` node specifically, prefer using `IncludeLaunchDescription` to include `ecza_hardware_bridge`'s `agent.launch.py` so respawn settings are maintained in one place (feature 004) and not duplicated here.
- [ ] T009 [US1] Wire `rover_params.yaml` as the parameter file for every composable node in `rover.launch.py` using `parameters=[LaunchConfiguration('params_file')]` per FR-002
- [ ] T010 [US1] Add `params_file` launch argument to `rover.launch.py` defaulting to the installed `rover_params.yaml` path; validate file existence before node startup per FR-006

**Checkpoint**: Single `ros2 launch` command → all five node names visible in `ros2 node list` within 15 s; pushing stick produces `/cmd_vel` traffic.

---

## Phase 4: User Story 2 — Single Config File for All Parameters (Priority: P2)

**Goal**: Editing `system_params.yaml` and relaunching takes effect immediately
with no source changes or image rebuilds needed.

**Independent Test**: Edit `max_linear_speed` to half its current value. Relaunch.
Push stick to full deflection. Confirm `/cmd_vel` linear.x equals the new value
(logged by the teleop node at startup).

- [ ] T011 [US2] Verify all nodes in `rover.launch.py` consume parameters exclusively from `rover_params.yaml` and not from any package-level default files that could shadow the system config; add a comment in the launch file documenting this intent per FR-002
- [ ] T012 [US2] Add a `validate_config.py` helper script `src/ecza_bringup/scripts/validate_config.py` that reads `rover_params.yaml` and checks all required keys are present; prints which keys are missing and exits non-zero per FR-006
- [ ] T013 [US2] Document in `src/ecza_bringup/doc/quickstart.md` the exact `rover_params.yaml` keys, their types, valid ranges, and what happens at runtime if a key is missing or misspelled per FR-007, US2-AC2

**Checkpoint**: Changing any value in `rover_params.yaml` and relaunching takes ≤30 s and the new value is reflected in node startup logs per SC-003.

---

## Phase 5: User Story 3 — rosbag2 Recording in One Command (Priority: P3)

**Goal**: `ros2 launch ecza_bringup rover.launch.py record:=true` additionally
starts a `ros2 bag record` process capturing `/joy`, `/cmd_vel`,
`/wheel_velocities`, `/diagnostics` into the configured output directory.

**Independent Test**: Launch with `record:=true`, drive for 30 s, stop. Confirm
a rosbag2 directory exists with all four topic streams. Play back with `ros2 bag
play` and confirm all topics publish.

- [ ] T014 [US3] Add `record` launch argument (default `false`) to `rover.launch.py`; when `true`, start a `ros2 bag record` `ExecuteProcess` action capturing `/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics` into `~/rover_bags/` (configurable via `bag_output_dir` launch argument) per FR-004, US3-AC1
- [ ] T015 [US3] Add `IfCondition(LaunchConfiguration('record'))` guard on the bag record action so no bag file is created when `record:=false` per US3-AC3
- [ ] T016 [US3] Document the `record:=true` usage, output directory, and playback command in `src/ecza_bringup/doc/quickstart.md` per FR-007

**Checkpoint**: `record:=true` → bag file in output dir after stop; `record:=false` (default) → no bag created; `ros2 bag play` replays all four topics.

---

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T017 [P] Finalize `src/ecza_bringup/doc/quickstart.md` with all required sections: prerequisites (ROS2 Humble, workspace build), F710 D-mode setup, WiFi/ESP32 provisioning link, launch command and all arguments, WiFi environment recommendations per FR-007
- [ ] T018 [P] Add `src/ecza_bringup/launch/teleop_only.launch.py` for development use (no hardware required): starts only joy_linux, teleop, and kinematics nodes — useful for verifying the software stack without the ESP32 or micro-ROS agent
- [ ] T019 Run full end-to-end system test: follow quickstart from scratch, verify SC-001 (operational within 10 s), SC-002 (param change + relaunch ≤30 s), SC-004 (5-min bag playable), SC-005 (30 min continuous without crash)

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    └── Phase 2 (Foundational — system_params.yaml + config validation)
            ├── Phase 3 (US1 — P1) 🎯 MVP
            │       └── Phase 4 (US2 — P2) [config isolation verification]
            ├── Phase 5 (US3 — P3) [recording flag]
            └── Phase 6 (Polish)
```

**Cross-feature dependencies**: This feature composes 001–004. Full US1 hardware
verification (ESP32 in `ros2 node list`) requires feature 001 firmware + feature
004 agent to be operational. Software-only verification (no ESP32) is possible
with T018 (`teleop_only.launch.py`).

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 3 (T001–T003) | 2 |
| Phase 2 — Foundational | 3 (T004–T006) | 1 |
| Phase 3 — US1 (P1) | 4 (T007–T010) | 0 |
| Phase 4 — US2 (P2) | 3 (T011–T013) | 0 |
| Phase 5 — US3 (P3) | 3 (T014–T016) | 0 |
| Phase 6 — Polish | 3 (T017–T019) | 2 |
| **Total** | **19** | **5** |
