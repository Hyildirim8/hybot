# Tasks: Joystick Teleop Node

**Feature branch**: `003-joystick-teleop`
**Input**: `specs/003-joystick-teleop/spec.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the ROS2 package skeleton for the teleop node.

- [ ] T001 Create ROS2 package `ecza_teleop` with `ros2 pkg create --build-type ament_cmake ecza_teleop` in `src/ecza_teleop/`
- [ ] T002 [P] Add `package.xml` with `<depend>` on `rclcpp`, `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs` in `src/ecza_teleop/package.xml`
- [ ] T003 [P] Create `src/ecza_teleop/config/f710_dmode_params.yaml` with default axis mappings (axis_linear_x=1, axis_linear_y=0, axis_angular_z=3), speed limits (max_linear_speed=0.5, max_angular_speed=1.0), joy_deadzone=0.05, enable_button=5, require_enable_button=true

**Checkpoint**: `colcon build --packages-select ecza_teleop` succeeds with empty node.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Parameter infrastructure, axis mapping struct, and scaling logic
that all three stories depend on.

- [ ] T004 Declare all ROS2 parameters (`axis_linear_x`, `axis_linear_y`, `axis_angular_z`, `max_linear_speed`, `max_angular_speed`, `joy_deadzone`, `enable_button`, `require_enable_button`) with documented defaults in `src/ecza_teleop/src/teleop_node.cpp` per FR-002, FR-003, FR-004, FR-005, FR-006
- [ ] T005 [P] Implement startup parameter logging: log all parameter values at INFO level before the first Joy message is processed per FR-009 in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T006 [P] Implement `apply_deadzone(float value, float threshold) -> float` that returns 0.0 when `|value| < threshold` per FR-004 in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T007 Implement axis index validator: at startup, warn if any axis index parameter exceeds the known F710 axis count (8 axes in D-mode) per FR-011 in `src/ecza_teleop/src/teleop_node.cpp`

**Checkpoint**: Node starts, logs all parameters, deadzone function computes correctly.

---

## Phase 3: User Story 1 — Drive with Left and Right Sticks (Priority: P1) 🎯 MVP

**Goal**: Node reads `/joy`, scales axes by speed limits, applies deadzone, and
publishes proportional `Twist` on `/cmd_vel` for linear.x and angular.z.

**Independent Test**: With only the `joy` node and this node running (no hardware),
publish a simulated `/joy` message with `axes[1]=-1.0` (left stick fully forward,
negative by convention). Confirm `/cmd_vel` publishes `linear.x = max_linear_speed`.
Verify 50% deflection publishes exactly 50% of max speed.

- [ ] T008 [US1] Create `/joy` subscriber (`sensor_msgs/Joy`, QoS RELIABLE) in `src/ecza_teleop/src/teleop_node.cpp` per FR-001
- [ ] T009 [US1] Implement `joy_to_twist(Joy msg, params) -> Twist` that reads `axes[axis_linear_x]`, `axes[axis_angular_z]`, applies deadzone, multiplies by speed limits, and corrects sign (left-stick forward = positive linear.x by REP-103) per FR-002, FR-003 in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T010 [US1] Create `/cmd_vel` publisher (`geometry_msgs/Twist`) and publish result of `joy_to_twist()` in the `/joy` subscription callback per FR-001 in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T011 [US1] Set `linear.y = 0` and all unused Twist fields to 0 in `joy_to_twist()` (US2 adds strafe in Phase 4); publish zero Twist when all axes within deadzone per SC-002

**Checkpoint**: Simulated Joy message with full forward deflection → `linear.x == max_linear_speed`; centred sticks → all-zero Twist.

---

## Phase 4: User Story 2 — Holonomic Strafing (Priority: P2)

**Goal**: `axes[axis_linear_y]` is mapped to `linear.y` so the rover strafes.
Diagonal stick input produces simultaneous non-zero linear.x and linear.y.

**Independent Test**: Publish `/joy` with `axes[0]=1.0` (full right strafe),
confirm `linear.y == max_linear_speed` and `linear.x == 0.0`. Diagonal input
(axes[0]=0.7, axes[1]=-0.7) → both `linear.y` and `linear.x` proportional.

- [ ] T012 [US2] Add `axes[axis_linear_y]` extraction with deadzone and scale by `max_linear_speed` and assign to `Twist.linear.y` inside `joy_to_twist()` in `src/ecza_teleop/src/teleop_node.cpp` per FR-002, US2-AC1

**Checkpoint**: Strafe command publishes correct `linear.y`; diagonal input produces both `linear.x` and `linear.y` non-zero and proportional.

---

## Phase 5: User Story 3 — Enable Button Safety Lock (Priority: P3)

**Goal**: When `require_enable_button=true`, non-zero Twist is published only
while the enable button is held. Releasing it immediately publishes a zero Twist.
The guard is bypassable via parameter.

**Independent Test**: Move a stick while NOT holding the enable button → only
zero-velocity Twist published. Hold the button + move stick → motion command
published. Release button mid-motion → zero Twist published within one Joy cycle.

- [ ] T013 [US3] Implement `EnableGuard` logic in `joy_to_twist()`: if `require_enable_button=true` and `buttons[enable_button] != 1`, return zero Twist; otherwise return the computed Twist per FR-005, FR-006, FR-007 in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T014 [US3] Implement `/joy` topic watchdog: create a `rclcpp::WallTimer` that fires after `joy_watchdog_timeout_ms` (default: 500 ms) and publishes a zero Twist if no Joy message has been received within that window per FR-008 in `src/ecza_teleop/src/teleop_node.cpp`; reset the timer on every received Joy message
- [ ] T015 [US3] Add `joy_watchdog_timeout_ms` as a declared ROS2 parameter (default: 500) and wire it to the watchdog timer period in `src/ecza_teleop/src/teleop_node.cpp` per FR-008

**Checkpoint**: Stick movement without enable button → zero Twist only. Watchdog fires after 500 ms of Joy silence → zero Twist published. `require_enable_button: false` bypasses the guard.

---

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T016 [P] Create `src/ecza_teleop/launch/teleop.launch.py` that starts the `joy_linux` node and the teleop node, loading `config/f710_dmode_params.yaml` as a shared parameter file per FR-010
- [ ] T017 [P] Implement `/diagnostics` publisher (`diagnostic_msgs/DiagnosticArray`) at 1 Hz in `src/ecza_teleop/src/teleop_node.cpp`, reporting: all parameter values (`axis_linear_x`, `axis_linear_y`, `axis_angular_z`, `joy_deadzone`, `enable_button`, speed limits), watchdog state (active/timed_out), and enable guard state (enabled/disabled). Required by Constitution Principle V; `rqt_robot_monitor` MUST display the teleop node status.
- [ ] T018 [P] Add duplicate axis mapping detection at startup: check if any two of `axis_linear_x`, `axis_linear_y`, `axis_angular_z` share the same index and log WARN if so per Edge Cases in `src/ecza_teleop/src/teleop_node.cpp`
- [ ] T019 Add `README.md` to `src/ecza_teleop/` documenting: topic names, QoS, all parameter names and defaults, F710 D-mode vs X-mode axis tables, enable button default (index 5 = RB in D-mode), watchdog default

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    └── Phase 2 (Foundational — params, deadzone, axis validator)
            ├── Phase 3 (US1 — P1) 🎯 MVP
            │       └── Phase 4 (US2 — P2) [adds linear.y to existing callback]
            ├── Phase 5 (US3 — P3) [enable guard + watchdog]
            └── Phase 6 (Polish)
```

US2 is a single additive task on T009 — no new files. US3 is independent of US2.

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 3 (T001–T003) | 2 |
| Phase 2 — Foundational | 4 (T004–T007) | 2 |
| Phase 3 — US1 (P1) | 4 (T008–T011) | 0 |
| Phase 4 — US2 (P2) | 1 (T012) | 0 |
| Phase 5 — US3 (P3) | 3 (T013–T015) | 0 |
| Phase 6 — Polish | 4 (T016–T019) | 3 |
| **Total** | **19** | **6** |
