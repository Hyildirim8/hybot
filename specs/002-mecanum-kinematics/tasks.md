# Tasks: Mecanum Kinematics Node

**Feature branch**: `002-mecanum-kinematics`
**Input**: `specs/002-mecanum-kinematics/spec.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the ROS2 package skeleton and configure the colcon build.

- [ ] T001 Create ROS2 Python (or C++) package `ecza_kinematics` with `ros2 pkg create --build-type ament_cmake ecza_kinematics` (or `ament_python`) in `src/ecza_kinematics/`
- [ ] T002 [P] Add `package.xml` with `<depend>` on `rclcpp` (or `rclpy`), `geometry_msgs`, `std_msgs`, `diagnostic_msgs` in `src/ecza_kinematics/package.xml`
- [ ] T003 [P] Create `src/ecza_kinematics/config/kinematics_params.yaml` with default values: `wheel_separation_width: 0.26`, `wheel_base: 0.38`, `wheel_radius: 0.05` (placeholder; measured at calibration)

**Checkpoint**: `colcon build --packages-select ecza_kinematics` succeeds with an empty node.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core parameter infrastructure and message definition that all
three stories depend on.

**⚠️ CRITICAL**: The geometry parameter struct and parameter validation guard
must exist before any kinematic formula can be implemented.

- [ ] T004 Implement `RoverGeometry` parameter struct and `declare_geometry_params()` that declares `wheel_separation_width`, `wheel_base`, `wheel_radius` with documented defaults in `src/ecza_kinematics/src/kinematics_node.cpp` (or `.py`)
- [ ] T005 Implement `validate_geometry_params(RoverGeometry) -> bool` that returns false and logs ERROR if any value is ≤ 0; call on startup and on any parameter change event per FR-005 in `src/ecza_kinematics/src/kinematics_node.cpp`
- [ ] T006 [P] Implement startup parameter logging: call `RCLCPP_INFO` (or `get_logger()`) for each of the three geometry values per FR-004 in `src/ecza_kinematics/src/kinematics_node.cpp`

**Checkpoint**: Node starts, declares parameters, logs their values, and refuses to publish if any parameter is ≤ 0.

---

## Phase 3: User Story 1 — Forward, Backward, and Strafe Motion (Priority: P1) 🎯 MVP

**Goal**: Node subscribes to `/cmd_vel`, applies mecanum inverse kinematics for
linear-only commands, and publishes correct per-wheel velocities to `/wheel_velocities`.

**Independent Test**: `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'`
→ all four wheel velocities are equal and match the formula
`v_wheel = linear_x / wheel_radius`. Pure strafe: FL/RR positive, FR/RL negative
(or vice versa).

- [ ] T007 [US1] Implement mecanum inverse kinematics function `compute_wheel_velocities(linear_x, linear_y, angular_z, geometry) -> [fl, fr, rl, rr]` in `src/ecza_kinematics/src/kinematics_node.cpp`: formula uses `(wheel_separation_width + wheel_base) / 2` as the combined half-geometry term per FR-002
- [ ] T008 [US1] Implement `ignore_unused_twist_components()`: discard `linear.z`, `angular.x`, `angular.y` silently per FR-006 in the subscription callback in `src/ecza_kinematics/src/kinematics_node.cpp`
- [ ] T009 [US1] Create `/cmd_vel` subscriber (`geometry_msgs/Twist`, QoS RELIABLE) and wire to `compute_wheel_velocities()` in `src/ecza_kinematics/src/kinematics_node.cpp` per FR-001
- [ ] T010 [US1] Create `/wheel_velocities` publisher (`std_msgs/Float32MultiArray`, order FL[0] FR[1] RL[2] RR[3], units rad/s) — type MUST match ESP32 subscriber (001 FR-001); populate `data` array length 4 from `compute_wheel_velocities()` result per FR-001 in `src/ecza_kinematics/src/kinematics_node.cpp`
- [ ] T011 [US1] Add zero-output guard: if all Twist fields are zero, publish all-zero wheel velocities without calling the full formula per SC-005 in `src/ecza_kinematics/src/kinematics_node.cpp`

**Checkpoint**: `ros2 topic pub /cmd_vel` with pure linear.x, linear.y produce analytically correct FL/FR/RL/RR values to within ±0.001 rad/s; all-zero input produces all-zero output.

---

## Phase 4: User Story 2 — Rotation and Compound Motion (Priority: P2)

**Goal**: The same kinematics function correctly handles `angular.z` and
compound (linear + angular) commands, verified against the expected superposition.

**Independent Test**: `ros2 topic pub /cmd_vel` with pure `angular.z=1.0` → left wheels
positive, right wheels negative (or vice versa), magnitudes equal to
`(wheel_separation_width + wheel_base) / 2 * angular_z / wheel_radius`.
Compound command output equals sum of linear-only and rotation-only results.

- [ ] T012 [US2] Verify that `compute_wheel_velocities()` already handles `angular_z` via the standard mecanum formula (`fl += k*angular_z`, `fr -= k*angular_z`, `rl += k*angular_z`, `rr -= k*angular_z` where `k = (lx+ly)/2`) — add a dedicated compound-command integration test scenario to the manual test checklist in `docs/kinematics_test_scenarios.md`

> Note: T012 is a validation step. If the formula from T007 is correct, US2 is already implemented. This task confirms it and records the test scenarios.

**Checkpoint**: Eight canonical motion patterns (forward, backward, strafe-left, strafe-right, rotate-CW, rotate-CCW, diagonal-FL, diagonal-FR) all produce correct sign patterns per SC-002.

---

## Phase 5: User Story 3 — Configurable Geometry Parameters (Priority: P3)

**Goal**: All three geometry parameters are exposed as ROS2 parameters, loadable
from YAML, overridable at launch, and cause immediate scaling change in outputs.

**Independent Test**: Launch the node with `wheel_radius:=0.10` (double the default),
publish identical `/cmd_vel`, confirm all wheel velocities are halved.

- [ ] T013 [US3] Add `on_set_parameters_callback` that re-validates geometry on any parameter update and re-logs the new values per US3-AC2, FR-005 in `src/ecza_kinematics/src/kinematics_node.cpp`
- [ ] T014 [US3] Ensure `src/ecza_kinematics/config/kinematics_params.yaml` uses `ros__parameters` format for YAML parameter loading and is referenced from the launch file per FR-008
- [ ] T015 [US3] Create `src/ecza_kinematics/launch/kinematics.launch.py` that starts the node and loads `config/kinematics_params.yaml`; expose a launch argument `params_file` to override the default config path per FR-008

**Checkpoint**: `ros2 launch ecza_kinematics kinematics.launch.py wheel_radius:=0.10` produces wheel velocities at half scale compared to default for identical `/cmd_vel` input.

---

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T016 [P] Implement `/diagnostics` publisher (`diagnostic_msgs/DiagnosticArray`) reporting current geometry parameter values, operational status (`OK` / `ERROR`), and a `last_cmd_vel_stamp` key-value pair (timestamp of most recently processed `/cmd_vel`) per FR-007, FR-009 in `src/ecza_kinematics/src/kinematics_node.cpp`
- [ ] T017 [P] Add `<exec_depend>` entries in `package.xml` and verify `colcon build` and `colcon test` pass cleanly with no unresolved dependencies
- [ ] T018 Add `README.md` to `src/ecza_kinematics/` documenting: topic names, message types, QoS, all parameter names and defaults, mecanum formula reference, and the 8-pattern test table

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    └── Phase 2 (Foundational — geometry struct + validation)
            ├── Phase 3 (US1 — P1) 🎯 MVP
            │       └── Phase 4 (US2 — P2) [formula already complete at T007]
            ├── Phase 5 (US3 — P3) [parameter wiring]
            └── Phase 6 (Polish)
```

US2 has no new implementation tasks if the formula in T007 is correct — T012 only validates. US3 (T013–T015) is independent of US1/US2.

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 3 (T001–T003) | 2 |
| Phase 2 — Foundational | 3 (T004–T006) | 1 |
| Phase 3 — US1 (P1) | 5 (T007–T011) | 0 |
| Phase 4 — US2 (P2) | 1 (T012) | 0 |
| Phase 5 — US3 (P3) | 3 (T013–T015) | 0 |
| Phase 6 — Polish | 3 (T016–T018) | 2 |
| **Total** | **18** | **5** |
