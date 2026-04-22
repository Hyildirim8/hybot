# Feature Specification: Mecanum Kinematics Node

**Feature Branch**: `002-mecanum-kinematics`
**Created**: 2026-02-24
**Status**: Draft
**Input**: User description: "ROS2 Humble node that subscribes to /cmd_vel (geometry_msgs/Twist) and computes per-wheel velocities for a 4-wheel mecanum rover using wheel_separation_width=0.26m and wheel_base=0.38m, publishing results to /wheel_velocities"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Forward, Backward, and Strafe Motion Computation (Priority: P1)

A motion controller (teleop or autonomous) publishes a `Twist` message on
`/cmd_vel` requesting linear motion. The kinematics node transforms that command
into four individual wheel velocity values and publishes them so that the rover
moves as intended — forward, backward, or sideways — without rotating.

**Why this priority**: Linear motion is the most fundamental movement type and
is required to validate that the kinematic model is producing physically correct
outputs for the most common use cases.

**Independent Test**: Publish a known `/cmd_vel` Twist message (e.g., 0.5 m/s
forward) and verify that the four published wheel velocities on `/wheel_velocities`
match the analytically expected values for the mecanum kinematic formula, using
the configured geometry parameters.

**Acceptance Scenarios**:

1. **Given** the kinematics node is running with correct geometry parameters,
   **When** a `/cmd_vel` message with only `linear.x = 0.5` is published,
   **Then** all four wheel velocities are equal, positive, and match the expected
   value derived from the mecanum forward-kinematics formula.

2. **Given** the kinematics node is running,
   **When** a `/cmd_vel` message with only `linear.y = 0.3` (strafe) is published,
   **Then** front-left and rear-right wheels have equal positive velocities, and
   front-right and rear-left have equal negative velocities (or vice versa),
   matching the expected mecanum strafe formula.

3. **Given** the kinematics node is running,
   **When** a `/cmd_vel` message with all fields zero is published,
   **Then** all four published wheel velocities are zero.

---

### User Story 2 - Rotation and Compound Motion Computation (Priority: P2)

The kinematics node correctly handles in-place rotation commands and compound
commands combining linear and rotational components simultaneously.

**Why this priority**: Holonomic motion is the defining capability of a mecanum
rover. Rotation and compound motion are required for navigation and turning
manoeuvres.

**Independent Test**: Publish a pure rotation command (`angular.z` only) and verify
the four wheel velocities have alternating signs matching the expected rotational
kinematics formula. Then publish a combined linear + rotation command and verify
the output is the superposition of the two independent results.

**Acceptance Scenarios**:

1. **Given** the kinematics node is running,
   **When** a `/cmd_vel` message with only `angular.z = 1.0` is published,
   **Then** the four wheel velocities alternate in sign (left wheels positive,
   right wheels negative, or vice versa) with magnitudes consistent with the
   configured geometry.

2. **Given** the kinematics node is running,
   **When** a `/cmd_vel` message with both `linear.x = 0.5` and `angular.z = 0.5`
   is published,
   **Then** the four wheel velocities are the arithmetic superposition of the
   expected pure-linear and pure-rotation results.

---

### User Story 3 - Geometry Parameters Are Configurable at Runtime (Priority: P3)

All rover geometry values (wheel separation width, wheel base, wheel radius) are
exposed as ROS2 parameters and can be set via a parameter file or command-line
override without recompiling. A change in parameter values produces correspondingly
different wheel velocity outputs for the same `/cmd_vel` input.

**Why this priority**: Supporting configurable geometry ensures the node can be
reused across hardware revisions and enables wheel radius calibration without
touching source code — a Constitution Principle II requirement.

**Independent Test**: Launch the node with two different `wheel_radius` values,
send identical `/cmd_vel` inputs, and verify that the published wheel velocities
scale proportionally between the two runs.

**Acceptance Scenarios**:

1. **Given** the node is launched with a parameter file specifying geometry values,
   **When** the node starts,
   **Then** it logs all three geometry parameter values at INFO level.

2. **Given** the node is running,
   **When** a geometry parameter is overridden on the command line,
   **Then** subsequent `/wheel_velocities` outputs reflect the new parameter value.

3. **Given** `wheel_radius` is doubled compared to a reference run,
   **When** the same `/cmd_vel` input is sent,
   **Then** all four wheel velocities are halved compared to the reference run
   (inverse proportionality: smaller radius → faster wheel rotation for same
   linear speed).

---

### Edge Cases

- What happens when `/cmd_vel` contains non-zero `linear.z`, `angular.x`, or
  `angular.y`?
  → The node MUST silently ignore those components; mecanum rovers operate in 2D.
- What happens if a geometry parameter is set to zero or negative?
  → The node MUST log an ERROR and refuse to publish wheel velocities until
  valid parameters are provided.
- What happens if the `/cmd_vel` subscription receives messages faster than the
  node can process?
  → The node MUST process each message; no velocity command must be silently dropped
  (standard ROS2 QoS reliability applies).
- What happens when commanded velocities produce wheel speeds exceeding the
  physical maximum?
  → The kinematics node MUST NOT clamp; clamping is the responsibility of the
  hardware layer. The node MUST publish the mathematically correct (unclamped)
  value and let downstream handle saturation.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The node MUST subscribe to `/cmd_vel` (`geometry_msgs/Twist`) and
  publish to `/wheel_velocities` (`std_msgs/msg/Float32MultiArray`, array length
  4, order FL[0] FR[1] RL[2] RR[3], units rad/s) with individual speed values
  for all four wheels. This type MUST match the subscriber on the ESP32 firmware
  (feature 001 FR-001); Float64 variants MUST NOT be used.

- **FR-002**: The node MUST implement the standard mecanum inverse kinematic
  equations to compute per-wheel velocities from `linear.x`, `linear.y`, and
  `angular.z` components of the Twist message.

- **FR-003**: The node MUST expose the following as named ROS2 parameters with
  documented defaults:
  - `wheel_separation_width` (default: 0.26 m)
  - `wheel_base` (default: 0.38 m)
  - `wheel_radius` (default: 0.05 m — placeholder only; this value MUST be
    measured on the physical rover and updated in `config/rover_params.yaml`
    before any hardware motion tests. Using the placeholder during hardware
    tests will produce incorrect wheel velocities.)

- **FR-004**: The node MUST log all geometry parameter values at INFO level on
  startup.

- **FR-005**: The node MUST log an ERROR and cease publishing if any geometry
  parameter is zero or negative.

- **FR-006**: The node MUST ignore `linear.z`, `angular.x`, and `angular.y`
  components of the received Twist message without producing any error.

- **FR-007**: The node MUST publish to `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
  including current parameter values and operational status.

- **FR-008**: The node MUST be loadable from a parameter YAML file located under
  the package's `config/` directory.

- **FR-009**: The node MUST log the timestamp of the most recently processed
  `/cmd_vel` message at DEBUG level for correlation during diagnostics.
  `std_msgs/Float32MultiArray` does not carry a header field; timestamp
  correlation is therefore performed via the `/diagnostics` topic (FR-007) which
  MUST include a `message: "last_cmd_vel_stamp: <sec>.<nanosec>"` key-value pair.

### Key Entities

- **Twist Command**: Input from `/cmd_vel` — `linear.x` (forward/back m/s),
  `linear.y` (strafe m/s), `angular.z` (rotation rad/s). Other fields ignored.

- **WheelVelocities**: Output on `/wheel_velocities` — `std_msgs/msg/Float32MultiArray`
  with `data` array of length 4: index 0 = front-left, 1 = front-right, 2 = rear-left,
  3 = rear-right, each value in rad/s (signed). The `layout` field is not populated.

- **RoverGeometry**: The three named parameters (`wheel_separation_width`,
  `wheel_base`, `wheel_radius`) that together define the mecanum kinematic model.

## Assumptions

- The mecanum wheels are arranged in the standard X-drive orientation:
  front-left and rear-right share one roller diagonal; front-right and rear-left
  share the other. The kinematic equations assume this standard orientation.
- Output wheel velocities are in rad/s (angular velocity at the wheel hub).
  The hardware layer is responsible for converting rad/s to PWM duty cycles.
- The `wheel_radius` value will be populated via parameter file before hardware
  tests. During software development a placeholder value (e.g., 0.05 m) may be
  used.
- The node publishes `std_msgs/msg/Float32MultiArray` on `/wheel_velocities`.
  This was decided to match the ESP32 micro-ROS subscriber (feature 001 FR-001),
  which only has first-class support for float32. Float64MultiArray MUST NOT be
  used; the decision is final and not deferred to plan time.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: For any valid `/cmd_vel` Twist input, the four published wheel
  velocities match the analytically expected mecanum kinematic formula values to
  within a floating-point tolerance of ±0.001 rad/s.

- **SC-002**: Pure forward, pure backward, pure strafe-left, pure strafe-right,
  pure rotate-left, pure rotate-right, and two compound commands (8 total) all
  produce physically correct wheel velocity sign patterns in hardware or simulation
  verification.

- **SC-003**: Changing `wheel_separation_width`, `wheel_base`, or `wheel_radius`
  via parameter override produces correspondingly scaled output velocities without
  recompiling the node.

- **SC-004**: The node processes `/cmd_vel` messages and publishes corresponding
  `/wheel_velocities` messages with a latency under 10 ms on the development
  machine.

- **SC-005**: Zero-velocity Twist input always produces zero wheel velocities
  (no drift due to floating-point arithmetic).
