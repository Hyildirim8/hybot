# Feature Specification: Joystick Teleop Node

**Feature Branch**: `003-joystick-teleop`
**Created**: 2026-02-24
**Status**: Draft
**Input**: User description: "ROS2 Humble teleop node that reads Logitech F710 joystick sensor_msgs/Joy messages and converts axis inputs to geometry_msgs/Twist on /cmd_vel with configurable axis mappings, deadzone, and speed scaling"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Drive the Rover with Left and Right Sticks (Priority: P1)

An operator picks up the Logitech F710 controller (set to D-mode), pushes the
left stick forward/backward and the right stick left/right. The rover responds
by moving forward/backward and rotating respectively. The mapping feels intuitive
— pushing the stick harder makes the rover move faster, up to the configured
maximum speed.

**Why this priority**: This is the minimum viable teleoperation capability. Without
it the rover cannot be driven at all. All other features (kinematics, hardware
bridge) are useless until this story works end-to-end.

**Independent Test**: With only the `joy` node and the teleop node running (no
hardware), push the left stick fully forward and verify that a `/cmd_vel` message
with a positive `linear.x` equal to the configured maximum linear speed is
published. Verify the same proportional scaling at 50% stick deflection.

**Acceptance Scenarios**:

1. **Given** the teleop node is running and the F710 is connected in D-mode,
   **When** the left stick is pushed fully forward,
   **Then** a `/cmd_vel` Twist message is published with `linear.x` equal to the
   configured `max_linear_speed` and `linear.y`, `angular.z` equal to zero.

2. **Given** the teleop node is running,
   **When** the left stick is at 50% forward deflection,
   **Then** `linear.x` in the published Twist equals 50% of `max_linear_speed`.

3. **Given** the teleop node is running,
   **When** the right stick is pushed fully to the right,
   **Then** `angular.z` in the published Twist equals the configured
   `max_angular_speed` (negative, indicating clockwise rotation by convention),
   and `linear.x`, `linear.y` are zero.

4. **Given** all sticks are centred (within deadzone),
   **When** a Joy message is received,
   **Then** a zero Twist (`linear.x = linear.y = angular.z = 0`) is published.

---

### User Story 2 - Holonomic Strafing with Left Stick Lateral Axis (Priority: P2)

The operator pushes the left stick sideways. The rover strafes left or right
without rotating, exploiting the mecanum wheel capability. This must feel direct:
the rover moves in the direction the stick points.

**Why this priority**: Lateral strafing is the key capability that distinguishes
a mecanum rover from a differential-drive robot. Without this story the mecanum
platform offers no advantage over a simpler design.

**Independent Test**: Push the left stick fully to the right with no forward/backward
deflection. Verify `/cmd_vel` publishes `linear.y` equal to `max_linear_speed`
(or `-max_linear_speed` depending on axis polarity) with all other fields zero.

**Acceptance Scenarios**:

1. **Given** the teleop node is running,
   **When** the left stick is pushed fully to the right (no forward/backward),
   **Then** the published Twist has `linear.y` equal to the maximum strafe speed
   and `linear.x`, `angular.z` equal to zero.

2. **Given** the teleop node is running,
   **When** the left stick is pushed diagonally (forward + right simultaneously),
   **Then** the published Twist has both `linear.x` and `linear.y` non-zero, with
   each proportional to the stick deflection on its respective axis.

---

### User Story 3 - Enable Button Safety Lock (Priority: P3)

The teleop node requires the operator to hold a designated "enable" button on the
controller to allow motion commands to be published. When the enable button is
released, the node immediately publishes a zero Twist, stopping the rover.

**Why this priority**: A safety dead-man's switch prevents accidental motion
when the controller is bumped or picked up. This is a key safety feature for
hardware testing.

**Independent Test**: Move a stick fully forward while NOT holding the enable
button. Verify that no non-zero `/cmd_vel` is published. Then hold the enable
button and verify motion commands flow normally.

**Acceptance Scenarios**:

1. **Given** the teleop node is running and enable button is not held,
   **When** any stick is moved,
   **Then** only zero-velocity Twist messages are published (or no messages,
   depending on configuration).

2. **Given** the enable button is held and the rover is moving,
   **When** the enable button is released,
   **Then** a zero Twist is immediately published.

3. **Given** the enable button feature is disabled via parameter,
   **When** sticks are moved without holding any button,
   **Then** motion commands are published normally (enable button check is skipped).

---

### Edge Cases

- What happens when the F710 is disconnected mid-operation?
  → The `joy` node stops publishing; the teleop node MUST detect the `/joy` topic
  going silent and publish a zero Twist within the configured watchdog period.
- What happens when axes produce non-zero output at the centre position (stick drift)?
  → The `joy_deadzone` parameter MUST suppress values below the threshold, treating
  them as zero.
- What happens when two axes are mapped to the same Twist component?
  → Parameter validation MUST detect duplicate mappings at startup and log a WARN.
  Last-write-wins behaviour is acceptable but must be documented.
- What happens when the F710 toggle is set to X-mode (XInput) instead of D-mode?
  → Axis indices will be different; the user must be informed via documentation.
  The node MUST NOT silently produce incorrect motion.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The node MUST subscribe to `/joy` (`sensor_msgs/Joy`) and publish
  `geometry_msgs/Twist` to `/cmd_vel`.

- **FR-002**: The node MUST map joystick axes to Twist components via named ROS2
  parameters:
  - `axis_linear_x` (default: left stick vertical axis index)
  - `axis_linear_y` (default: left stick horizontal axis index)
  - `axis_angular_z` (default: right stick horizontal axis index)

- **FR-003**: The node MUST scale axis values by configurable speed limits:
  - `max_linear_speed` (m/s)
  - `max_angular_speed` (rad/s)

- **FR-004**: The node MUST apply a `joy_deadzone` threshold: axis values with
  absolute magnitude below this threshold MUST be treated as zero.

- **FR-005**: The node MUST support an enable button (`enable_button` parameter,
  default: a designated button index). When enabled, motion commands are only
  published while this button is held.

- **FR-006**: The enable button feature MUST be disableable via
  `require_enable_button: false` parameter.

- **FR-007**: When the enable button is released (or the feature triggers a stop),
  the node MUST publish a zero Twist immediately.

- **FR-008**: The node MUST implement a `/joy` topic watchdog: if no Joy message
  is received within a configurable timeout, a zero Twist MUST be published.

- **FR-009**: The node MUST log all parameter values (axis mappings, speed limits,
  deadzone, enable button index) at INFO level on startup.

- **FR-010**: The node MUST be configurable from a YAML parameter file under the
  package's `config/` directory.

- **FR-011**: The node MUST validate axis index parameters at startup and log a
  WARN if any index exceeds the known axis count for the F710 controller.

### Key Entities

- **JoyInput**: The `sensor_msgs/Joy` message — `axes[]` (float32 array, range
  −1.0 to +1.0) and `buttons[]` (int32 array, 0 or 1).

- **TwistOutput**: The `geometry_msgs/Twist` message published to `/cmd_vel` —
  `linear.x`, `linear.y`, `angular.z` populated; all other fields zero.

- **AxisMapping**: The parameter set that binds a Joy axis index to a Twist field
  with a speed scale factor.

- **EnableGuard**: The logic that suppresses Twist output when the enable button
  is not held. Can be bypassed via parameter.

## Assumptions

- Logitech F710 in D-mode axis layout (standard Linux joystick driver mapping):
  - Axis 1: left stick vertical (forward = negative by convention; node corrects sign)
  - Axis 0: left stick horizontal
  - Axis 3: right stick horizontal
  The default parameter values are set for D-mode. X-mode users must override parameters.
- The enable button defaults to the right bumper (RB), which is button index 5 in
  D-mode. This can be overridden via the `enable_button` parameter.
- The teleop node publishes at the rate of incoming Joy messages (typically 50 Hz
  from the joy node). No additional rate limiting is required at this stage.
- Sign conventions (positive `linear.x` = forward, positive `angular.z` =
  counter-clockwise) follow the ROS REP-103 standard.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Stick deflection linearly maps to Twist output; a 100% deflection
  produces exactly `max_linear_speed` or `max_angular_speed` and a 50% deflection
  produces exactly half those values (within floating-point precision).

- **SC-002**: Stick values within the deadzone produce exactly zero Twist output
  with no residual drift.

- **SC-003**: Releasing the enable button or losing the `/joy` topic causes a
  zero Twist to be published within 100 ms.

- **SC-004**: All six motion patterns (forward, backward, strafe-left,
  strafe-right, rotate-left, rotate-right) are achievable with single-axis stick
  inputs using the default F710 D-mode parameter file.

- **SC-005**: The node starts and is operational within 2 seconds of launch,
  with all parameters logged before the first Joy message is processed.
