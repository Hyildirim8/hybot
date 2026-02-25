# Feature Specification: ESP32-S3 Motor Firmware

**Feature Branch**: `001-esp32-firmware`
**Created**: 2026-02-24
**Status**: Draft
**Input**: User description: "the firmware that controls the robot is esp32-s3-wroom-1 that controls the 4 wheels with 4 different dc motor drivers (BTS7960B H-bridge drivers, GB37-520 DC motors), communicating with ROS2 via WiFi"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Receive and Execute Wheel Velocity Commands (Priority: P1)

An operator (or the ROS2 system) sends per-wheel velocity targets to the ESP32
firmware. The firmware translates those targets into motor driver signals and the
four mecanum wheels spin at the commanded speeds within a short response time.
This is the core capability without which no robot motion is possible.

**Why this priority**: Without reliable velocity command execution there is no
functional robot. Everything else (teleoperation, kinematics correctness,
diagnostics) depends on this story being operational.

**Independent Test**: Connect the ESP32 to the WiFi network, send a structured
command (via micro-ROS or a test client on the same network) encoding four distinct
wheel speed values, and verify — using a tachometer or motor encoder readback —
that each wheel spins at the corresponding commanded speed. Confirm BTS7960B
RPWM/LPWM duty cycles match expectations for each commanded speed and direction.

**Acceptance Scenarios**:

1. **Given** the firmware is running and all four motor drivers are powered,
   **When** a command message with four non-zero speed values is received,
   **Then** each of the four wheels begins rotating at the corresponding commanded
   speed within 100 ms.

2. **Given** the firmware is running,
   **When** a command message with all four speeds set to zero is received,
   **Then** all four wheels stop within 100 ms.

3. **Given** the firmware is running,
   **When** a mix of positive (forward) and negative (reverse) speed values is
   commanded per-wheel (as required by mecanum holonomic motion),
   **Then** each wheel spins in the correct direction as indicated by the sign of
   its commanded speed.

4. **Given** the firmware is running,
   **When** a commanded speed value exceeds the configured maximum,
   **Then** the firmware clamps the output to the maximum and does not damage the
   motor driver.

---

### User Story 2 - Safe Stop on Communication Loss (Priority: P2)

If the firmware stops receiving valid wheel velocity commands for longer than a
configured timeout (watchdog), it immediately commands all four motors to stop.
This prevents the rover from continuing to move when the controlling system
crashes or the communication link is interrupted.

**Why this priority**: Safety during hardware testing is critical. A runaway rover
with no e-stop presents a risk of physical damage. This is a prerequisite for
unsupervised or bench-free testing.

**Independent Test**: Send a stream of commands, then deliberately stop sending
them. Verify — by observation or encoder readback — that all wheels stop after
the watchdog period expires, without any additional input.

**Acceptance Scenarios**:

1. **Given** wheels are spinning under an active command stream,
   **When** no new command is received for longer than the watchdog timeout,
   **Then** all motors are commanded to zero speed automatically.

2. **Given** a watchdog stop has occurred,
   **When** a valid command message is received again,
   **Then** the firmware resumes normal operation and executes the new command.

3. **Given** the firmware just powered on,
   **When** no command has ever been received,
   **Then** all motors remain stopped (safe default state).

---

### User Story 3 - Firmware Status Reporting (Priority: P3)

The firmware periodically reports its operational status (connection state,
current commanded speeds, watchdog state, any error flags) back to the host
system. The host ROS2 node can surface this information as diagnostics.

**Why this priority**: Without status reporting, debugging hardware faults during
integration is very difficult. This story enables the ROS2 hardware node to
publish meaningful diagnostics per Constitution Principle V.

**Independent Test**: Using a ROS2 host on the same WiFi network as the ESP32,
run `ros2 topic echo /firmware_status` without sending any commands. Verify that
status messages arrive at a regular interval and contain the expected fields
(commanded speeds per wheel, watchdog state, per-motor error flags). USB is not
used for operational data; all status reporting occurs over the micro-ROS WiFi
link.

**Acceptance Scenarios**:

1. **Given** the firmware is running,
   **When** no external tool reads status,
   **Then** status frames are still emitted at a regular interval (at least once
   per second).

2. **Given** a watchdog stop has been triggered,
   **When** status is read,
   **Then** the watchdog-triggered state is reflected in the status report.

3. **Given** a motor driver reports a fault condition (over-current, over-temp),
   **When** status is read,
   **Then** the fault is identified per-motor in the status report.

---

### Edge Cases

- What happens when a received command message is malformed, truncated, or has
  an invalid checksum (e.g., due to WiFi packet corruption)?
  → Firmware MUST discard the message, not crash, and increment an error counter.
- What happens if one motor driver fails to respond while others are healthy?
  → The firmware MUST stop all motors and set an error flag; partial motion is
  not permitted.
- What happens during brown-out or power instability on the ESP32?
  → Hardware watchdog (independent of software) MUST reset the ESP32; motors
  MUST default to stopped state on reset.
- What happens if commanded speed values use floating-point and the BTS7960B
  requires integer PWM duty cycles?
  → The firmware MUST define a clear mapping from speed units to PWM duty cycle
  using the formula `duty = (|speed_rad_s| / max_speed_rad_s) × PWM_MAX_TICKS`,
  where `PWM_MAX_TICKS` is the timer period register value (implementation-defined,
  e.g. 1000 for a 1 kHz PWM). The result MUST be rounded to the nearest integer
  (round-half-up). Negative values of `duty` are not possible; direction is
  handled by selecting RPWM vs LPWM, not by the duty cycle sign.
- What happens if both RPWM and LPWM are set to non-zero simultaneously on a
  BTS7960B channel (brake/shoot-through condition)?
  → The firmware MUST ensure only one of RPWM or LPWM is driven per channel at
  any time; simultaneous non-zero outputs MUST be treated as a firmware fault.
- What happens when the WiFi connection to the ROS2 host is lost?
  → The firmware MUST detect the micro-ROS session loss and trigger the software
  watchdog, commanding all motors to stop.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The firmware MUST accept a `std_msgs/msg/Float32MultiArray` message
  on topic `/wheel_velocities` containing four independent signed speed values
  in radians per second (rad/s), one per wheel in order: front-left [0],
  front-right [1], rear-left [2], rear-right [3].

- **FR-002**: The firmware MUST translate each received speed value into
  corresponding RPWM and LPWM duty cycle signals for the associated BTS7960B
  motor driver channel within 100 ms of message receipt. For forward motion
  (positive speed), LPWM MUST be driven and RPWM MUST be zero; for reverse
  (negative speed), RPWM MUST be driven and LPWM MUST be zero; for stop,
  both MUST be zero.

- **FR-003**: The firmware MUST independently control the direction (forward /
  reverse) of each GB37-520 motor by selecting the appropriate BTS7960B PWM
  channel (RPWM for reverse, LPWM for forward) based on the sign of the
  commanded speed.

- **FR-004**: The firmware MUST clamp any commanded speed that exceeds the
  configured maximum to that maximum value without crashing or generating
  undefined behaviour.

- **FR-005**: The firmware MUST implement a software watchdog: if no valid
  command message is received within a configurable timeout period, all four
  motors MUST be commanded to zero speed immediately.

- **FR-006**: The firmware MUST resume normal command processing after a watchdog
  stop, upon receipt of the next valid command message.

- **FR-007**: The firmware MUST emit a status report at a regular interval
  (minimum 1 Hz) containing: current commanded speed per wheel, watchdog state
  (active / timed-out), and per-motor error flags.

- **FR-008**: The firmware MUST discard malformed or incomplete messages without
  crashing; a malformed-message counter MUST be maintained and included in the
  status report.

- **FR-009**: The firmware MUST support a configurable maximum speed parameter
  (mapping from speed units to maximum PWM duty cycle) stored in non-volatile
  memory or set at compile time via a named constant. The default value MUST be
  `10.0 rad/s` (a conservative upper bound pending GB37-520 calibration; the
  exact rated speed must be measured and updated in NVS before hardware tests).

- **FR-010**: The firmware MUST stop all motors and enter a safe state on any
  hardware reset or brown-out event. Specifically: on reset, all RPWM and LPWM
  GPIO outputs MUST be driven LOW (0% duty cycle) before any other
  initialisation occurs, so that the BTS7960B drivers receive a defined stop
  signal rather than floating inputs. The firmware MUST enable the ESP32's
  built-in Task Watchdog Timer (TWDT) with a timeout of 2000 ms (4× the default
  software watchdog timeout) as an independent hardware-level safety net; a TWDT
  expiry triggers a full system reset which returns GPIO outputs to the safe
  LOW state via the reset handler described above.

- **FR-011**: The ESP32 MUST communicate with the ROS2 host over WiFi using
  micro-ROS (UDP transport). The firmware MUST:
  - Subscribe to topic `/wheel_velocities` (message type
    `std_msgs/msg/Float32MultiArray`, array length 4, order FL/FR/RL/RR,
    units rad/s) with QoS: reliability=RELIABLE, durability=VOLATILE,
    history=KEEP_LAST(1).
  - Publish to topic `/firmware_status` (message type
    `std_msgs/msg/String` encoded as JSON until a custom message type is
    defined in the plan phase) with QoS: reliability=BEST_EFFORT,
    durability=VOLATILE, history=KEEP_LAST(1), at minimum 1 Hz.
  - Register as a named node in the ROS2 graph (node name: `esp32_firmware_node`,
    namespace: `/rover`) making the ESP32 a first-class participant in DDS
    discovery.

- **FR-012**: The WiFi connection parameters (SSID, password, ROS2 agent IP
  address and port) MUST be configurable without recompiling the firmware
  (stored in non-volatile memory or a provisioning step).

- **FR-013**: The firmware MUST detect loss of the micro-ROS WiFi session and
  trigger the software watchdog (all motors stop) within the configured
  timeout period.

### Key Entities

- **WheelCommand**: A `std_msgs/msg/Float32MultiArray` message received on
  `/wheel_velocities`. Array length is always 4; index order is FL[0], FR[1],
  RL[2], RR[3]; units are rad/s; values are signed (positive = forward,
  negative = reverse). The `layout` field is not used by the firmware.

- **FirmwareStatus**: A `std_msgs/msg/String` (JSON-encoded) message published
  on `/firmware_status` at minimum 1 Hz. Fields: `commanded_speeds` (array of
  4 floats, rad/s), `watchdog_state` (string: `"active"` | `"timed_out"`),
  `motor_faults` (array of 4 booleans, FL/FR/RL/RR), `uptime_ms` (integer),
  `malformed_msg_count` (integer). A custom message type MAY replace this in
  the plan phase; if so, FR-011 and this entity MUST be updated.

- **MotorChannel**: Represents one physical GB37-520 motor and its BTS7960B
  driver pair. Attributes: wheel position (FL/FR/RL/RR), current commanded
  speed (rad/s), RPWM duty cycle, LPWM duty cycle, active direction, fault flag.

- **WatchdogTimer**: A configurable countdown that resets on each valid command
  receipt; fires a stop-all-motors event on expiry.

## Assumptions

- **Motor**: GB37-520 DC gearmotor (4×). Exact encoder resolution and no-load
  RPM are to be confirmed during plan/calibration phase.
- **Motor driver**: BTS7960B high-current H-bridge (4×, one per motor). Each
  driver is controlled via two PWM signals (RPWM and LPWM) plus enable pins.
  The enable pins are held HIGH by the firmware during normal operation.
- **Communication**: The ESP32 communicates with the ROS2 host PC exclusively
  over WiFi using micro-ROS (UDP transport). There is no USB-serial operational
  data path; USB is used only for firmware flashing and debugging.
- The micro-ROS agent runs on the ROS2 host PC on the same WiFi network as the
  ESP32. The agent IP and port are provisioned into the firmware.
- Speed units received by the firmware are in radians per second (rad/s),
  matching the output of the ROS2 kinematics node. The firmware converts rad/s
  to BTS7960B PWM duty cycle using a calibrated mapping constant.
- Wheel radius is not needed by the firmware; it operates on per-wheel angular
  velocity commands supplied by the host kinematics node.
- The watchdog timeout default value is 500 ms; this is configurable.
- WiFi latency on a local network is assumed to be under 20 ms for normal
  operation. Higher latency will cause watchdog stops; this is acceptable and
  by design.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All four wheels respond to a velocity command within 100 ms of
  message receipt under normal operating conditions.

- **SC-002**: When the command stream is interrupted, all motors stop within the
  configured watchdog timeout period (default 500 ms) with no additional human
  intervention.

- **SC-003**: The firmware correctly executes at least 8 distinct motion patterns
  (forward, backward, strafe-left, strafe-right, rotate-left, rotate-right, and
  two diagonal combinations) without any wheel spinning in the wrong direction.

- **SC-004**: The firmware operates continuously for at least 30 minutes under
  repeated velocity commands without crashing, stalling, or requiring a reset.

- **SC-005**: Status reports are emitted at 1 Hz or faster and contain all
  required fields; a host tool can parse 100 consecutive status frames without
  a single parse error.

- **SC-006**: The micro-ROS topic interface (topic names, message types,
  QoS settings) is fully documented such that an independent developer can
  configure the micro-ROS agent and verify communication without additional
  clarification.

- **SC-007**: WiFi connection loss is detected within the configured watchdog
  timeout and all motors stop automatically, verified by cutting the WiFi
  access point during an active motion command.
