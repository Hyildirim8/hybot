# Feature Specification: micro-ROS WiFi Agent & Hardware Integration

**Feature Branch**: `004-hardware-bridge`
**Created**: 2026-02-24
**Updated**: 2026-02-24 — revised from USB-serial bridge to micro-ROS WiFi agent
**Status**: Draft
**Input**: User description: "ROS2 Humble hardware bridge node that connects to the ESP32-S3 firmware over WiFi using micro-ROS; the ESP32 subscribes to /wheel_velocities and publishes firmware status directly as a ROS2 node on the same network"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - ESP32 Appears as a ROS2 Node over WiFi (Priority: P1)

The ESP32 firmware (running micro-ROS) connects to the micro-ROS agent on the
host PC over the local WiFi network. After connection, the ESP32 appears in
`ros2 node list` as a first-class ROS2 node, subscribing to `/wheel_velocities`
directly. No separate bridge or proxy node is required for normal operation.

**Why this priority**: This is the fundamental integration story. All other
features assume the ESP32 participates in the ROS2 graph. If this story does
not work, no wheel velocity command can reach the hardware.

**Independent Test**: Start the micro-ROS agent on the host PC. Power on the
ESP32 on the same WiFi network. Within 10 seconds, run `ros2 node list` and
verify the ESP32 micro-ROS node appears. Run `ros2 topic list` and verify
`/wheel_velocities` and the firmware status topic are present.

**Acceptance Scenarios**:

1. **Given** the micro-ROS agent is running on the host PC and the ESP32 is
   powered on the same WiFi network,
   **When** the micro-ROS session is established,
   **Then** the ESP32 node appears in `ros2 node list` within 10 seconds.

2. **Given** the ESP32 micro-ROS node is connected,
   **When** a `/wheel_velocities` message is published on the ROS2 host,
   **Then** the ESP32 receives and acts on it within the command latency budget
   (motor response within 100 ms of publication).

3. **Given** the ESP32 micro-ROS node is connected,
   **When** the firmware emits a status report,
   **Then** a corresponding ROS2 message appears on the firmware status topic
   and is visible via `ros2 topic echo`.

---

### User Story 2 - micro-ROS Agent Lifecycle Management (Priority: P2)

The micro-ROS agent process is managed as a ROS2-compatible process that can be
started, monitored, and stopped as part of the system launch. The host PC
automatically starts the agent on launch and the ESP32 reconnects if the WiFi
link or agent is temporarily interrupted.

**Why this priority**: Robust agent lifecycle management is the operational
prerequisite for reliable hardware test sessions. Without it, every WiFi glitch
requires manual intervention.

**Independent Test**: Start the system launch (including the agent). Kill the
agent process manually. Verify the ESP32 detects the session loss, stops motors
via watchdog, and — when the agent is restarted — reconnects and resumes normal
operation without any additional user action.

**Acceptance Scenarios**:

1. **Given** the micro-ROS agent is launched as part of the system,
   **When** the agent process is stopped and restarted,
   **Then** the ESP32 automatically reconnects within the configured retry window
   and `/wheel_velocities` commands flow again.

2. **Given** the micro-ROS session is lost (agent stopped or WiFi interrupted),
   **When** the ESP32 detects session loss,
   **Then** the firmware watchdog triggers and all motors stop within the
   configured timeout (default 500 ms).

3. **Given** the micro-ROS agent is included in the ROS2 launch file,
   **When** the launch file is executed,
   **Then** the agent starts with the correct transport parameters (UDP, host IP,
   port) loaded from the system config YAML.

---

### User Story 3 - Firmware Diagnostics via ROS2 Topic (Priority: P3)

The ESP32 micro-ROS node publishes its operational status (commanded wheel
speeds, watchdog state, BTS7960B fault flags, WiFi signal quality) as a ROS2
topic. This data is consumable by `rqt_robot_monitor` or any standard ROS2
diagnostic tool without any additional translation node.

**Why this priority**: Because the ESP32 is a full ROS2 participant, diagnostics
flow natively without a proxy. This story validates that the diagnostics path is
complete and usable, fulfilling Constitution Principle V.

**Independent Test**: With the ESP32 connected and motors running, subscribe to
the firmware diagnostics topic. Observe that status messages arrive at ≥1 Hz
and contain all expected fields. Trigger a fault (e.g., disable a motor driver
enable pin) and verify the fault flag appears in the topic.

**Acceptance Scenarios**:

1. **Given** the ESP32 micro-ROS node is connected,
   **When** a subscriber is attached to the firmware status topic,
   **Then** messages arrive at a rate of at least 1 Hz.

2. **Given** a BTS7960B fault condition is present (e.g., over-current),
   **When** the firmware detects it,
   **Then** a fault flag for the affected motor channel appears in the next
   status message.

3. **Given** the micro-ROS session is healthy,
   **When** the `/diagnostics` aggregator is running on the host,
   **Then** the ESP32 diagnostic data appears in `rqt_robot_monitor` via the
   `firmware_diagnostics_node` adapter included in this package.

---

### Edge Cases

- What happens if the WiFi network is unavailable when the ESP32 powers on?
  → The firmware MUST retry WiFi connection indefinitely; motors MUST remain
  stopped until a micro-ROS session is established.
- What happens if multiple ESP32 boards are on the same network (future expansion)?
  → micro-ROS node names MUST include a unique identifier (e.g., derived from
  MAC address) to prevent namespace collision. Single-board is the current scope.
- What happens if the micro-ROS agent and the ESP32 are on different subnets?
  → UDP multicast or direct-address configuration MUST be documented. Cross-subnet
  operation is out of scope for this feature but MUST NOT be architecturally blocked.
- What happens if WiFi latency spikes above the watchdog threshold during operation?
  → Watchdog triggers and motors stop. This is acceptable and by design. The
  quickstart guide MUST document the recommended WiFi environment requirements.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: A micro-ROS agent MUST be run on the ROS2 host PC, configured to
  accept UDP connections from the ESP32 on a documented default port.

- **FR-002**: The micro-ROS agent MUST be startable as part of the system launch
  file, with transport parameters (protocol, port, allowed client addresses)
  loaded from the system config YAML.

- **FR-003**: The ESP32 micro-ROS node MUST subscribe to `/wheel_velocities`
  directly (not via any proxy), making it a first-class consumer of that topic.

- **FR-004**: A host-side `firmware_diagnostics_node` (part of the
  `ecza_hardware_bridge` package) MUST subscribe to `/firmware_status`
  (`std_msgs/msg/String` JSON, published by the ESP32 micro-ROS node) and
  re-publish the decoded fields as `diagnostic_msgs/DiagnosticArray` on
  `/diagnostics` at a minimum rate of 1 Hz. This adapter node is the sole
  host-side component responsible for bridging the firmware JSON status into
  the standard ROS2 diagnostics infrastructure. The ESP32 micro-ROS node
  publishes only to `/firmware_status`; it does NOT publish DiagnosticArray
  natively (micro-ROS has no stable DiagnosticArray support).

- **FR-005**: The system MUST support automatic reconnection: if the micro-ROS
  session is interrupted, the ESP32 MUST retry connection and resume normal
  operation without requiring a firmware restart.

- **FR-006**: The micro-ROS agent startup parameters (UDP port, bind address)
  MUST be configurable from the system config YAML without recompiling any code.

- **FR-007**: The firmware MUST detect micro-ROS session loss within the
  configured watchdog timeout and stop all motors.

- **FR-008**: The host-side launch configuration MUST document the required
  WiFi environment (same subnet, SSID, recommended channel/band) in the
  quickstart guide.

- **FR-009**: The ESP32 micro-ROS node name and topic namespace MUST be
  configurable to support future multi-board scenarios.

### Key Entities

- **micro-ROS Agent**: The host-side process that bridges micro-ROS clients
  (ESP32) to the full DDS-based ROS2 graph. Attributes: transport (UDP),
  port, bind address.

- **ESP32 micro-ROS Node**: The ROS2-visible node running on the ESP32.
  Subscribes to `/wheel_velocities`; publishes `std_msgs/String` JSON to
  `/firmware_status`. Does NOT publish DiagnosticArray natively.

- **FirmwareDiagnosticsNode**: A host-side ROS2 node in `ecza_hardware_bridge`
  that subscribes to `/firmware_status`, deserialises the JSON, and publishes
  `diagnostic_msgs/DiagnosticArray` to `/diagnostics` at pass-through rate.
  This is the adapter that makes ESP32 status visible to `rqt_robot_monitor`.

- **WiFiSession**: The network connection between ESP32 and host. Attributes:
  SSID, IP addresses, latency, connection state.

- **DiagnosticsPublisher**: The micro-ROS publisher on the ESP32 that converts
  internal firmware state into `diagnostic_msgs/DiagnosticArray` messages.

## Assumptions

- The micro-ROS agent is the standard `micro-ros-agent` from the micro-ROS
  project, available as a ROS2 Humble package or Docker image.
- Transport protocol: UDP over WiFi (micro-ROS UDP transport). TCP transport
  is not required for this feature.
- The host PC and the ESP32 are on the same WiFi subnet. Cross-subnet routing
  is out of scope.
- The micro-ROS agent default port is 8888 (UDP); configurable.
- The ESP32 WiFi credentials and agent IP/port are provisioned into the firmware
  at build time or via a one-time configuration step. Dynamic provisioning
  (e.g., WiFi portal) is out of scope.
- Only one ESP32 is in the network for this feature. Multi-board is out of scope.
- The micro-ROS agent is included in the ROS2 bringup launch file (feature 005).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The ESP32 micro-ROS node appears in `ros2 node list` within 10
  seconds of the agent starting on a network where the ESP32 is already powered.

- **SC-002**: A `/wheel_velocities` message published on the host reaches the
  ESP32 and produces a motor response within 150 ms end-to-end (including WiFi
  latency), measured on a local WiFi network.

- **SC-003**: After a micro-ROS session interruption (agent restart), the ESP32
  reconnects and resumes normal operation within 10 seconds with no manual
  intervention.

- **SC-004**: Firmware diagnostic messages appear on `/diagnostics` at 1 Hz or
  faster via the `firmware_diagnostics_node` adapter and are visible in
  `rqt_robot_monitor` with a working `diagnostic_aggregator` running.

- **SC-005**: The full WiFi-connected stack operates continuously for 30 minutes
  under active teleoperation without session drops requiring manual recovery.
