# Feature Specification: Full System Launch & Integration

**Feature Branch**: `005-system-launch`
**Created**: 2026-02-24
**Status**: Draft
**Input**: User description: "ROS2 Humble launch configuration that starts the complete mecanum rover teleoperation stack: joy node, joystick teleop node, mecanum kinematics node, and hardware bridge node in a single launch file with a single YAML config"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - One-Command Full Stack Startup (Priority: P1)

An operator sits down at the development PC, ensures the F710 controller is
connected, and runs a single launch command. The launch starts the micro-ROS
agent, all ROS2 nodes, and waits for the ESP32 (already powered on the WiFi
network) to connect. Within seconds, all nodes are running, the controller is
active, and moving the sticks drives the rover. No manual node-by-node startup
or WiFi/network configuration is required.

**Why this priority**: Until a single-command launch exists, every hardware test
session requires manually starting four separate nodes in the right order. This
story is the integration milestone that makes the system usable as a product, not
just a collection of components.

**Independent Test**: From a clean terminal with no nodes running, execute the
launch command (with the ESP32 powered on the same WiFi network). Within 10
seconds, verify via `ros2 node list` that all expected nodes are running —
including the ESP32 micro-ROS node — and verify that pushing a stick produces
`/cmd_vel` messages without any additional setup.

**Acceptance Scenarios**:

1. **Given** the F710 is connected in D-mode and the ESP32 is powered on the
   same WiFi network,
   **When** the launch command is executed,
   **Then** all five components (`joy`, `teleop`, `kinematics`, `micro-ros-agent`,
   and the ESP32 micro-ROS node) appear in `ros2 node list` within 15 seconds.

2. **Given** the system is fully launched,
   **When** the left stick is pushed forward,
   **Then** `/cmd_vel`, `/wheel_velocities`, and micro-ROS messages to the ESP32 are
   all observable in sequence, confirming the full data pipeline is active.

3. **Given** the system is fully launched,
   **When** one node crashes,
   **Then** the launch system logs the failure and — depending on the configured
   respawn policy — either attempts to restart the node or shuts down the full
   stack cleanly.

---

### User Story 2 - Single Config File for All Parameters (Priority: P2)

All tuneable parameters for the entire system (geometry, speed limits, micro-ROS
agent port, deadzone, axis mappings) are declared in a single YAML file that the operator can
edit before launch. No parameter is split across multiple files. Changing a value
in this file and relaunching is all that is needed to reconfigure the system.

**Why this priority**: During hardware testing, operators frequently need to adjust
parameters (speed limits for safety, micro-ROS agent port if it changes,
geometry after measuring the wheel radius). A single config file eliminates
confusion and reduces the risk of editing the wrong file.

**Independent Test**: Edit the single config YAML to change `max_linear_speed` to
a new value. Relaunch the system. Verify that the teleop node logs the new value
at startup and that the maximum speed of the rover corresponds to the new value.

**Acceptance Scenarios**:

1. **Given** a single `rover_params.yaml` config file exists,
   **When** a parameter value is changed and the system is relaunched,
   **Then** the updated value is picked up by the relevant node and logged at INFO
   level on startup.

2. **Given** the config file is absent or missing required fields,
   **When** the launch command is executed,
   **Then** the launch system logs an ERROR identifying the missing file or field
   and does not start the stack in an undefined state.

---

### User Story 3 - rosbag2 Recording in One Command (Priority: P3)

The launch configuration includes an optional flag or separate launch variant that
simultaneously starts the full stack and a `rosbag2` recording of the key topics
(`/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics`). This allows hardware
test sessions to be recorded for post-hoc analysis with zero extra setup.

**Why this priority**: Constitution Principle V requires rosbag2 support. Wrapping
recording into the launch configuration makes it frictionless — operators do not
need to remember to start a separate recording process.

**Independent Test**: Launch the system with the recording flag enabled. Drive the
rover for 30 seconds. Stop the launch. Verify a rosbag2 directory exists containing
all four topic streams with non-zero message counts.

**Acceptance Scenarios**:

1. **Given** the system is launched with recording enabled,
   **When** the rover is driven for any duration,
   **Then** a rosbag2 bag is written to the configured output directory.

2. **Given** the bag was recorded,
   **When** it is played back with `ros2 bag play`,
   **Then** all four recorded topics publish messages at the rates they were
   originally recorded.

3. **Given** the system is launched without the recording flag,
   **When** the system runs,
   **Then** no bag files are created (recording is opt-in, not default).

---

### Edge Cases

- What happens if the F710 is not connected when the launch command is run?
  → The `joy` node MUST log a WARN and wait for the device to appear. The rest
  of the stack MUST still start.
- What happens if the ESP32 is not on the WiFi network when the launch command is run?
  → The micro-ROS agent MUST start and wait for the ESP32 to connect. All other
  ROS2 nodes MUST start normally. The system becomes fully operational when the
  ESP32 joins the network.
- What happens if the config YAML contains a typo in a parameter name?
  → ROS2 parameter loading will silently ignore unrecognised keys; the node will
  use its default. The spec requires each node to log its effective parameter
  values at startup so mismatches are visible.
- What happens when the system is launched on a machine where the workspace is
  not sourced?
  → The launch command will fail with a ROS2 package-not-found error. The quickstart
  guide MUST document the workspace sourcing step.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: A single ROS2 launch file MUST start all five components: `joy`
  (or `joy_linux`), teleop node, kinematics node, micro-ROS agent, and a
  `diagnostic_aggregator` node (required for Constitution Principle V
  observability). A `ros2 topic relay` is NOT required.

- **FR-002**: The launch file MUST load all node parameters from a single YAML
  configuration file named `rover_params.yaml` located in the launch package's
  `config/` directory.

- **FR-003**: The config YAML (`rover_params.yaml`) MUST contain, at minimum:
  - Rover geometry parameters (`wheel_separation_width`, `wheel_base`, `wheel_radius`)
  - Speed limits (`max_linear_speed`, `max_angular_speed`)
  - Deadzone (`joy_deadzone`)
  - Axis mappings for the F710 in D-mode (`axis_linear_x`, `axis_linear_y`, `axis_angular_z`)
  - Enable button index (`enable_button`)
  - micro-ROS agent transport parameters (`agent_port`, `agent_bind_address`)
  - `joy_watchdog_timeout_ms`: timeout (ms) after which the teleop node publishes
    a zero Twist when no `/joy` message is received (default: 500 ms; feature 003)
  - `firmware_watchdog_timeout_ms`: timeout (ms) that the ESP32 firmware uses to
    stop all motors when no `/wheel_velocities` command is received (default: 500 ms;
    feature 001). This value is informational in the host config; the firmware reads
    it from NVS. Include it here for operator visibility and documentation.

  These two watchdog keys MUST be named differently to prevent confusion; a single
  `watchdog_timeout_ms` key is insufficient and MUST NOT be used.

- **FR-004**: The launch file MUST support a launch argument `record:=true` that
  additionally starts a `rosbag2` recording of `/joy`, `/cmd_vel`,
  `/wheel_velocities`, and `/diagnostics`.

- **FR-005**: The launch file MUST define a respawn policy for each node (at
  minimum, configurable via a launch argument).

- **FR-006**: If the config file is not found at the expected path, the launch
  MUST fail with a clear error message and MUST NOT start nodes with undefined
  parameters.

- **FR-007**: A quickstart guide MUST be provided documenting:
  - Prerequisites (ROS2 Humble, micro-ROS agent, workspace build steps)
  - F710 D-mode setup
  - WiFi network setup (SSID, provisioning ESP32 credentials and agent IP)
  - The launch command and all available launch arguments
  - Recommended WiFi environment (same subnet, 2.4 GHz band recommended for range)

- **FR-008**: The launch package MUST declare all constituent packages as
  dependencies in its `package.xml`.

### Key Entities

- **SystemLaunchFile**: The primary ROS2 launch file that composes all five
  components (joy, teleop, kinematics, micro-ROS agent, optional diagnostics
  aggregator) with their parameter files.

- **SystemConfig**: The single YAML file (`rover_params.yaml`) containing all
  runtime parameters for the full stack, including micro-ROS agent transport
  parameters. Used by both native launch (feature 005) and Docker runtime
  (feature 006); editing this file is the only action required to reconfigure
  the system.

- **QuickstartGuide**: Documentation (`quickstart.md` or `README.md`) covering
  the complete setup and launch procedure including WiFi provisioning.

## Assumptions

- All ROS2 packages (joy, teleop, kinematics) are built into the same ROS2
  workspace and can be composed in a single launch file.
- The micro-ROS agent (`micro-ros-agent`) is installed on the host PC as a
  ROS2 Humble package or run via the official Docker image.
- The ESP32 is pre-provisioned with the WiFi SSID, password, and host PC IP
  address before the first launch. Re-provisioning requires a firmware update
  or a separate provisioning tool (out of scope for this spec).
- The launch package is named `ecza_bringup` or similar; the exact name is
  decided at plan time.
- `ros2 bag record` (rosbag2) is available as part of the standard ROS2 Humble
  installation.
- The quickstart guide is a Markdown file versioned in the repository.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A user following the quickstart guide from scratch (fresh ROS2
  Humble install, no prior project knowledge) can drive the rover within 15
  minutes of first reading.

- **SC-002**: All ROS2 host nodes are running and `/cmd_vel` is responding to
  stick input within 10 seconds of executing the launch command on a prepared
  machine (ESP32 already on the network and pre-provisioned).

- **SC-003**: Changing any parameter in `rover_params.yaml` and relaunching
  takes no more than 30 seconds and produces the expected changed behaviour without
  editing any source file.

- **SC-004**: A 5-minute rosbag2 recording launched with `record:=true` contains
  all four expected topics and can be fully replayed without errors.

- **SC-005**: The full stack runs continuously for 30 minutes with the rover
  being actively driven without any node crash or required manual restart.
