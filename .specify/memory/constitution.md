<!--
SYNC IMPACT REPORT
==================
Version change: 1.0.0 → 1.1.0  (MINOR: new hardware section content, Principle IV revised)
Modified principles:
  - IV. Hardware Abstraction Layer — updated to reflect WiFi micro-ROS replacing USB-serial
Added hardware facts:
  - Motors: GB37-520 DC gearmotor
  - Drivers: BTS7960B H-bridge (dual PWM RPWM/LPWM)
  - Communication: WiFi micro-ROS UDP (USB is flashing-only)
Specs updated by this amendment:
  ✅ specs/001-esp32-firmware/spec.md — BTS7960B dual-PWM, WiFi micro-ROS, GB37-520
  ✅ specs/004-hardware-bridge/spec.md — rewritten as micro-ROS WiFi agent spec
  ✅ specs/005-system-launch/spec.md — USB cable removed, micro-ROS agent added
Specs NOT requiring changes:
  ✅ specs/002-mecanum-kinematics/spec.md — unaffected (publishes /wheel_velocities, agnostic to transport)
  ✅ specs/003-joystick-teleop/spec.md — unaffected (publishes /cmd_vel, agnostic to transport)
Follow-up TODOs:
  - TODO(WHEEL_RADIUS): Exact wheel radius not provided; must be measured and declared in config/
  - TODO(GB37_SPEED_MAP): GB37-520 RPM-to-rad/s and rad/s-to-PWM calibration needed before hardware tests
-->

# Ecza Robotu Constitution

## Core Principles

### I. ROS2-First Architecture

Every subsystem MUST be implemented as a ROS2 Humble node or package.
Communication between subsystems MUST use ROS2 topics, services, or actions —
no direct inter-process calls that bypass the ROS2 middleware (rmw).
Each package MUST be independently buildable with `colcon build --packages-select <pkg>`.
Launch files MUST be provided for every runnable configuration.

**Rationale**: ROS2 Humble is the LTS middleware backbone of the project. Strict
adherence ensures portability, tooling compatibility (rviz2, rqt, rosbag2), and
long-term maintainability across hardware revisions.

### II. Mecanum Kinematics Correctness (NON-NEGOTIABLE)

The rover uses 4 mecanum wheels arranged in an X-drive configuration.
Physical dimensions MUST be treated as named parameters — never hardcoded inline:

- `wheel_separation_width`: 0.26 m (centre-to-centre distance between left and right
  wheels, measured on the front axle)
- `wheel_base`: 0.38 m (centre-to-centre distance between front and rear axles)
- `wheel_radius`: TODO(WHEEL_RADIUS) — must be measured and declared before hardware tests

The kinematic model MUST correctly compute per-wheel velocities for holonomic motion
(forward/backward, lateral strafe, rotation, and compound combinations).
All motion commands MUST originate from a `geometry_msgs/Twist` message on `/cmd_vel`.
Hardware-specific wheel velocity commands MUST be published on a separate topic
(e.g., `/wheel_velocities`) and MUST NOT mix kinematics with hardware driver logic.

**Rationale**: Incorrect wheel geometry constants silently produce drifting motion.
Separating kinematics from hardware drivers ensures each layer can be tested and
swapped independently.

### III. Joystick Input via joy / joy_linux

All human input MUST enter the system through the `joy` or `joy_linux` ROS2 package
producing `sensor_msgs/Joy` messages on `/joy`.
A dedicated `teleop` node MUST subscribe to `/joy` and translate axis/button mappings
to `/cmd_vel` Twist messages.
The Logitech F710 controller MUST be configured in **D-mode** (DirectInput) for
reliable Linux HID recognition.
Axis and button indices MUST be declared as named ROS2 parameters — no magic numbers
in source code.
A `joy_deadzone` parameter MUST be provided to suppress stick noise.

**Rationale**: Standardising on `sensor_msgs/Joy` decouples the physical joystick
from the rest of the system, enabling simulation, key remapping, and controller
substitution without code changes.

### IV. Hardware Abstraction Layer

All hardware-specific code (BTS7960B PWM control, GB37-520 motor management,
WiFi/micro-ROS session handling) MUST reside in the ESP32 firmware layer.
No kinematics, navigation, or teleop ROS2 node MUST contain any hardware-specific
logic or know about the physical motor driver interface.
The ESP32 communicates with the ROS2 graph exclusively via WiFi using micro-ROS;
USB-serial MUST NOT be used as an operational data channel.
A `micro-ros-agent` process MUST be the sole host-side component that interfaces
with the ESP32; no custom bridge node is required.

**Rationale**: Hardware abstraction enables simulation (mock micro-ROS nodes) and
simplifies hardware swap-outs without touching higher-level ROS2 logic. WiFi
micro-ROS makes the ESP32 a first-class ROS2 graph participant, eliminating the
need for a serial proxy/bridge node.

### V. Observability & Diagnostics

Every node MUST log its name and key parameter values at INFO level on startup.
Every node MUST log at WARN or ERROR on anomalous conditions.
Nodes MUST publish diagnostic information via `diagnostic_msgs/DiagnosticArray`
on `/diagnostics` where feasible.
`rosbag2` recording of `/cmd_vel`, `/joy`, `/wheel_velocities`, and `/diagnostics`
MUST be supported and documented in the quickstart guide.

**Rationale**: Debugging physical robot behaviour without observability is
impractical. Structured diagnostics allow rqt_robot_monitor and post-hoc
bag analysis after hardware test sessions.

### VI. Simplicity & Incremental Delivery

Features MUST be built in the smallest independently testable increments.
Navigation stacks, SLAM, or autonomy layers MUST NOT be introduced until manual
teleoperation is proven stable through hardware tests.
YAGNI applies: only implement what is required for the current feature's
acceptance criteria.
Complex dependencies MUST be justified in a "Complexity Tracking" section
before introduction.

**Rationale**: Rover projects accumulate complexity rapidly. Incremental delivery
keeps the hardware integration surface small and failures easy to localise.

## Hardware & Platform Constraints

- **OS**: Ubuntu 22.04 LTS
- **ROS distribution**: ROS2 Humble Hawksbill (supported until May 2027)
- **Joystick**: Logitech F710 wireless gamepad — toggle switch MUST be set to **D** (DirectInput)
- **Rover geometry**:
  - Front axle width (left–right wheel centre distance): **260 mm**
  - Wheelbase (front–rear axle centre distance): **380 mm**
  - Wheel type: 4 × mecanum (2 left-hand, 2 right-hand roller orientation)
  - Wheel radius: TODO(WHEEL_RADIUS) — measure and update `config/rover_params.yaml`
- **Motors**: 4 × GB37-520 DC gearmotor (one per wheel)
- **Motor drivers**: 4 × BTS7960B high-current H-bridge (one per motor)
  - Control interface: dual PWM (RPWM / LPWM) + enable pins
  - Forward: LPWM driven, RPWM = 0; Reverse: RPWM driven, LPWM = 0
  - MUST NOT drive RPWM and LPWM simultaneously non-zero (shoot-through)
- **ESP32 ↔ ROS2 communication**: WiFi, micro-ROS UDP transport
  - The ESP32-S3-WROOM-1 runs micro-ROS and joins the ROS2 graph as a first-class node
  - A `micro-ros-agent` process runs on the host PC (same WiFi subnet)
  - USB is used only for firmware flashing and debugging; NOT for operational data
- **Build system**: `colcon` with `ament_cmake` (C++) or `ament_python` (Python) per package
- **Languages**: Python 3.10 preferred for rapid iteration; C++17 for performance-critical
  nodes (kinematics)
- **Testing**: `pytest` (Python), `ament_cmake_gtest` (C++), `launch_testing` for integration
- **Simulation**: Gazebo (optional — not a hard dependency for MVP teleoperation)
- **Parameter files**: YAML, versioned under `config/` within each package
- **Launch files**: versioned under `launch/` within each package

## Development Workflow

1. Every new capability starts with a feature spec (`/speckit.specify`) mapping to one
   or more ROS2 packages or nodes.
2. Constitution Check MUST be performed at plan time and again after design.
3. Packages MUST pass `colcon test --packages-select <pkg>` before merge to `main`.
4. Hardware test sessions MUST be documented with rosbag2 recordings referenced in the PR.
5. All ROS2 dependencies MUST be declared in `package.xml`; undeclared runtime dependencies
   constitute a Constitution violation.
6. Parameter values MUST NOT be hardcoded in source — use parameter files under `config/`.

## Governance

This constitution supersedes all other development practices for the ecza-robotu project.
Amendments require:

1. A written rationale documenting what changes and why.
2. A version increment per semantic versioning rules:
   - **MAJOR**: Backward-incompatible principle removal or redefinition.
   - **MINOR**: New principle or section added, or materially expanded guidance.
   - **PATCH**: Clarifications, wording fixes, non-semantic refinements.
3. Consistency propagation: all `.specify/templates/` files and `.github/agents/` files
   MUST be reviewed and updated if impacted by the amendment.
4. All PRs and plan reviews MUST include a "Constitution Check" section verifying
   compliance with the six Core Principles.
5. Violations MAY be accepted only when justified in a "Complexity Tracking" section
   of the implementation plan and approved by the project owner.

For runtime development guidance refer to `.github/agents/speckit.*.agent.md` and
`.specify/templates/` for workflow templates.

**Version**: 1.1.0 | **Ratified**: 2026-02-24 | **Last Amended**: 2026-02-24
