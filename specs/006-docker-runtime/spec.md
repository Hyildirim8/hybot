# Feature Specification: Docker Runtime for ROS2 Rover Stack

**Feature Branch**: `006-docker-runtime`
**Created**: 2026-02-24
**Status**: Draft
**Input**: User description: "Run the complete mecanum rover ROS2 software stack (joy, teleop, kinematics, micro-ROS agent, diagnostics) inside Docker containers so that the system can be deployed on any host without manual ROS2 Humble installation"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - One-Command Full Stack Launch (Priority: P1)

A developer or operator clones the repository on any Linux machine and, without
installing ROS2 Humble or any ROS2 packages manually, starts the entire rover
software stack — joystick driver, teleop node, kinematics node, micro-ROS agent,
and diagnostics — with a single command. All nodes appear in the ROS2 graph and
the rover responds to joystick input within two minutes of running that command.

**Why this priority**: The entire value of containerisation is frictionless
deployment. If the stack cannot be launched in one step without host ROS2
dependencies, the feature has not delivered its core promise. Everything else
(config, rebuild, debugging) is secondary to this story.

**Independent Test**: Start from a fresh machine that has only Docker installed.
Clone the repo, run the single launch command, connect the Logitech F710 joystick
and the ESP32 to the same WiFi network, and confirm within two minutes that all
five services appear healthy and `/cmd_vel` responds to joystick input. No
`apt install ros-humble-*` or `source /opt/ros/humble/setup.bash` should be
required on the host.

**Acceptance Scenarios**:

1. **Given** Docker is installed and the repo is cloned on a host with no ROS2,
   **When** the operator runs the documented single launch command,
   **Then** all five services (joy, teleop, kinematics, micro-ros-agent,
   diagnostics) start and report healthy status within 120 seconds.

2. **Given** the stack is running,
   **When** the operator moves the F710 joystick,
   **Then** `/cmd_vel` messages appear on the ROS2 topic at the expected rate.

3. **Given** the stack is running,
   **When** the operator runs the ROS2 node list command inside the container,
   **Then** all expected nodes are listed, including the ESP32 micro-ROS node.

---

### User Story 2 - Configuration Without Rebuilding (Priority: P2)

An operator needs to change rover parameters — wheel geometry, speed limits,
joystick axis mappings, WiFi credentials for the micro-ROS agent — without
rebuilding the Docker image. They edit a single configuration file on the host
and restart the stack. The new parameters take effect immediately.

**Why this priority**: Hardware calibration (wheel radius, speed limits, WiFi
subnet) changes frequently during development. Requiring an image rebuild for
every parameter change would make iteration impractical and defeats the purpose
of a clean runtime environment.

**Independent Test**: Change the `max_linear_speed` parameter in the host-mounted
config file, restart the stack without rebuilding the image, send a joystick
command at full deflection, and confirm the rover's linear speed cap reflects the
new value.

**Acceptance Scenarios**:

1. **Given** the stack is running with a known `max_linear_speed`,
   **When** the operator edits the host-side config file and restarts the stack,
   **Then** the new `max_linear_speed` is respected without rebuilding the image.

2. **Given** the WiFi subnet or micro-ROS agent address changes,
   **When** the operator updates the config file and restarts,
   **Then** the micro-ROS agent connects to the new address without any image
   rebuild.

3. **Given** the operator has not edited the config file,
   **When** the stack starts,
   **Then** sensible defaults (matching the values defined in the constitution)
   are used automatically.

---

### User Story 3 - Joystick Device Access Inside Container (Priority: P2)

The Logitech F710 joystick is plugged into a USB port on the host. The
containerised joy node detects the joystick and publishes `/joy` messages without
any manual device path configuration, on both first boot and after replug events.

**Why this priority**: USB device passthrough is a known friction point in
Docker. If the joystick is not accessible inside the container, the entire
teleoperation chain is broken. Solving this reliably and documenting it is
essential for operator confidence.

**Independent Test**: Plug the F710 into the host, start the stack, and confirm
that `/joy` messages appear. Then unplug and replug the joystick and confirm
that messages resume without restarting the stack.

**Acceptance Scenarios**:

1. **Given** the F710 is plugged in before the stack starts,
   **When** the stack starts,
   **Then** `/joy` messages appear within 10 seconds, no manual device path
   configuration required.

2. **Given** the F710 is unplugged and replugged while the stack is running,
   **When** the joy node detects the device again,
   **Then** `/joy` messages resume within 15 seconds without restarting the
   container.

3. **Given** no joystick is connected,
   **When** the stack starts,
   **Then** the joy node reports a clear warning in its logs (not a crash), and
   all other nodes start normally.

---

### User Story 4 - ROS2 Bag Recording from Container (Priority: P3)

An operator triggers rosbag2 recording from within the containerised stack.
The bag file is saved to a host-mounted directory so it persists after the
container stops and can be analysed with standard ROS2 tools on the host or
any other machine.

**Why this priority**: Rosbag recording is required by Constitution Principle V
(Observability). Without host-mounted persistence, bag files are lost when the
container stops, making post-run analysis impossible.

**Independent Test**: Start the stack with the record flag set, drive the rover
for 30 seconds, stop the stack, and confirm a valid `.mcap` or `.db3` bag file
exists on the host filesystem containing `/joy`, `/cmd_vel`, `/wheel_velocities`,
and `/diagnostics` topics.

**Acceptance Scenarios**:

1. **Given** the stack is started with recording enabled,
   **When** the stack runs for at least 30 seconds and is then stopped,
   **Then** a valid bag file containing all four required topics exists in the
   host-side output directory.

2. **Given** the operator does not enable recording,
   **When** the stack runs,
   **Then** no bag file is created and no error is logged.

3. **Given** the host output directory does not exist when recording is enabled,
   **When** the stack starts,
   **Then** the directory is created automatically and recording proceeds.

---

### Edge Cases

- What happens if Docker is not installed or is below the minimum required version?
  → The launch script MUST detect this, print a clear error message with the
  minimum required version, and exit before attempting to pull or start any image.
- What happens if the host machine does not have the required joystick device
  permissions (udev rules not set)?
  → The documentation MUST include udev setup instructions; the joy node MUST
  log a permission-denied message (not a silent failure) when device access fails.
- What happens if the micro-ROS agent cannot reach the ESP32 (wrong IP, ESP32 not
  on WiFi yet)?
  → The micro-ROS agent container MUST restart with a backoff policy; the health
  check MUST distinguish between "agent running but ESP32 not connected" and
  "agent crashed", and report accordingly.
- What happens if the host has an incompatible CPU architecture (e.g., ARM vs x86)?
  → The documentation MUST state the supported architectures; the image build MUST
  fail with a clear error on unsupported architectures rather than producing a
  silently broken image.
- What happens if a container image is unavailable (no internet, registry down)?
  → The stack MUST be buildable from source using a documented offline build path;
  the launch script MUST detect a missing image and instruct the operator to build
  locally.
- What happens when the host ROS2 `ROS_DOMAIN_ID` conflicts with the container's
  domain?
  → The domain ID MUST be configurable via the `.env` file (`ROS_DOMAIN_ID=42`
  default); it MUST be documented and isolated from common host defaults (0).
- What happens when `RECORD=true` is passed directly to `docker compose up`
  without using `scripts/launch.sh`?
  → The recorder service will NOT start (Compose activates profiles via
  `COMPOSE_PROFILES`, not `RECORD`). `scripts/launch.sh` MUST be documented as
  the required entrypoint; the quickstart MUST warn against calling bare
  `docker compose up` when recording is desired.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: A single launch command (e.g., `docker compose up`) MUST start all
  required services: joy driver, teleop node, kinematics node, micro-ROS agent,
  and diagnostics aggregator.

- **FR-002**: A custom ROS2 image MUST be built from the official ROS2 Humble
  base image and MUST include all rover ROS2 packages built from the repository
  source: `ecza_teleop`, `ecza_kinematics`, `ecza_hardware_bridge` (including
  the `firmware_diagnostics_node` adapter), and the `ecza_bringup` package (for
  launch files and config). The `joy_linux` package MUST also be present (installed
  via rosdep). The micro-ROS agent service MUST use the upstream official
  `micro-ros-agent` image pulled from its public registry; it MUST NOT be bundled
  into the custom rover image.

- **FR-003**: All runtime parameters (wheel geometry, speed limits, joystick axis
  mappings, micro-ROS agent address and port, ROS domain ID, watchdog timeout)
  MUST be read from a single host-mounted configuration file that can be edited
  without rebuilding the image.

- **FR-004**: The joy service container MUST run in privileged mode to guarantee
  access to `/dev/input` joystick devices without manual device path
  specification. The solution MUST handle the joystick being connected on any
  `/dev/input/js*` or `/dev/input/event*` path. Privileged mode is scoped
  exclusively to the joy service; all other services MUST remain non-privileged.

- **FR-005**: Rosbag2 recording MUST be activatable via the `RECORD=true`
  environment variable (e.g., `RECORD=true docker compose up`). A wrapper script
  `scripts/launch.sh` MUST translate `RECORD=true` into `COMPOSE_PROFILES=record`
  before invoking `docker compose up`, because Docker Compose natively activates
  the `recorder` service via `profiles: [record]` and does not respond to arbitrary
  environment variables. Operators MUST use `scripts/launch.sh` (not bare
  `docker compose up`) to ensure the `RECORD` variable is correctly forwarded.
  Recorded bags MUST be written to a host-mounted output directory (`./bags/`).

- **FR-006**: Each service MUST have a defined restart policy so that transient
  failures (micro-ROS agent losing the ESP32 connection, joystick momentarily
  disconnected) cause an automatic restart with a back-off delay rather than
  leaving the stack in a broken state.

- **FR-007**: The stack MUST expose a two-tier health-check mechanism invocable
  from the host without entering any container:
  - **Tier 1** (fast): reports whether all host-side Compose services are in a
    running state. MUST complete within 5 seconds.
  - **Tier 2** (full): additionally verifies that all expected ROS2 topics
    (`/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics`) are actively
    publishing AND that the ESP32 micro-ROS node appears in `ros2 node list`.
    MUST clearly distinguish between "agent running, ESP32 not yet connected"
    and "agent crashed". MUST complete within 10 seconds.

- **FR-008**: ALL services MUST run with `network_mode: host` so that each
  service shares the host network stack. This is required because Fast-DDS UDP
  multicast (used by all ROS2 DDS discovery) fails silently on Docker bridge
  networks — nodes on a bridge network may never discover one another or may
  experience 10–30 s discovery timeouts. Host networking eliminates NAT and
  ensures the micro-ros-agent can reach the ESP32 via UDP on the local WiFi
  subnet. *(Supersedes clarification Q2 answer; see Clarifications section.)*

- **FR-009**: The custom rover image MUST be built from source using a `docker
  compose build` command. No pre-built image registry is provided or required.
  The build MUST succeed in an environment with no internet access once all base
  image layers and apt packages have been fetched in a prior online `docker
  compose pull` step. The quickstart documentation MUST clearly state that
  `docker compose build` is a prerequisite to `docker compose up`.

- **FR-010**: The `ROS_DOMAIN_ID` MUST be configurable and MUST default to a
  value that does not conflict with common host defaults; it MUST be consistent
  across all services in the stack.

- **FR-011**: All services MUST run as a non-root user. The joy service is the
  sole exception: it MUST run in privileged mode to access `/dev/input` devices,
  but still MUST run as a non-root user within that privileged container. All
  other services MUST be non-privileged.

- **FR-012**: A quickstart section in the documentation MUST cover: Docker
  installation prerequisites, udev joystick permissions, WiFi provisioning for
  the ESP32, config file editing, the launch command, and how to verify the stack
  is healthy.

### Key Entities

- **ComposeStack**: The set of Docker Compose service definitions that together
  constitute the running rover software stack. Attributes: service list
  (joy, teleop, kinematics, micro-ros-agent, diagnostics, optional recorder),
  shared network, shared config volume, shared output volume.

- **RoverConfig**: The single host-side YAML file mounted into all containers.
  Contains all tuneable runtime parameters. No container rebuild is needed to
  change its values.

- **RecordingOutput**: The host-side directory mounted into the recorder service.
  Contains rosbag2 files written during operation. Persists after container stop.

- **HealthCheck**: A host-runnable command or script that performs a two-tier
  check: tier 1 reports Compose service states (fast, no ROS2 dependency);
  tier 2 reports active ROS2 topics and whether the ESP32 micro-ROS node has
  joined the graph (requires the ROS2 graph to be reachable).

## Clarifications

### Session 2026-02-24

- Q: How many Docker images make up the stack — single shared, per-service, or split? → A: Two images: one custom ROS2 image containing all rover nodes (joy, teleop, kinematics, diagnostics, recorder), plus the upstream official `micro-ros-agent` image pulled directly.
- Q: What Docker network mode should the micro-ROS agent use to reach the ESP32 on the local WiFi subnet? → A (original): `network_mode: host` for the micro-ros-agent service only. **SUPERSEDED**: Implementation planning research found that Fast-DDS UDP multicast fails silently on Docker bridge networks regardless of which service initiates discovery. FR-008 has been updated to require `network_mode: host` for ALL services.
- Q: How does the non-root container user gain access to `/dev/input` for the joystick? → A: The joy service container runs with `--privileged` (privileged mode) to guarantee device access; all other services remain non-privileged.
- Q: Where does the custom rover image come from on a fresh machine — registry pull or local build? → A: Build-from-source only; no image registry. Operators run `docker compose build` before first use.
- Q: What does the health check verify — host containers only, or also the ESP32 ROS2 node presence? → A: Two-tier: tier 1 = all host containers running; tier 2 = all expected ROS2 topics active including the ESP32 micro-ROS node.

## Assumptions

- The host operating system is Ubuntu 22.04 LTS (the same OS targeted by ROS2
  Humble). Other Linux distributions may work but are not the primary target.
- Docker Engine (not Docker Desktop) is the container runtime; minimum version
  is Docker 24.0 and Docker Compose v2.20 or later.
- The stack uses two images: one custom rover image (all ROS2 nodes) built from
  this repository, and the upstream official `micro-ros-agent` image. This
  separates rover build concerns from micro-ROS agent versioning.
- ALL containers use `network_mode: host`. Fast-DDS UDP multicast fails silently
  on Docker bridge networks; host networking is required for reliable DDS node
  discovery across all services, not just the micro-ros-agent.
- The joy service container runs in privileged mode to access `/dev/input`
  devices. This is the accepted tradeoff for reliable joystick passthrough;
  the security surface is limited to the single joy service.
- The host machine has an x86_64 (amd64) CPU; ARM support (e.g., Raspberry Pi)
  is out of scope for this feature but must not be architecturally blocked.
- The ESP32 and the host machine are on the same local WiFi network; the
  micro-ROS agent uses UDP transport on the same subnet.
- The Logitech F710 is in D-mode (DirectInput) as required by Constitution
  Principle III; the joy driver inside the container is `joy_linux`.
- Rosbag2 uses the default `.mcap` storage format; no custom serialisation is
  required.
- The custom rover image is built locally from source only; no image registry
  (Docker Hub, GHCR, or otherwise) is used or published to. CI/CD image
  publishing is explicitly out of scope for this feature.
- `ROS_DOMAIN_ID` defaults to 42 inside the container to avoid conflict with
  the most common host default of 0.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Starting from a machine with only Docker installed, the complete
  rover stack is operational (all nodes visible, `/cmd_vel` responsive to
  joystick) within 120 seconds of running `docker compose up` — assuming
  `docker compose build` has already been completed successfully.

- **SC-002**: An operator can change any parameter in the config file and have it
  take effect after a stack restart, without rebuilding the image, in under 60
  seconds.

- **SC-003**: The joystick is detected and `/joy` messages are published without
  any manual device configuration on at least two different host machines.

- **SC-004**: After the stack is stopped, a valid rosbag2 file recorded during
  the session is present on the host filesystem and playable with standard ROS2
  tools.

- **SC-005**: If any single service crashes and restarts, the full stack returns
  to a healthy state automatically within 30 seconds without operator
  intervention.

- **SC-006**: The tier-1 health-check reports the correct container state within
  5 seconds; the tier-2 health-check reports the correct ROS2 topic and node
  status (including ESP32 connectivity) within 10 seconds of being invoked.

- **SC-007**: The complete documentation (quickstart through advanced config) is
  sufficient for a developer unfamiliar with the project to launch the stack
  successfully on a first attempt, as validated by a walkthrough test with a
  new team member.
