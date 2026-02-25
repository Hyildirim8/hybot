# Research: Docker Runtime for ROS2 Rover Stack

**Feature**: `006-docker-runtime`
**Date**: 2026-02-24
**Status**: Complete — all NEEDS CLARIFICATION items resolved

---

## R-001: ROS2 Humble Base Image

**Decision**: Use `ros:humble-ros-base` as the Dockerfile base image.

**Rationale**:
- `ros:humble-ros-base` is the authoritative upstream image maintained by OSRF
  (the ROS project itself). The `ros:humble-*` Docker Official Images are community
  mirrors that lag behind by a patch cycle.
- `ros-base` variant (~500 MB) includes the DDS stack, `rclcpp`/`rclpy`, and colcon
  tooling. The `desktop` variant adds RViz, Gazebo, and GUI tools (~4 GB) — not
  needed for a headless rover runtime.

**Alternatives considered**:
- `ros:humble-ros-base` (Docker Hub Official mirror) — acceptable but prefer OSRF source.
- `ros:humble-desktop` — rejected; 8× larger, no operational benefit for this stack.

---

## R-002: Docker Network Mode for ROS2 Nodes

**Decision**: `network_mode: host` for **all** Compose services (not only micro-ros-agent).

**Rationale**:
Fast-DDS (the default RMW for ROS2 Humble) uses UDP multicast on `239.255.0.1:7400`
for Simple Discovery Protocol. Docker bridge networks do not forward multicast
between attached containers by default. Results in 10–30 s discovery timeouts or
complete topic discovery failure between containers on the same bridge.

With `network_mode: host`, all containers share the host's network stack. DDS
multicast reaches all participants natively. This is the production-standard pattern
for single-host robot deployments.

**Alternatives considered**:
- Bridge + `FASTRTPS_DEFAULT_PROFILES_FILE` with `<DisableMulticast/>` and explicit
  peer IPs — requires hardcoded container IPs that change on restart; operationally
  fragile, rejected.
- Bridge + CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) — adds an undeclared
  RMW dependency not in the constitution; rejected.
- Bridge + Compose `extra_hosts` for peer resolution — partial workaround only; still
  broken for SHM transport, rejected.

**Pitfall documented**: With host networking, `ROS_DOMAIN_ID=42` (default) is the
only isolation layer separating containerised nodes from any host ROS2 processes.
Ensure no host processes use domain 42.

---

## R-003: micro-ROS Agent Image and Configuration

**Decision**: Use `microros/micro-ros-agent:humble` (pinned tag); run with
`network_mode: host`; command `udp4 --port 8888 --middleware dds --verbose 4`;
restart policy `unless-stopped`.

**Rationale**:
- `microros/micro-ros-agent:humble` is the official upstream image. Pinning to
  `humble` (not `latest`) prevents silent breakage on new ROS distro releases.
- The agent binary takes transport arguments as CLI args — there is no environment
  variable for transport configuration. The `command:` field in Compose provides these.
- `network_mode: host` is required (and consistent with R-002). The ESP32 sends UDP
  packets to the host's WiFi IP. The agent must bind to that same interface and reply
  to the ESP32's ephemeral source port — only possible with host networking.
- `restart: unless-stopped` — the agent process does not crash on ESP32 disconnect
  (it waits 30 s then drops the session). `unless-stopped` restarts only on genuine
  crash, not on `docker compose stop`.

**Health check**:
- `ss -ulnp | grep -q ':8888'` — confirms the UDP socket is bound (agent alive).
- Does NOT confirm ESP32 is connected. That requires a separate tier-2 check:
  `ros2 node list | grep <esp32_node_name>`.

**Known Ubuntu 22.04 pitfalls**:
1. `ufw` may block UDP 8888 — must be opened: `sudo ufw allow 8888/udp`.
2. Fast-DDS multicast on WiFi can flood 2.4 GHz — consider disabling multicast in
   the DDS profile for the micro-ROS domain if congestion observed.
3. WiFi kernel reassembly buffer (`ipfrag_time`, `ipfrag_high_thresh`) may need tuning
   for large ROS messages over lossy WiFi.

---

## R-004: Joystick Device Passthrough (joy_linux)

**Decision**: Joy service runs `privileged: true` with `volumes: - /dev/input:/dev/input`.
Non-root user created in Dockerfile; `group_add: [input]` in Compose for belt-and-
suspenders. Device interface: `/dev/input/js*` (joystick subsystem, not `event*`).

**Rationale**:
- `joy_linux` uses the Linux joystick subsystem (`/dev/input/js*`), not the generic
  event subsystem (`/dev/input/event*`). Device path defaults to `/dev/input/js0`;
  configurable via `~dev` ROS2 parameter.
- Hot-plug support requires bind-mounting `/dev/input` as a volume (not `devices:`).
  The `devices:` Compose field only passes device nodes that exist at container start.
  With a bind-mounted volume, new `js*` nodes created by hot-plug are immediately
  visible inside the container. `joy_linux` implements a 1-second retry loop
  (`open()` on `ENOENT/EIO`) so reconnect is automatic.
- `privileged: true` is needed because `/dev/input/js*` is owned `root:input` mode
  `0660`, and the host `input` group GID varies (commonly 999 on Ubuntu 22.04 but
  not guaranteed). Hard-coding the GID at image build time is fragile.
- `group_add: [input]` is added alongside `privileged: true` as belt-and-suspenders
  for scenarios where Docker propagates group membership checks.

**udev rule for host** (documented in quickstart):
```
KERNEL=="js[0-9]*", SUBSYSTEM=="input", GROUP="input", MODE="0660"
```
Reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

**Scope of privileged exception**: joy service only. All other services remain
non-privileged, running as `rosuser` (UID 1000).

---

## R-005: Dockerfile Pattern (Multi-Stage Build)

**Decision**: Three-stage pattern — `deps` → `build` → `runtime`.

**Rationale**:
- `deps` stage: `COPY` source, run `rosdep install` — cached independently of source changes.
- `build` stage: `colcon build` with `--cmake-args -DCMAKE_BUILD_TYPE=Release`.
  Use `--merge-install` (not `--symlink-install`) for the runtime stage so install/
  paths are self-contained with no symlinks that resolve to the build/ tree.
- `runtime` stage: copy only `/ws/install` from build stage. Omit `build/` and `log/`
  (can triple image size). Result is a minimal runtime image.
- `SHELL ["/bin/bash", "-c"]` required in all stages — `source` does not work with
  the default `/bin/sh`.
- Entrypoint script sources `/opt/ros/humble/setup.bash` and `/ws/install/setup.bash`
  before `exec "$@"`.

---

## R-006: Parameter Injection (Host YAML → Container Nodes)

**Decision**: Single `./config/rover_params.yaml` bind-mounted read-only as
`/config/rover_params.yaml` inside all service containers. Each ROS2 launch file
declares a `params_file` launch argument defaulting to `/config/rover_params.yaml`
and passes it to all nodes via `parameters=[params_file]`.

**Rationale**:
- Single file, single mount, no rebuild required for any parameter change.
- ROS2 `parameters=[<path>]` in `launch_ros.actions.Node` loads all matching
  `ros__parameters` keys (double underscore mandatory) at node startup.
- Wildcard `/**:` top-level key in the YAML allows shared parameters without per-node
  namespace sections.

**Pitfall documented**: YAML keys use `ros__parameters` (double underscore) — single
underscore is silently ignored by the ROS2 parameter loader.

---

## R-007: ROS_DOMAIN_ID Isolation

**Decision**: `ROS_DOMAIN_ID=42` default for all services, declared in `.env` and
overridable per deployment.

**Rationale**:
- Domain ID 42 maps to a UDP port range that does not overlap with host processes
  using the conventional default of 0.
- Safe domain IDs on Linux: 0–101 and 215–232 (avoids ephemeral port range
  32768–60999). 42 is within the safe range.
- Declared in `.env` so operators can override for multi-robot scenarios without
  editing `docker-compose.yaml`.

---

## R-008: rosbag2 Recording

**Decision**: Optional `recorder` service in `docker-compose.yaml`, activated by
`RECORD=true` env var (Compose profile or `.env`). Bags written to `./bags/`
bind-mounted as `/bags` inside the container (read-write). Directory auto-created
by Compose volume definition.

**Rationale**:
- `rosbag2` records to `/bags/<timestamp>/` inside the container; the bind-mount
  persists them to the host after container stop (satisfying SC-004).
- Using a Compose profile (`profiles: [record]`) or an env-gated `restart: "no"`
  with a `command: sh -c 'if [ "$RECORD" = "true" ]; then ...; fi'` keeps the
  base `docker compose up` clean without a separate recorder container appearing.

---

## Summary of All Resolved Clarifications

| ID | Topic | Decision |
|----|-------|----------|
| R-001 | Base image | `ros:humble-ros-base` |
| R-002 | Network mode | `network_mode: host` for all services |
| R-003 | micro-ROS agent | `microros/micro-ros-agent:humble`, `udp4 --port 8888`, `restart: unless-stopped` |
| R-004 | Joystick passthrough | `privileged: true` + `/dev/input` bind-mount; `joy_linux` uses `js*` |
| R-005 | Dockerfile pattern | Three-stage: deps → build → runtime; `--merge-install` |
| R-006 | Parameter injection | Single `./config/rover_params.yaml` bind-mount; `ros__parameters` YAML format |
| R-007 | ROS_DOMAIN_ID | Default 42 in `.env`, overridable |
| R-008 | Bag recording | `RECORD=true` activates recorder service; `./bags/` bind-mount |
