# Data Model: Docker Runtime for ROS2 Rover Stack

**Feature**: `006-docker-runtime`
**Date**: 2026-02-24

---

## Entities

### ComposeStack

Represents the complete set of Docker Compose service definitions that together
constitute the running rover software stack.

| Attribute | Type | Value / Constraint |
|-----------|------|--------------------|
| `name` | string | `ecza-robotu` (Compose project name) |
| `services` | list\<Service\> | joy, teleop, kinematics, micro-ros-agent, diagnostics, recorder |
| `networks` | — | None defined (all services use `network_mode: host`) |
| `volumes` | list\<Volume\> | config-volume (read-only), bags-volume (read-write) |
| `env_file` | path | `.env` at repo root |

---

### Service

One Docker Compose service. Each service maps to one running container.

| Attribute | Type | Value / Constraint |
|-----------|------|--------------------|
| `name` | string | one of: joy, teleop, kinematics, micro-ros-agent, diagnostics, recorder |
| `image` | string | `ecza-robotu:runtime` (rover image) or `microros/micro-ros-agent:humble` |
| `network_mode` | string | `host` (all services) |
| `restart` | string | `unless-stopped` (all services); recorder is `no` when `RECORD!=true` |
| `privileged` | bool | `true` for joy service only; `false` for all others |
| `user` | string | `rosuser` (UID 1000) for all services |
| `environment` | map | At minimum `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` |
| `command` | string | ROS2 launch invocation or agent CLI |
| `healthcheck` | HealthCheck | See HealthCheck entity |
| `depends_on` | list\<string\> | teleop depends on joy; kinematics none; agent none |

**Service instances**:

| Service name | Image | Command pattern | Privileged |
|---|---|---|---|
| `joy` | `ecza-robotu:runtime` | `ros2 launch rover_bringup joy.launch.py params_file:=/config/rover_params.yaml` | ✅ yes |
| `teleop` | `ecza-robotu:runtime` | `ros2 launch rover_bringup teleop.launch.py params_file:=/config/rover_params.yaml` | ❌ no |
| `kinematics` | `ecza-robotu:runtime` | `ros2 launch rover_bringup kinematics.launch.py params_file:=/config/rover_params.yaml` | ❌ no |
| `micro-ros-agent` | `microros/micro-ros-agent:humble` | `udp4 --port 8888 --middleware dds --verbose 4` | ❌ no |
| `diagnostics` | `ecza-robotu:runtime` | `ros2 launch rover_bringup diagnostics.launch.py params_file:=/config/rover_params.yaml` | ❌ no |
| `recorder` | `ecza-robotu:runtime` | `ros2 bag record -o /bags/session /joy /cmd_vel /wheel_velocities /diagnostics` | ❌ no |

---

### RoverConfig

The single host-side YAML file mounted read-only into all containers at `/config/rover_params.yaml`.

| Parameter group | Key | Default | Description |
|---|---|---|---|
| Geometry | `wheel_separation_width` | `0.26` | metres; left–right wheel centre distance |
| Geometry | `wheel_base` | `0.38` | metres; front–rear axle distance |
| Geometry | `wheel_radius` | `TODO` | metres; must be measured before hardware tests |
| Speed limits | `max_linear_speed` | `0.5` | m/s |
| Speed limits | `max_angular_speed` | `1.0` | rad/s |
| Joystick | `joy_deadzone` | `0.05` | 0–1 normalised |
| Joystick | `axis_linear_x` | `1` | F710 left stick vertical |
| Joystick | `axis_linear_y` | `0` | F710 left stick horizontal |
| Joystick | `axis_angular_z` | `3` | F710 right stick horizontal |
| Joystick | `enable_button` | `5` | F710 right bumper (dead-man switch) |
| micro-ROS | `agent_port` | `8888` | UDP port micro-ros-agent listens on |
| Watchdog | `watchdog_timeout_ms` | `500` | ms; firmware watchdog timeout |

**YAML structure** (double underscore keys are mandatory):
```yaml
/**:
  ros__parameters:
    wheel_separation_width: 0.26
    wheel_base: 0.38
    wheel_radius: 0.05   # TODO: measure and update
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    joy_deadzone: 0.05
    axis_linear_x: 1
    axis_linear_y: 0
    axis_angular_z: 3
    enable_button: 5
    watchdog_timeout_ms: 500
```

**Validation rules**:
- `wheel_separation_width`, `wheel_base`, `wheel_radius` MUST be positive non-zero floats.
- `joy_deadzone` MUST be in range [0, 1).
- `max_linear_speed` and `max_angular_speed` MUST be positive.
- Missing file at container start: each node logs ERROR and exits (Principle V).

---

### RecordingOutput

The host-side directory bind-mounted as `/bags` inside the recorder service.

| Attribute | Type | Constraint |
|-----------|------|-----------|
| `host_path` | path | `./bags/` relative to repo root; auto-created by Compose |
| `container_path` | path | `/bags` |
| `bag_format` | string | `mcap` (rosbag2 default for Humble) |
| `topics` | list | `/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics` |
| `session_dir_pattern` | string | `session_<ISO8601-timestamp>` (auto-named by rosbag2) |
| `persistence` | — | Survives container stop; never cleaned by the stack |

---

### HealthCheck

A two-tier verification mechanism invocable from the host without entering containers.

#### Tier 1 — Container State (fast, ≤5 s)

| Attribute | Value |
|-----------|-------|
| Mechanism | `docker compose ps` or `docker inspect` |
| What it checks | Each service container is in `running` state |
| Invocation | `docker compose ps --format json \| jq` or the helper script `docker/healthcheck.sh --tier1` |
| Output | Per-service: `running` / `restarting` / `exited` |

#### Tier 2 — ROS2 Graph State (full, ≤10 s)

| Attribute | Value |
|-----------|-------|
| Mechanism | `docker exec <kinematics-container> ros2 topic list` + `ros2 node list` |
| Topics verified | `/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics` all present and active |
| ESP32 check | ESP32 micro-ROS node appears in `ros2 node list` |
| ESP32 state | Reported as `esp32_connected: true/false` — NOT a failure condition (ESP32 may be powering on) |
| Invocation | `docker/healthcheck.sh --tier2` |

**State distinction**:
| Condition | Tier 1 | Tier 2 | Meaning |
|---|---|---|---|
| All services running, ESP32 connected | ✅ | ✅ | Fully operational |
| All services running, ESP32 not yet joined | ✅ | `esp32_connected: false` | Agent waiting for ESP32 |
| micro-ros-agent crashed, restarting | ⚠️ restarting | ❌ | Agent fault |
| joy service exited | ❌ | ❌ | Joy node fault |

---

### DockerImage

The custom rover image produced by `docker compose build`.

| Attribute | Value |
|-----------|-------|
| `tag` | `ecza-robotu:runtime` |
| `base` | `ros:humble-ros-base` |
| `build_stages` | deps, build, runtime (3-stage) |
| `colcon_args` | `--cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install` |
| `user` | `rosuser` (UID 1000, GID 1000, supplementary group `input` GID matched to host) |
| `entrypoint` | `docker/entrypoint.sh` |
| `no_registry` | true — never published; always built from source |

---

## State Transitions

### Stack Lifecycle

```
STOPPED
  │
  ▼ docker compose build
BUILT (image exists)
  │
  ▼ docker compose up
STARTING (containers starting, ROS2 graph forming)
  │
  ▼ all services running + /joy active (≤120 s)
OPERATIONAL
  │  ┌─────────────────────────────────────┐
  │  │ service crash → auto-restart (≤30 s) │
  │  └─────────────────────────────────────┘
  │
  ▼ docker compose stop
STOPPED
```

### ESP32 Connectivity (within OPERATIONAL state)

```
AGENT_WAITING
  │
  ▼ ESP32 connects via WiFi UDP
ESP32_CONNECTED   ──► (motion commands flow)
  │
  ▼ WiFi loss (≥watchdog_timeout_ms)
AGENT_WAITING     ──► (ESP32 firmware stops all motors)
  │
  ▼ ESP32 reconnects
ESP32_CONNECTED
```
