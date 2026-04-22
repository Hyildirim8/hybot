# Contract: Docker Compose Stack Schema

**Feature**: `006-docker-runtime`
**Date**: 2026-02-24
**Contract Type**: Operational interface — the schema that operators and CI tools
must conform to when deploying or extending the rover stack.

---

## 1. Launch Interface

### Primary command

```
docker compose up
```

Starts all services. The image must have been built first with:

```
docker compose build
```

### Optional flags (via `.env` or inline env)

| Variable | Values | Default | Effect |
|----------|--------|---------|--------|
| `ROS_DOMAIN_ID` | 0–101, 215–232 | `42` | DDS domain for all services |
| `RECORD` | `true` / `false` | `false` | Activates the recorder service |
| `AGENT_PORT` | integer | `8888` | UDP port for micro-ros-agent |
| `VERBOSE` | `0`–`6` | `4` | micro-ros-agent verbosity |

### Teardown

```
docker compose down
```

Stops and removes containers. Does NOT remove the `./bags/` directory or the built image.

---

## 2. Configuration File Contract

**Path on host**: `./config/rover_params.yaml`
**Path in container**: `/config/rover_params.yaml` (read-only bind-mount)

All keys MUST use `ros__parameters` (double underscore). Missing required keys
cause the affected node to log ERROR and exit.

### Required keys

```yaml
/**:
  ros__parameters:
    # Rover geometry — all must be positive non-zero floats
    wheel_separation_width: <float>   # metres; default 0.26
    wheel_base: <float>               # metres; default 0.38
    wheel_radius: <float>             # metres; MUST be measured before hardware tests

    # Speed limits — must be positive
    max_linear_speed: <float>         # m/s; default 0.5
    max_angular_speed: <float>        # rad/s; default 1.0

    # Joystick — integer axis/button indices (F710 D-mode layout)
    joy_deadzone: <float>             # [0, 1); default 0.05
    axis_linear_x: <int>              # default 1 (left stick vertical)
    axis_linear_y: <int>              # default 0 (left stick horizontal)
    axis_angular_z: <int>             # default 3 (right stick horizontal)
    enable_button: <int>              # default 5 (right bumper, dead-man switch)

    # Watchdog
    watchdog_timeout_ms: <int>        # ms; default 500
```

### Optional keys

```yaml
/**:
  ros__parameters:
    # micro-ROS agent (informational; agent port is set via AGENT_PORT env var)
    agent_port: <int>                 # default 8888; must match AGENT_PORT
```

**Validation contract**: A node that detects a zero or negative geometry value
MUST log at ERROR level and cease publishing (Principle II compliance).

---

## 3. Health Check Interface

### Tier 1 — Container state

```bash
docker/healthcheck.sh --tier1
```

**Output contract** (stdout, one line per service):

```
joy            running
teleop         running
kinematics     running
micro-ros-agent running
diagnostics    running
recorder       running|stopped
```

Exit code: `0` if all required services are `running`, `1` otherwise.

### Tier 2 — ROS2 graph state

```bash
docker/healthcheck.sh --tier2
```

**Output contract** (stdout):

```
topics:
  /joy                  active
  /cmd_vel              active
  /wheel_velocities     active
  /diagnostics          active
esp32_node:             connected|waiting
```

Exit code: `0` if all four topics are active (ESP32 state is informational only,
does not affect exit code). `1` if any required topic is missing.

---

## 4. Volume / File Contract

| Host path | Container path | Mode | Description |
|-----------|----------------|------|-------------|
| `./config/rover_params.yaml` | `/config/rover_params.yaml` | `ro` | All runtime params |
| `./bags/` | `/bags/` | `rw` | rosbag2 output (auto-created) |
| `/dev/input` | `/dev/input` | `rw` | Joystick devices (joy service only) |

---

## 5. Image Contract

| Attribute | Value |
|-----------|-------|
| Tag | `ecza-robotu:runtime` |
| Build command | `docker compose build` |
| No registry | Never pushed; always built locally |
| Entrypoint | `/entrypoint.sh` — sources ROS2 setup, then `exec "$@"` |
| Working directory | `/ws` |
| Default user | `rosuser` (UID 1000) |
| Exposed ports | None (host networking; no port mapping) |

**Breaking change contract**: Any change to the entrypoint interface, the
`/config/rover_params.yaml` key schema, or the health check exit codes constitutes
a breaking change and requires a version bump in the Compose file label.

---

## 6. Compose Service Dependency Contract

```
[joy] ──────────────► [teleop]
                           │
                           ▼
                      [kinematics]
                           │
                           ▼
              [micro-ros-agent] (independent)
              [diagnostics]     (independent)
              [recorder]        (independent, conditional on RECORD=true)
```

`depends_on` in Compose enforces start ordering only; readiness is determined
by the tier-2 health check, not by container start order alone.
