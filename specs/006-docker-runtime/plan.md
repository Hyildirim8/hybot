# Implementation Plan: Docker Runtime for ROS2 Rover Stack

**Branch**: `006-docker-runtime` | **Date**: 2026-02-24 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/006-docker-runtime/spec.md`

## Summary

Package the full mecanum rover ROS2 stack (joy_linux, teleop, kinematics,
micro-ros-agent, diagnostics, optional recorder) into a Docker Compose project.
A single `docker compose build && docker compose up` starts everything on any
Ubuntu 22.04 host with Docker — no host-side ROS2 Humble installation needed.
All runtime parameters flow through a single host-mounted YAML; no image rebuild
is required for config changes. The micro-ros-agent uses `network_mode: host`
for ESP32 WiFi UDP reach; all other ROS2 nodes also use host networking to avoid
Fast-DDS multicast discovery failures on bridge networks.

## Technical Context

**Language/Version**: Dockerfile (multi-stage), YAML (Compose v3.8+), Bash (helper scripts)
**Base Image**: `ros:humble-ros-base` (authoritative OSRF source, ~500 MB, no GUI tooling)
**micro-ROS Agent Image**: `microros/micro-ros-agent:humble` (upstream official, pinned to `humble` tag)
**Primary Dependencies**:
  - Docker Engine ≥ 24.0, Docker Compose ≥ v2.20 (Compose file format v3.8)
  - ROS2 Humble packages: `joy_linux`, `teleop_twist_joy`, `ros2bag`, `rosbag2`, `diagnostic_aggregator`
  - All rover ROS2 packages from this repo (colcon workspace built inside image)
**Storage**: Host-mounted bind volumes only — `./config/` (YAML params, read-only) and `./bags/` (rosbag2 output)
**Testing**: `docker compose run --rm <service> <command>` smoke tests; tier-1 health via `docker compose ps`; tier-2 health via `docker exec` + `ros2 topic list`
**Target Platform**: Ubuntu 22.04 LTS, x86_64 (amd64); Docker Engine (not Docker Desktop)
**Project Type**: Container runtime / deployment packaging (wraps existing ROS2 packages)
**Network Mode**: `network_mode: host` for **all** services (not just micro-ros-agent) — required because Fast-DDS UDP multicast does not work reliably across Docker bridge networks; host networking is the production-standard pattern for ROS2 on a single-host robot
**Performance Goals**: Stack up within 120 s of `docker compose up` (image pre-built); all ROS2 topics live within that window
**Constraints**: No image registry; build-from-source only. Joy service runs privileged (scoped exception). Non-root user (`rosuser`, UID 1000) for all services. `ROS_DOMAIN_ID=42` default.
**Scale/Scope**: Single host, single rover, ~6 Compose services, 2 Docker images

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-checked after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. ROS2-First Architecture | ✅ PASS | All rover nodes remain ROS2 packages; Docker is packaging only, not a new communication layer |
| II. Mecanum Kinematics Correctness | ✅ PASS | Kinematics node is unchanged; wheel params flow through mounted YAML at runtime |
| III. Joystick Input via joy/joy_linux | ✅ PASS | `joy_linux` runs inside container; `/dev/input` bind-mounted; F710 D-mode required |
| IV. Hardware Abstraction Layer | ✅ PASS | micro-ros-agent uses upstream official image; no custom bridge node introduced |
| V. Observability & Diagnostics | ✅ PASS | rosbag2 recording via `RECORD=true`; bags written to host-mounted `./bags/`; diagnostics node in stack |
| VI. Simplicity & Incremental Delivery | ⚠️ JUSTIFIED | Joy service runs privileged; all services use `network_mode: host` — see Complexity Tracking |

### Post-Design Re-check (after Phase 1)

*To be completed after data-model.md and contracts/ are written.*

## Project Structure

### Documentation (this feature)

```text
specs/006-docker-runtime/
├── plan.md              ← this file
├── research.md          ← Phase 0 output
├── data-model.md        ← Phase 1 output
├── quickstart.md        ← Phase 1 output
├── contracts/           ← Phase 1 output
│   └── compose-schema.md
└── tasks.md             ← Phase 2 (speckit.tasks — NOT created here)
```

### Source Code (repository root)

```text
docker/
├── Dockerfile               # Multi-stage: deps → build → runtime
├── entrypoint.sh            # Sources ROS2 + workspace setup.bash, execs CMD
└── healthcheck.sh           # Tier-2: ros2 node list + topic check

config/
└── rover_params.yaml        # Single host-mounted config (all tuneable params)

bags/                        # Host-side rosbag2 output (git-ignored, auto-created)

docker-compose.yaml          # Top-level Compose file (all services)
docker-compose.override.yaml # Dev overrides (verbose logging, bind-mount src)
.env                         # Default env vars (ROS_DOMAIN_ID, RECORD, etc.)
```

**Structure Decision**: Flat layout at repo root. `docker/` holds all container
build assets. `config/` holds the single runtime YAML mounted read-only into all
services. No restructuring of existing ROS2 package paths — they are COPYed into
the image at build time.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|--------------------------------------|
| Joy service runs `privileged: true` | Ubuntu 22.04 `/dev/input/js*` nodes are owned `root:input` mode `0660`. A non-root user needs GID membership matching the host `input` GID, which varies across Ubuntu installs (commonly 999 or 1000). Privileged is the only reliably portable hot-plug solution. | `devices:` only passes through nodes existing at container start — hot-plug fails. `group_add` requires knowing host GID at image build time — not portable. Scoped to joy service only. |
| `network_mode: host` for all services | Fast-DDS UDP multicast does not work across Docker bridge networks by default, causing 10–30 s discovery timeouts or complete topic discovery failure. Host networking makes DDS multicast work natively. | Bridge + explicit Fast-DDS peer config requires hardcoded container IPs (change on restart). CycloneDDS as alternative RMW adds an undeclared constitution dependency. |
