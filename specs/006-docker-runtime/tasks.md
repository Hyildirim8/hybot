# Tasks: Docker Runtime for ROS2 Rover Stack

**Feature branch**: `006-docker-runtime`
**Input**: `specs/006-docker-runtime/spec.md`, `plan.md`, `research.md`, `data-model.md`, `contracts/compose-schema.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the repository-root Docker project skeleton and configure
the build environment.

- [X] T001 Create top-level directory structure at repository root: `docker/`, `config/`, `bags/` (git-ignored) per `plan.md` Project Structure
- [X] T002 [P] Create `.env` file with default environment variables: `ROS_DOMAIN_ID=42`, `RECORD=false`, `AGENT_PORT=8888`, `VERBOSE=4` per `contracts/compose-schema.md` §1, `research.md` R-007
- [X] T003 [P] Create `.gitignore` entry for `bags/` directory and add `bags/.gitkeep` placeholder so the directory is tracked but its contents are not
- [X] T004 [P] Create `config/rover_params.yaml` with all required keys and defaults from `contracts/compose-schema.md` §2: geometry (`wheel_separation_width: 0.26`, `wheel_base: 0.38`, `wheel_radius: 0.05`), speed limits, joystick axes, deadzone, enable button, watchdog timeout
- [X] T005 [P] Create `config/rover_params.yaml.example` as a copy of the defaults with inline comments explaining each parameter (the operator-facing reference)

**Checkpoint**: Directory structure exists; `.env` and `config/rover_params.yaml` are valid YAML.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The Dockerfile (multi-stage) and entrypoint must exist before any
Compose service can be built or run. The `rosuser` non-root user and TWDT
environment must be established.

**⚠️ CRITICAL**: All user stories depend on `docker compose build` succeeding.

- [X] T006 Create `docker/Dockerfile` with three stages per `research.md` R-005:
  - Stage 1 `deps`: `FROM ros:humble-ros-base`, install rosdep dependencies with `rosdep install --from-paths src --ignore-src -y`
  - Stage 2 `build`: colcon build with `--merge-install` (NOT `--symlink-install`) into `/ws/install`
  - Stage 3 `runtime`: copy only `/ws/install` from build stage; create non-root user `rosuser` (UID 1000); `SHELL ["/bin/bash", "-c"]`
- [X] T007 Create `docker/entrypoint.sh`: sources `/opt/ros/humble/setup.bash` and `/ws/install/setup.bash`, then `exec "$@"` per `contracts/compose-schema.md` §5
- [X] T008 [P] Create `docker-compose.yaml` with all six services (joy, teleop, kinematics, micro-ros-agent, diagnostics, recorder) — stub versions with `image: ecza-robotu:runtime` (or upstream image for agent), `network_mode: host`, and volume mounts per FR-008, `research.md` R-002; recorder service guarded by `profiles: [record]`
- [X] T009 [P] Create `docker-compose.override.yaml` for development: bind-mount `./src` into the build stage, enable verbose logging, override `VERBOSE=6` for the agent
- [X] T010 Run `docker compose build` and confirm the image builds successfully with `RUN id rosuser` as a final validation step; fix any rosdep or colcon errors before proceeding

**Checkpoint**: `docker compose build` exits 0; `docker images | grep ecza-robotu` shows the image; all GPIO and ROS2 setup steps execute without error in build log.

---

## Phase 3: User Story 1 — One-Command Full Stack Launch (Priority: P1) 🎯 MVP

**Goal**: `docker compose up` starts all five services on a machine with no
host-side ROS2. All nodes appear in the ROS2 graph; `/cmd_vel` responds to
joystick input within 120 s of launch (image pre-built).

**Independent Test**: From a clean terminal with only Docker installed (no ROS2),
run `docker compose up`. Within 120 s, all services reach running state. Run
`docker compose exec kinematics ros2 node list` — all expected nodes visible.
Move the F710 stick and confirm `/cmd_vel` messages appear.

- [X] T011 [US1] Populate the `joy` service in `docker-compose.yaml`: `image: ecza-robotu:runtime`, `privileged: true`, `volumes: [/dev/input:/dev/input]`, `network_mode: host`, `user: rosuser`, `command: ros2 run joy joy_linux` per FR-004, `research.md` R-004
- [X] T012 [US1] Populate the `teleop` service in `docker-compose.yaml`: `image: ecza-robotu:runtime`, `network_mode: host`, `user: rosuser`, `volumes: [./config/rover_params.yaml:/config/rover_params.yaml:ro]`, `command: ros2 run ecza_teleop teleop_node --ros-args --params-file /config/rover_params.yaml`, `depends_on: [joy]`
- [X] T013 [US1] Populate the `kinematics` service in `docker-compose.yaml`: same image/user/network/volume pattern; `command: ros2 run ecza_kinematics kinematics_node --ros-args --params-file /config/rover_params.yaml`, `depends_on: [teleop]`
- [X] T014 [US1] Populate the `micro-ros-agent` service in `docker-compose.yaml`: `image: microros/micro-ros-agent:humble`, `network_mode: host`, `command: udp4 --port ${AGENT_PORT} --middleware dds --verbose ${VERBOSE}`, `restart: unless-stopped` per `research.md` R-003
- [X] T015 [US1] Populate the `diagnostics` service in `docker-compose.yaml`: `image: ecza-robotu:runtime`, `network_mode: host`, `user: rosuser`, `command: ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file /config/rover_params.yaml`
- [X] T016 [US1] Create `docker/healthcheck.sh` — tier-1 implementation: iterate Compose service names, call `docker compose ps --format json`, output `running`/`stopped` per service, exit 0 if all required services running per `contracts/compose-schema.md` §3

**Checkpoint**: `docker compose up` → all five services running; tier-1 health check exits 0 within 5 s per SC-006.

---

## Phase 4: User Story 2 — Configuration Without Rebuilding (Priority: P2)

**Goal**: Editing `config/rover_params.yaml` on the host and restarting the
stack applies new parameter values without `docker compose build`.

**Independent Test**: Change `max_linear_speed` in `config/rover_params.yaml`.
Run `docker compose restart`. Confirm the teleop node logs the new value at startup
(visible in `docker compose logs teleop`).

- [X] T017 [US2] Verify all rover services (teleop, kinematics, diagnostics) mount `./config/rover_params.yaml:/config/rover_params.yaml:ro` in `docker-compose.yaml` — this should already be set from T012–T015; explicitly confirm no service hard-codes parameter values in the `command:` field per FR-003
- [X] T018 [US2] Add `environment:` sections to all services reading from `.env`: `ROS_DOMAIN_ID=${ROS_DOMAIN_ID}` to ensure domain ID consistency across all containers per FR-010, `research.md` R-007
- [X] T019 [US2] Document the parameter change workflow in `specs/006-docker-runtime/quickstart.md` Step 4 (already drafted): emphasise that `docker compose build` is NOT needed for config changes, only for source code changes per FR-009

**Checkpoint**: Edit any value in `config/rover_params.yaml`, run `docker compose restart`, confirm new value in `docker compose logs` — no rebuild needed.

---

## Phase 5: User Story 3 — Joystick Device Access Inside Container (Priority: P3)

**Goal**: The `joy` service detects the F710 on any `/dev/input/js*` path,
including after replug, because `/dev/input` is bind-mounted and the service
runs privileged.

**Independent Test**: Plug in the F710 before launch → `/joy` messages appear
within 10 s. Unplug and replug while the stack is running → messages resume
within 15 s without container restart.

- [X] T020 [US3] Confirm the joy service `command` uses `joy_linux` (not `joy`): `joy_linux` uses `/dev/input/js*` (kernel joystick subsystem) while `joy` uses SDL `/dev/input/event*`; update if needed per `research.md` R-004, spec Assumptions
- [X] T021 [US3] Add `restart: unless-stopped` to the joy service in `docker-compose.yaml` so that a momentary permission error or missing device causes an automatic retry rather than leaving the service stopped per FR-006
- [X] T022 [US3] Add the udev rule to `specs/006-docker-runtime/quickstart.md` Step 1b (already drafted): `KERNEL=="js[0-9]*", SUBSYSTEM=="input", GROUP="input", MODE="0660"` — confirm this rule is present and correct

**Checkpoint**: F710 plug → `/joy` messages within 10 s; replug → messages resume within 15 s; no-joystick start → joy node logs clear warning, all other services unaffected per US3-AC3.

---

## Phase 6: User Story 4 — rosbag2 Recording from Container (Priority: P3)

**Goal**: `RECORD=true docker compose up` additionally starts the recorder
service; bags are written to `./bags/` on the host and persist after stop.

**Independent Test**: `RECORD=true docker compose up`, drive for 30 s, stop.
Confirm a `.mcap` bag directory exists in `./bags/` on the host containing
`/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics` topics with non-zero
message counts. Replay with `ros2 bag play`.

- [X] T023 [US4] Populate the `recorder` service in `docker-compose.yaml` using `profiles: [record]` so it only starts when `COMPOSE_PROFILES=record`; command: `ros2 bag record -o /bags/session_$(date +%Y%m%d_%H%M%S) /joy /cmd_vel /wheel_velocities /diagnostics`; `volumes: [./bags:/bags:rw]` per FR-005, `research.md` R-008. Note: activated via `scripts/launch.sh RECORD=true`, NOT by setting `RECORD=true` directly in the environment.
- [X] T024 [US4] Ensure `./bags/` is auto-created on host by adding `volumes:` top-level entry in `docker-compose.yaml` with `driver: local` or a pre-create script in the launch command per US4-AC3
- [X] T025 [US4] Update `docker/healthcheck.sh` with tier-2 implementation: `docker compose exec kinematics ros2 topic list` to verify `/joy`, `/cmd_vel`, `/wheel_velocities`, `/diagnostics` are active; `ros2 node list | grep esp32_firmware_node` for ESP32 status; output and exit code per `contracts/compose-schema.md` §3

**Checkpoint**: `RECORD=true` → valid bag in `./bags/` after stop; `RECORD=false` (default) → no bag created; tier-2 health check exits 0 when all topics active.

---

## Phase 7: Polish & Cross-Cutting Concerns

- [X] T026 [P] Create `scripts/launch.sh` wrapper script that: (1) validates `docker --version ≥ 24.0` and `docker compose version ≥ 2.20`, aborting with a clear error message if below minimum per Edge Cases; (2) translates `RECORD=true` in the environment into `COMPOSE_PROFILES=record` before invoking `docker compose up "$@"`, so operators can use `RECORD=true ./scripts/launch.sh` as documented per FR-005, A6. This script is the canonical launch entrypoint; the quickstart MUST reference it instead of bare `docker compose up`.
- [X] T027 [P] Add `restart: unless-stopped` to all remaining services (kinematics, teleop, diagnostics) in `docker-compose.yaml` per FR-006; confirm micro-ros-agent already has `unless-stopped` from T014
- [X] T028 Add Compose `healthcheck:` to the micro-ros-agent service: `test: ["CMD", "sh", "-c", "ss -ulnp | grep ':${AGENT_PORT}'"]`, `interval: 10s`, `timeout: 5s`, `retries: 3` per `research.md` R-003
- [X] T029 Run the full quickstart end-to-end validation: fresh Docker-only machine, follow `specs/006-docker-runtime/quickstart.md` steps 1–9, confirm tier-2 health check exits 0 and rover responds to joystick per SC-001, SC-006, SC-007

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    └── Phase 2 (Foundational — Dockerfile + Compose skeleton + image build)
            ├── Phase 3 (US1 — P1) 🎯 MVP [all five services wired up]
            │       └── Phase 4 (US2 — P2) [config mount verification]
            ├── Phase 5 (US3 — P3) [joystick passthrough confirmation]
            ├── Phase 6 (US4 — P3) [recorder service + tier-2 healthcheck]
            └── Phase 7 (Polish)
```

### User Story Dependencies

| Story | Depends on foundational | Depends on other stories |
|-------|------------------------|--------------------------|
| US1 (P1) | T006–T010 (Dockerfile builds) | None |
| US2 (P2) | T006–T010 | T012–T015 (services exist to verify) |
| US3 (P2) | T006–T010, T011 (joy service) | None independently |
| US4 (P3) | T006–T010 | T025 (tier-2) reads topics from US1 services |

### Parallel Opportunities

Within Phase 1: T002–T005 all independent.
Within Phase 2: T008 and T009 can be written before T010 (build validation).
Within Phase 3: T011–T015 are separate service stanzas in the same file — assign one service per developer if staffed.

---

## Parallel Example: Phase 3 Service Wiring

```
T011 (joy service)        ─┐
T012 (teleop service)     ─┤  All can be written concurrently
T013 (kinematics service) ─┤  (different stanzas in docker-compose.yaml)
T014 (agent service)      ─┤
T015 (diagnostics service)─┘
Then: T010 (docker compose build) — sequential
Then: T016 (healthcheck.sh tier-1) — sequential
```

---

## Implementation Strategy

### MVP Scope (US1 only — Phases 1–3)

1. Complete Phase 1: Setup
2. Complete Phase 2: Dockerfile + build
3. Complete Phase 3: All five services wired
4. **Validate**: `docker compose up` → all nodes visible, joystick works
5. Stop and demo — containerised rover, MVP delivered

### Incremental Delivery

| Increment | Stories | Validates |
|-----------|---------|-----------|
| 1 | Phases 1–3 (US1) | One-command launch, no host ROS2 needed |
| 2 | + Phase 4 (US2) | Config changes without rebuild |
| 3 | + Phase 5 (US3) | Joystick replug works |
| 4 | + Phase 6 (US4) | Bag recording and tier-2 health check |
| 5 | + Phase 7 (Polish) | Restart policies, healthchecks, version guard |

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 5 (T001–T005) | 4 |
| Phase 2 — Foundational | 5 (T006–T010) | 2 |
| Phase 3 — US1 (P1) | 6 (T011–T016) | 5 |
| Phase 4 — US2 (P2) | 3 (T017–T019) | 0 |
| Phase 5 — US3 (P2) | 3 (T020–T022) | 0 |
| Phase 6 — US4 (P3) | 3 (T023–T025) | 0 |
| Phase 7 — Polish | 4 (T026–T029) | 2 |
| **Total** | **29** | **13** |
