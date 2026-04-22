# Tasks: Wheel Encoder Feedback

**Input**: `specs/008-encoder-feedback/` — plan.md, spec.md, research.md, data-model.md, contracts/topics.md
**Branch**: `008-encoder-feedback`
**Tech stack**: C17 (ESP-IDF firmware), `driver/pulse_cnt.h`, `driver/gptimer.h`, `micro_ros_espidf_component`
**Modified files**: 5 existing + 4 new firmware files + 1 config file

---

## Phase 1: Setup (Blocking Infrastructure)

**Purpose**: Changes that must exist before any encoder code can compile or run.
These touch the build system and micro-ROS session limits — all downstream tasks
depend on them.

- [X] T001 Raise `RMW_UXRCE_MAX_PUBLISHERS` from `2` to `3` in `firmware/components/micro_ros_espidf_component/colcon.meta`
- [X] T002 Rebuild `libmicroros.a` by running `idf.py build` in `firmware/` (depends on T001)
- [X] T003 Add `esp_driver_pcnt` and `esp_driver_gptimer` to `REQUIRES` in `firmware/main/CMakeLists.txt`
- [X] T004 **[Deferred to Phase 3 — see T004b]** ~~Add `encoder.c` and `wheel_publisher.c` to `SRCS`~~ — leave a `# TODO(T004b): add encoder.c wheel_publisher.c after Phase 3` comment in `firmware/main/CMakeLists.txt` now to mark the insertion point; actual SRCS entries are added in T004b after source files exist

**Checkpoint**: Build system ready — PCNT/GPTimer components available, publisher limit raised, `libmicroros.a` rebuilt. (Note: build will NOT include encoder sources yet; that is intentional until T004b.)

---

## Phase 2: Foundational (Blocking Prerequisites for All User Stories)

**Purpose**: Executor capacity and spin-rate changes needed by both the publisher
(US1) and diagnostics (US3). Must complete before user story phases.

- [X] T005 Increase executor capacity from `3` to `4` in `uros_transport.c` line 112–113 — `rclc_executor_init(executor, &support->context, 4, &s_allocator)`
- [X] T006 [P] Change `SPIN_TIMEOUT_NS` from `50ULL * 1000000ULL` to `15ULL * 1000000ULL` in `firmware/main/app_main.c` (enables 50 Hz timer resolution)
- [X] T007 Add encoder Kconfig menu to `firmware/main/Kconfig.projbuild` with symbols: `CONFIG_ENCODER_CPR` (default 1980), `CONFIG_ENCODER_NOISE_FLOOR` (default 2), `CONFIG_ENCODER_SAMPLE_PERIOD_MS` (default 20), `CONFIG_ENCODER_GLITCH_NS` (default 1000)

**Checkpoint**: Foundation ready — executor has capacity for wheel_publisher timer; spin rate supports 50 Hz; Kconfig symbols defined.

---

## Phase 3: User Story 1 — Real-Time Wheel Velocity Feedback (Priority: P1) 🎯 MVP

**Goal**: Firmware reads all four quadrature encoders via PCNT hardware and publishes
measured wheel velocities as `Float32MultiArray` on `/wheel_velocities` at 50 Hz.

**Independent Test**: `ros2 topic hz /wheel_velocities` shows ~50 Hz. Spin each wheel
by hand — only that wheel's index changes. All values positive when driving forward.
All values 0.0 when stationary.

### Implementation for User Story 1

- [X] T008 [P] [US1] Create `firmware/main/encoder.h` — declare `encoder_init()`, `encoder_start()`, `g_encoder_velocities[4]`, `g_encoder_counts[4]`, `g_encoder_faults[4]`; define GPIO defaults: FL(8,9), FR(10,11), RL(12,13), RR(14,21); define `#ifndef CONFIG_ENCODER_*` guard macros matching Kconfig symbols from T007
- [X] T009 [US1] Create `firmware/main/encoder.c` — implement `encoder_init()`: allocate 4 PCNT units with `accum_count=1`, configure both channels per unit for 4X quadrature (edge=A level=B / edge=B level=A), set glitch filter, add ±32767 watchpoints, clear counts (depends on T007, T008)
- [X] T010 [US1] Add GPTimer periodic alarm in `firmware/main/encoder.c` — `velocity_sample_cb()` ISR: reads `pcnt_unit_get_count()` for all 4 units, computes `Δcount`, applies noise-floor suppression, computes `ω = Δcount * 2π / (CPR * Δt_s)`, clamps to `MAX_SPEED_RAD_S`, writes `g_encoder_velocities[]`, `g_encoder_counts[]`, `g_encoder_faults[]` (depends on T009)
- [X] T011 [US1] Implement `encoder_start()` in `firmware/main/encoder.c` — calls `pcnt_unit_enable()` + `pcnt_unit_start()` for all 4 units (depends on T009)
- [X] T012 [P] [US1] Create `firmware/main/wheel_publisher.h` — declare `wheel_publisher_init(rcl_node_t*, rclc_support_t*, rclc_executor_t*)` and `wheel_publisher_fini(rcl_node_t*)`
- [X] T013 [US1] Create `firmware/main/wheel_publisher.c` — implement `wheel_publisher_init()`: init static `Float32MultiArray` with `s_data[4]` static buffer (no malloc), create `rcl_publisher_t` on `/wheel_velocities`, register `rclc_timer` at period `CONFIG_ENCODER_SAMPLE_PERIOD_MS` ms (default 20 ms — from Kconfig, not hardcoded) in executor; timer callback reads `g_encoder_velocities[]` and calls `rcl_publish()` (depends on T005, T008, T012)
- [X] T014 [US1] Implement `wheel_publisher_fini()` in `firmware/main/wheel_publisher.c` — calls `rcl_timer_fini()` then `rcl_publisher_fini()` in that order; does NOT call `Float32MultiArray__fini()` (static buffer) (depends on T013)
- [X] T004b [US1] Add `encoder.c` and `wheel_publisher.c` to `SRCS` in `firmware/main/CMakeLists.txt` now that both files exist (depends on T013, T014; removes the TODO comment left by T004)
- [X] T015 [US1] Wire `encoder_init()` and `encoder_start()` into `firmware/main/app_main.c` — add calls after `motor_init()` and before the micro-ROS retry loop; add `wheel_publisher_init()` inside the retry loop after `status_reporter_init()`; add `wheel_publisher_fini()` in teardown before `status_reporter_fini()` (depends on T011, T013, T014)
- [ ] T016 [US1] Flash firmware to ESP32-S3 and verify: `ros2 topic hz /wheel_velocities` shows ~50 Hz; `ros2 topic echo /wheel_velocities` shows `[0.0, 0.0, 0.0, 0.0]` at rest; spinning each wheel by hand changes only that index; all positive when driving forward (SC-001, SC-004). **FR-009 disconnect test**: note current `encoder_counts[i]` value, kill the micro-ROS agent (`docker compose stop micro_ros_agent`), wait 10 s, restart agent — confirm `/wheel_velocities` resumes publishing within 1 s and `encoder_counts[i]` continues from the pre-disconnect value (not reset to 0).

**Checkpoint**: US1 complete — `/wheel_velocities` publishing at 50 Hz with correct per-wheel isolation and sign convention. MVP delivered.

---

## Phase 4: User Story 2 — Closed-Loop Odometry in RViz (Priority: P2)

**Goal**: Enable `mecanum_drive_controller` to use measured wheel velocities for
odometry by setting `open_loop: false` in `config/controllers.yaml`.

**Independent Test**: Drive 1 m straight; `ros2 topic echo /odom` reports `x` in
0.90–1.10 m range and yaw within ±5°. Requires US1 to be verified first (FR-011).

**⚠️ Prerequisite gate**: Do NOT apply T017 until T016 (US1 hardware verification) passes.

### Implementation for User Story 2

- [ ] T017 [US2] Change `open_loop: true` to `open_loop: false` in `config/controllers.yaml` under `mecanum_drive_controller` and `/**` wildcard sections (depends on T016 verified)
- [ ] T018 [US2] Rebuild Docker image and restart stack: `docker compose build && docker compose down && docker compose up -d`
- [ ] T019 [US2] Hardware verify per `quickstart.md` US2 procedure: drive 1 m straight, confirm `/odom` `x` ∈ [0.90, 1.10] m, yaw ≤ 5°; drive 1 m square, confirm return position error ≤ 15 cm (SC-003)
- [ ] T020 [US2] Record verification bag: `ros2 bag record /odom /cmd_vel /wheel_velocities /firmware_status` during 1 m straight drive (Constitution §IV hardware test documentation requirement)

**Checkpoint**: US2 complete — odometry computed from real encoder feedback; position error ≤ 5 cm on 1 m straight drive.

---

## Phase 5: User Story 3 — Encoder Diagnostics (Priority: P3)

**Goal**: Extend `/firmware_status` JSON to include per-wheel `encoder_counts`,
`encoder_velocities`, and `encoder_faults` so wiring faults and CPR misconfiguration
are visible without an oscilloscope.

**Independent Test**: `ros2 topic echo /firmware_status` shows all three new fields.
Spin a wheel 10 revolutions — `encoder_counts[i]` increases by ~1980 ticks. Stop —
`encoder_velocities[i]` drops to 0.0 within one 1 Hz publish cycle.

### Implementation for User Story 3

- [X] T021 [US3] Add three fields to `FirmwareStatus` struct in `firmware/main/status_reporter.h`: `int32_t encoder_counts[4]`, `float encoder_velocities[4]`, `bool encoder_faults[4]` (depends on T008)
- [X] T022 [US3] Extend `status_serialize()` in `firmware/main/status_reporter.c` to read `g_encoder_counts[]`, `g_encoder_velocities[]`, `g_encoder_faults[]` into the new struct fields and emit them as JSON keys `"encoder_counts"`, `"encoder_velocities"`, `"encoder_faults"` after existing fields (depends on T010, T021)
- [ ] T023 [US3] Flash updated firmware; verify via `ros2 topic echo /firmware_status` that JSON includes all three new arrays; spin one wheel 10 revolutions by hand and confirm `encoder_counts[i]` accumulates ~1980 ticks per revolution (SC-006)

**Checkpoint**: US3 complete — all three encoder diagnostic fields visible in `/firmware_status`; cumulative count verified against known CPR.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: CPR calibration confirmation, stability validation, and documentation.

- [ ] T024 [P] Verify CPR by hand: spin one output shaft exactly 10 revolutions, check `encoder_counts[i]` = 19 800 ± 20; if outside tolerance update `CONFIG_ENCODER_CPR` default in `firmware/main/Kconfig.projbuild` and re-flash
- [ ] T025 30-minute stability run: drive rover continuously with joystick including full-speed bursts, direction reversals, and one simulated micro-ROS disconnect (cycle USB); confirm no firmware crash, no watchdog reset, no count corruption (SC-005)
- [X] T026 [P] Add `#include "encoder.h"` header guard comment in `firmware/main/app_main.c` explaining one-time init before retry loop (code clarity for future maintainers)
- [X] T027 [P] Update `firmware/main/CMakeLists.txt` comment block to document new SRCS and REQUIRES additions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 completion — blocks all user stories
- **Phase 3 (US1)**: Depends on Phase 2 — MVP; must complete before US2 gate check
- **Phase 4 (US2)**: Depends on Phase 3 **hardware verification** (T016) — `open_loop: false` is explicitly gated
- **Phase 5 (US3)**: Depends on Phase 3 data structures (T008, T010) — can be done in parallel with Phase 4 once T010 exists
- **Phase 6 (Polish)**: Depends on all user stories complete

### User Story Dependencies

- **US1**: Encoder hardware layer; all other stories depend on it
- **US2**: Depends on US1 verified (hardware gate); single config change + Docker rebuild
- **US3**: Depends on US1 data structures (`g_encoder_*` globals); can be parallelised with US2

### Within Each User Story

- `encoder.h` (T008) before `encoder.c` (T009, T010, T011)
- `encoder.c` complete before `wheel_publisher.c` (T013) — publisher reads globals
- `wheel_publisher.h` (T012) before `wheel_publisher.c` (T013)
- Hardware verify (T016) before `open_loop: false` (T017) — explicit gate

---

## Parallel Opportunities

### Phase 1 (can parallelize T003 + T004 prep with T001 + T002)

```
T001 colcon.meta → T002 rebuild libmicroros   [sequential — rebuild depends on edit]
T003 CMakeLists REQUIRES                       [parallel with T001/T002]
```

### Phase 3 (US1 parallel opportunities)

```
T008 encoder.h ──┬──► T009 encoder.c init
T012 publisher.h ┘    T010 GPTimer ISR     [T009 + T012 in parallel after T008]
                       T011 encoder_start  [T010 → T011 sequential]
                  └──► T013 publisher.c    [parallel with T009 if T008 done]
```

### Phase 5 + 4 (can run in parallel after T010)

```
Phase 4: T017 → T018 → T019 → T020
Phase 5: T021 → T022 → T023           [can run simultaneously with Phase 4]
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001–T004)
2. Complete Phase 2: Foundational (T005–T007)
3. Complete Phase 3: User Story 1 (T008–T015)
4. **Flash and validate** T016 — spin test each wheel, verify 50 Hz, verify sign
5. **STOP** — `/wheel_velocities` is live; rover has real encoder feedback
6. Proceed to US2 and US3 only after MVP validated

### Incremental Delivery

- **After T016** (US1 done): real encoder feedback active; open-loop odometry still running but correct data is on the wire
- **After T019** (US2 done): closed-loop odometry active; drift visibly reduced in RViz
- **After T023** (US3 done): full diagnostic visibility; CPR and sign issues diagnosable over ROS topic alone

### Single-Developer Sequence

```
T001 → T002 → T003/T004 prep → T005 → T006 → T007
→ T008 → T009 → T010 → T011 → T012 → T013 → T014 → T015
→ T016 [GATE: verify hardware]
→ T021 → T022 → T023   (US3 diagnostic — quick, no hardware dependency)
→ T017 → T018 → T019 → T020   (US2 odometry — requires hardware drive test)
→ T024 → T025 → T026 → T027
```
