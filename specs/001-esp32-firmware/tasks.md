# Tasks: ESP32-S3 Motor Firmware

**Feature branch**: `001-esp32-firmware`
**Input**: `specs/001-esp32-firmware/spec.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the ESP-IDF firmware project skeleton and configure the
build environment. No story-specific logic yet.

- [X] T001 Initialize ESP-IDF project with `idf.py create-project rover_firmware` and set target `idf.py set-target esp32s3` in `firmware/`
- [X] T002 Add micro-ROS as an ESP-IDF component by cloning `micro_ros_espidf_component` into `firmware/components/micro_ros_espidf_component/`
- [X] T003 [P] Create `firmware/CMakeLists.txt` registering `main/` component and `components/micro_ros_espidf_component`
- [X] T004 [P] Create `firmware/sdkconfig.defaults` setting: `CONFIG_ESP_TASK_WDT_TIMEOUT_S=2`, `CONFIG_FREERTOS_HZ=1000`, `CONFIG_LWIP_SO_RCVBUF=1`, UDP stack enabled
- [X] T005 [P] Create `firmware/main/CMakeLists.txt` listing all source files to be added in later tasks

**Checkpoint**: `idf.py build` succeeds with an empty `app_main` before any feature code.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that ALL three user stories depend on.
No story work can begin until this phase is complete.

**⚠️ CRITICAL**: All three stories share the motor driver abstraction, the
WiFi + micro-ROS transport, and the NVS configuration layer. These MUST exist
first.

- [X] T006 Define `MotorChannel` struct and `motor_init()` / `motor_set_pwm(channel, rpwm_duty, lpwm_duty)` / `motor_stop_all()` API in `firmware/main/motor.h` and `firmware/main/motor.c` — sets LEDC PWM channels, drives all GPIO LOW on init per FR-010
- [X] T007 Implement LEDC PWM output for 4×BTS7960B (8 channels: RPWM+LPWM per motor) with `PWM_MAX_TICKS` constant in `firmware/main/motor.c`; GPIO pins defined as named constants; shoot-through guard enforced in `motor_set_pwm()` per Edge Cases
- [X] T008 [P] Implement NVS read/write helpers for WiFi credentials and agent IP/port in `firmware/main/nvs_config.h` and `firmware/main/nvs_config.c` — provides `nvs_config_load()` returning a `RoverConfig` struct (FR-012)
- [X] T009 Implement WiFi station init + IP-obtained event wait in `firmware/main/wifi.h` and `firmware/main/wifi.c`; credentials sourced from `nvs_config_load()` result
- [X] T010 Implement micro-ROS UDP transport init, node creation (`esp32_firmware_node`, namespace `/rover`), executor spin loop in `firmware/main/uros_transport.h` and `firmware/main/uros_transport.c`; node name and namespace match FR-011
- [X] T011 Enable ESP32 Task Watchdog Timer (TWDT) at 2000 ms in `firmware/main/app_main.c` via `esp_task_wdt_init()` before any FreeRTOS task creation; register main task with TWDT per FR-010

**Checkpoint**: Device boots, connects to WiFi, registers as `/rover/esp32_firmware_node` in `ros2 node list`, all motors stay stopped, TWDT active.

---

## Phase 3: User Story 1 — Receive and Execute Wheel Velocity Commands (Priority: P1) 🎯 MVP

**Goal**: Firmware subscribes to `/wheel_velocities`, converts each rad/s value
to BTS7960B RPWM/LPWM duty cycles, and drives the four GB37-520 motors within
100 ms of message receipt.

**Independent Test**: Publish `std_msgs/msg/Float32MultiArray` on `/wheel_velocities`
with 4 distinct nonzero values. Confirm each wheel spins in the correct direction
and stops within 100 ms of an all-zeros command. Confirm a value exceeding max
is clamped, not passed through.

- [X] T012 [US1] Implement `speed_to_duty(float speed_rad_s) -> uint32_t` using formula `duty = (|speed| / MAX_SPEED_RAD_S) × PWM_MAX_TICKS`, round-half-up, in `firmware/main/motor.c`; `MAX_SPEED_RAD_S` sourced from NVS or compile-time constant per FR-009
- [X] T013 [US1] Implement per-wheel clamp `clamp_speed(float speed) -> float` enforcing `[-MAX_SPEED_RAD_S, +MAX_SPEED_RAD_S]` in `firmware/main/motor.c` per FR-004
- [X] T014 [US1] Create micro-ROS subscriber for `/wheel_velocities` (`std_msgs/msg/Float32MultiArray`, QoS RELIABLE/VOLATILE/KEEP_LAST(1)) in `firmware/main/velocity_subscriber.h` and `firmware/main/velocity_subscriber.c` per FR-001, FR-011
- [X] T015 [US1] Implement subscription callback: validate array length == 4, call `clamp_speed()` per element, call `motor_set_pwm()` for each of the 4 `MotorChannel` instances (FL[0] FR[1] RL[2] RR[3]) per FR-002, FR-003
- [X] T016 [US1] Register subscriber with executor and wire callback into `uros_transport` spin loop in `firmware/main/app_main.c`

**Checkpoint**: `ros2 topic pub /wheel_velocities` drives all four wheels; all-zeros stops them; clamping verified by publishing an out-of-range value.

---

## Phase 4: User Story 2 — Safe Stop on Communication Loss (Priority: P2)

**Goal**: A FreeRTOS timer resets on every valid command received. If no command
arrives within the watchdog timeout, all motors are stopped automatically. Normal
operation resumes on the next valid command.

**Independent Test**: Publish a stream of commands, then stop publishing. Confirm
all wheels stop after ≤500 ms with no manual intervention. Confirm that resuming
publication restores motion.

- [X] T017 [US2] Implement `WatchdogTimer` as a FreeRTOS `xTimerCreate` with configurable period (default 500 ms from NVS/constant) and callback `watchdog_expire_cb()` that calls `motor_stop_all()` and sets `g_watchdog_state = TIMED_OUT` in `firmware/main/watchdog.h` and `firmware/main/watchdog.c` per FR-005
- [X] T018 [US2] Add `watchdog_reset()` call inside the `/wheel_velocities` subscription callback (after successful message validation) so each valid command restarts the timer; set `g_watchdog_state = ACTIVE` per FR-006 in `firmware/main/velocity_subscriber.c`
- [X] T019 [US2] Implement micro-ROS session loss detection in `firmware/main/uros_transport.c`: on `RMW_RET_ERROR` or executor timeout, call `watchdog_expire_cb()` immediately (do not wait for timer expiry) and attempt transport re-initialisation per FR-013
- [X] T020 [US2] Set safe default state at boot: call `motor_stop_all()` and `watchdog_expire_cb()` before the executor spin loop starts in `firmware/main/app_main.c`; ensure no motion occurs before the first valid command per US2-AC3

**Checkpoint**: Interrupt WiFi during active motion — all wheels stop within watchdog period. Resume WiFi — motion resumes on next published command.

---

## Phase 5: User Story 3 — Firmware Status Reporting (Priority: P3)

**Goal**: Firmware publishes a JSON-encoded `FirmwareStatus` to `/firmware_status`
at ≥1 Hz, reflecting current commanded speeds, watchdog state, per-motor fault
flags, uptime, and malformed message count.

**Independent Test**: Run `ros2 topic echo /firmware_status` with no commands
being sent. Verify messages arrive at ≥1 Hz with all expected JSON fields. Trigger
a watchdog stop; verify `watchdog_state` changes to `"timed_out"` in the next
frame.

- [X] T021 [P] [US3] Define `FirmwareStatus` JSON schema as a C struct and `status_serialize(FirmwareStatus *s, char *buf, size_t len) -> int` serialiser in `firmware/main/status_reporter.h` and `firmware/main/status_reporter.c`; fields: `commanded_speeds[4]`, `watchdog_state`, `motor_faults[4]`, `uptime_ms`, `malformed_msg_count` per FR-007, Key Entities
- [X] T022 [P] [US3] Implement malformed-message counter `g_malformed_msg_count` incremented in the subscriber callback when array length ≠ 4 or message deserialization fails; exposed via `status_reporter.h` per FR-008
- [X] T023 [US3] Create micro-ROS publisher for `/firmware_status` (`std_msgs/msg/String`, QoS BEST_EFFORT/VOLATILE/KEEP_LAST(1)) in `firmware/main/status_reporter.c` per FR-007, FR-011
- [X] T024 [US3] Implement FreeRTOS timer (1 Hz) that calls `status_serialize()` on the current global state snapshot and publishes to `/firmware_status`; timer registered with executor in `firmware/main/app_main.c`
- [X] T025 [US3] Wire `g_watchdog_state`, `g_motor_faults[4]`, `g_commanded_speeds[4]`, `uptime_ms` (`esp_timer_get_time()` / 1000) into the status snapshot read by the 1 Hz timer in `firmware/main/status_reporter.c`

**Checkpoint**: `ros2 topic hz /firmware_status` shows ≥1 Hz; `ros2 topic echo /firmware_status` shows all fields; watchdog state reflects timed_out after communication loss.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Completeness, safety hardening, and documentation that spans all
three stories.

- [X] T026 [P] Add `sdkconfig` entries for GPIO pin assignments (RPWM/LPWM/EN per motor × 4) as `Kconfig`-exposed symbols in `firmware/main/Kconfig.projbuild` so pins are configurable without editing source
- [X] T027 [P] Add NVS provisioning helper script `tools/provision_nvs.py` that writes SSID, password, agent IP, agent port, and max speed to NVS over USB (flash time only) per FR-012
- [X] T028 Add `README.md` to `firmware/` documenting: topic names, message types, QoS profiles, JSON status schema, GPIO pinout table, NVS key names, watchdog default, and `idf.py build flash monitor` commands per SC-006
- [X] T029 [P] Validate TWDT subscription in main task and any additional FreeRTOS tasks (subscriber task, status timer task) using `esp_task_wdt_add()` to prevent false TWDT resets during normal operation
- [X] T030 Run quickstart validation: flash firmware, publish 8 motion patterns from `ros2 topic pub`, verify SC-003 (no wheel wrong direction), record 100 consecutive `/firmware_status` frames and confirm zero parse errors per SC-005

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    └── Phase 2 (Foundational)  ← BLOCKS all user stories
            ├── Phase 3 (US1 — P1) 🎯 MVP
            ├── Phase 4 (US2 — P2)
            └── Phase 5 (US3 — P3)
                        └── Phase 6 (Polish)
```

### User Story Dependencies

| Story | Depends on foundational tasks | Depends on other stories |
|-------|-------------------------------|--------------------------|
| US1 (P1) | T006–T011 | None |
| US2 (P2) | T006–T011, T014 (subscriber exists) | Integrates with US1 subscriber callback |
| US3 (P3) | T006–T011 | Reads state written by US1 + US2 globals |

US2 has a soft dependency on T014–T015 (the subscriber must exist to call
`watchdog_reset()` inside it). Start US2 after US1's T015 is complete, or
implement T018 as a stub first and fill it in when T015 is ready.

### Parallel Opportunities

Within Phase 2: T008 (NVS) can run parallel to T006–T007 (motor driver).
Within Phase 5: T021 (serialiser) and T022 (malformed counter) are independent.
Across phases: Phase 6 tasks T026, T027, T029 can start as soon as Phase 2 completes.

---

## Parallel Example: Phase 2

```
Parallel track A:  T006 → T007 → T009 → T010 → T011
Parallel track B:  T008 (NVS config — no motor dependency)
```

## Parallel Example: User Story 3

```
Parallel:  T021 (status serialiser)
           T022 (malformed counter)
Sequential after both: T023 → T024 → T025
```

---

## Implementation Strategy

### MVP Scope (US1 only — Phases 1–3)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **Validate**: `ros2 topic pub /wheel_velocities` drives all 4 wheels correctly
5. Stop and demo if ready — rover moves, MVP delivered

### Incremental Delivery

| Increment | Stories | Validates |
|-----------|---------|-----------|
| 1 | Phases 1–3 (US1) | Rover moves on command |
| 2 | + Phase 4 (US2) | Rover stops safely on comms loss |
| 3 | + Phase 5 (US3) | Full observability via `/firmware_status` |
| 4 | + Phase 6 (Polish) | Production-ready, documented, validated |

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 5 (T001–T005) | 3 |
| Phase 2 — Foundational | 6 (T006–T011) | 1 |
| Phase 3 — US1 (P1) | 5 (T012–T016) | 0 |
| Phase 4 — US2 (P2) | 4 (T017–T020) | 0 |
| Phase 5 — US3 (P3) | 5 (T021–T025) | 2 |
| Phase 6 — Polish | 5 (T026–T030) | 3 |
| **Total** | **30** | **9** |
