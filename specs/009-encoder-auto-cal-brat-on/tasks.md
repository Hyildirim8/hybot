# Tasks: Encoder Auto-Calibration (009-encoder-auto-cal-brat-on)

**Input**: Design documents from `specs/009-encoder-auto-cal-brat-on/`
**Branch**: `009-encoder-auto-cal-brat-on`
**Prerequisites**: Feature 008 (`encoder-feedback`) merged — `g_encoder_velocities[]` globals and PCNT hardware must be present.
**Tests**: Not requested — manual hardware verification via quickstart.md.
**Organization**: Tasks grouped by user story to enable independent delivery and testing.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: User story label (US1, US2, US3) — present only in story phases

---

## Phase 1: Setup

**Purpose**: Add new files and extend build system so subsequent phases compile.

- [X] T001 Add `calibration.c` stub (empty `void calibration_run(void){}` etc.) to `firmware/main/CMakeLists.txt` SRCS list — only `.c` files go in SRCS; `calibration.h` is **not** listed there (ESP-IDF build error if added)
- [X] T002 [P] Add `menu "Rover Calibration"` block to `firmware/main/Kconfig.projbuild` with symbols: `CONFIG_CALIBRATE_ON_BOOT` (bool, default n), `CONFIG_CAL_TRIGGER_GPIO` (int, default 22), `CONFIG_CAL_BURST_MS` (int, default 500), `CONFIG_CAL_DUTY_LOW_PCT` (int, default 30), `CONFIG_CAL_DUTY_HIGH_PCT` (int, default 70)
- [X] T003 [P] Add NVS key constants `NVS_KEY_CAL_DIR_0..3` and `NVS_KEY_CAL_SPD_0..3` to `firmware/main/nvs_config.h`

**Checkpoint**: `idf.py build` compiles with stub files — no linker errors.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: `CalibrationParams` struct, global, and NVS load/default logic — required by ALL user stories before their changes to encoder.c, velocity_subscriber.c, app_main.c can compile correctly.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T004 Create `firmware/main/calibration.h` with `CalibrationParams` typedef struct (`int8_t dir_sign[4]`, `float speed_scale[4]`, `bool calibrated`), `extern volatile CalibrationParams g_cal_params` declaration, and function prototypes: `calibration_params_load()`, `calibration_run()`, `calibration_reset_callback()`, `cal_gpio_trigger_active()`, `cal_nvs_keys_absent()`
- [X] T005 Create `firmware/main/calibration.c` with `volatile CalibrationParams g_cal_params` definition and implement `calibration_params_load()`: open NVS `rover_cfg` handle, call `nvs_get_i8` for each `cal_dir_N` key (default +1 on `ESP_ERR_NVS_NOT_FOUND`); track a `bool all_dirs_found = true` flag — clear it if ANY key returns `ESP_ERR_NVS_NOT_FOUND`; call `nvs_get_u32`+memcpy for each `cal_spd_N` key (default 1.0f on absent); set `g_cal_params.calibrated = all_dirs_found` (true only when ALL 4 direction keys were loaded — partial NVS corruption treated as uncalibrated); close handle. Note: `cal_nvs_keys_absent()` (T006) uses a single-key probe as a fast "never calibrated" check — these two functions serve different purposes and are consistent.
- [X] T006 Implement `cal_nvs_keys_absent()` in `firmware/main/calibration.c`: open NVS handle, call `nvs_get_i8(h, "cal_dir_0", &dummy)`, return true if result is `ESP_ERR_NVS_NOT_FOUND`, close handle
- [X] T007 Implement `cal_gpio_trigger_active()` in `firmware/main/calibration.c`: configure `CONFIG_CAL_TRIGGER_GPIO` as input with internal pull-up using `gpio_config()`, read level, return true if level is 0 (active-low)
- [X] T008 Add `calibration_params_load()` call to `firmware/main/app_main.c` in the boot sequence AFTER `nvs_config_load()` and BEFORE `encoder_init()` (per R-006 ordering); add calibration trigger check block using `cal_gpio_trigger_active()` and `cal_nvs_keys_absent()` per R-001 pattern, calling stub `calibration_run()` placeholder

**Checkpoint**: `idf.py build` succeeds. On boot, serial log shows `calibration_params_load` completed with default values. `g_cal_params.dir_sign` = all +1, `speed_scale` = all 1.0.

---

## Phase 3: User Story 1 — Automatic Encoder Direction Calibration (P1) 🎯 MVP

**Goal**: Firmware detects per-wheel encoder direction at boot/on-demand, stores to NVS, and applies direction sign to velocity output so all wheels report positive values when driven forward.

**Independent Test**: Trigger calibration (GPIO 22 or `CONFIG_CALIBRATE_ON_BOOT=y`). Drive forward. All four `/wheel_velocities` values are positive. Re-trigger after physically swapping encoder cables — updated sign is stored and applied on next boot.

- [X] T009 [P] [US1] Apply direction sign in `firmware/main/encoder.c` `velocity_sample_cb()`: after computing raw `ω` (rad/s from Δcount and sample period), multiply by `g_cal_params.dir_sign[channel]` before the noise-floor check and before writing to `g_encoder_velocities[channel]`
- [X] T010 [US1] Implement direction calibration pass in `calibration_run()` in `firmware/main/calibration.c`: for each wheel (FL→FR→RL→RR): call `motor_set_pwm(ch, 0, duty_forward)` where `duty_forward = CONFIG_CAL_DUTY_LOW_PCT * PWM_MAX_TICKS / 100`, call `vTaskDelay(pdMS_TO_TICKS(350))`, read `g_encoder_velocities[ch]`, detect sign as `(vel >= 0.0f) ? +1 : -1`, call `motor_stop_all()`, call `vTaskDelay(pdMS_TO_TICKS(150))`, log result per FR-009 (`ESP_LOGI(TAG, "CAL: wheel %d direction %+d", ch, sign)`)
- [X] T011 [US1] After direction pass loop in `calibration_run()`, write all 4 direction signs to NVS in `firmware/main/calibration.c`: open `rover_cfg` handle, call `nvs_set_i8(h, NVS_KEY_CAL_DIR_N, sign)` for each wheel, call single `nvs_commit(h)` after all 4 writes (FR-002, R-004 atomicity), update `g_cal_params.dir_sign[]` and `g_cal_params.calibrated = true`, handle `ESP_ERR_NVS_NOT_FOUND` non-fatally per edge case spec
- [X] T012 [US1] Handle zero-count wheel in direction pass in `firmware/main/calibration.c`: if `g_encoder_velocities[ch] == 0.0f` after settle, log `ESP_LOGW(TAG, "CAL: wheel %d zero count — skipping, default +1")`, keep `dir_sign[ch] = +1`, set `CalibrationResult.valid = false` for that wheel, continue with next wheel

**Checkpoint**: US1 fully functional. Trigger calibration → all four `/wheel_velocities` positive on forward drive. Direction survives power cycle (SC-003). No micro-ROS agent needed (FR-006, edge case).

---

## Phase 4: User Story 2 — Speed Profile Calibration (P2)

**Goal**: Firmware measures per-wheel speed at two duty levels, computes a scaling factor, stores to NVS, and applies it to incoming velocity commands so all wheels reach equal speed within ±5%.

**Independent Test**: Run calibration (includes speed pass). Command all wheels at 5.0 rad/s. Read `/wheel_velocities` — all four values within ±5% of each other (SC-002).

- [X] T013 [US2] Apply speed scale in `firmware/main/velocity_subscriber.c` `velocity_callback()`: after `clamp_speed(msg->data.data[i])`, multiply the clamped speed by `g_cal_params.speed_scale[i]` before passing to `speed_to_duty()` (R-007 application point). No second `clamp_speed()` call is needed — `speed_to_duty()` already clamps its result to `[0, PWM_MAX_TICKS]`, so an over-scale (e.g., scale=2.0 × 18.75 rad/s = 37.5 rad/s) saturates at full duty rather than exceeding hardware limits. This is the intended behaviour.
- [X] T014 [US2] Implement speed calibration pass in `calibration_run()` in `firmware/main/calibration.c` (appended after direction pass): for each wheel: (a) run LOW duty burst — `motor_set_pwm(ch, 0, low_duty)`, `vTaskDelay(pdMS_TO_TICKS(350))`, record `measured_low = fabsf(g_encoder_velocities[ch])`; **then** `motor_stop_all()`, `vTaskDelay(pdMS_TO_TICKS(150))` (coast-down between LOW and HIGH bursts, same wheel); (b) run HIGH duty burst — `motor_set_pwm(ch, 0, high_duty)`, `vTaskDelay(pdMS_TO_TICKS(350))`, record `measured_high = fabsf(g_encoder_velocities[ch])`; (c) `motor_stop_all()`, `vTaskDelay(pdMS_TO_TICKS(150))` (inter-wheel coast-down); compute `commanded_low = CONFIG_CAL_DUTY_LOW_PCT/100.0f * MAX_SPEED_RAD_S` and `commanded_high = CONFIG_CAL_DUTY_HIGH_PCT/100.0f * MAX_SPEED_RAD_S`; compute `raw_scale = (commanded_low/measured_low + commanded_high/measured_high) / 2.0f`; log `ESP_LOGI(TAG, "CAL: wheel %d speed low=%.2f high=%.2f scale=%.3f", ch, measured_low, measured_high, raw_scale)` per FR-009
- [X] T015 [US2] Clamp and store speed scale in `calibration_run()` in `firmware/main/calibration.c`: apply `computed_scale = fmaxf(0.5f, fminf(2.0f, raw_scale))`, if clamping occurred emit `ESP_LOGW(TAG, "CAL: wheel %d scale clamped %.2f→%.2f — check mechanism", ch, raw_scale, computed_scale)` per FR-012; handle zero measured speed (stall) by clamping to 0.5f with WARN; after all 4 wheels write `nvs_set_u32`+memcpy for each `cal_spd_N` key, single `nvs_commit()`, update `g_cal_params.speed_scale[]`

**Checkpoint**: US2 fully functional. Post-calibration, equal commands produce equal measured velocities within ±5% (SC-002). Speed scaling survives reboot (SC-003).

---

## Phase 5: User Story 3 — Calibration Status Visibility and Reset (P3)

**Goal**: `/firmware_status` JSON exposes active calibration parameters; `/calibration_reset` service restores factory defaults without full NVS erase.

**Independent Test**: Read `/firmware_status` — JSON includes `cal_direction[4]` and `cal_speed_scale[4]`. Call `/calibration_reset` service — next status shows all +1 direction and 1.0 scale (SC-006).

- [X] T016 [P] [US3] Add `int8_t cal_direction[4]` and `float cal_speed_scale[4]` fields to `FirmwareStatus` struct in `firmware/main/status_reporter.h`
- [X] T017 [P] [US3] In `status_reporter.c`: (1) in `status_reporter_update()` (or equivalent populate function) copy `g_cal_params.dir_sign[i]` → `status->cal_direction[i]` and `g_cal_params.speed_scale[i]` → `status->cal_speed_scale[i]` for i=0..3, mirroring how `commanded_speeds` and `motor_faults` are populated; (2) in `status_serialize()` append `"cal_direction":[%d,%d,%d,%d],"cal_speed_scale":[%.4f,%.4f,%.4f,%.4f]` reading from `status->cal_direction[]` and `status->cal_speed_scale[]` (not directly from `g_cal_params`) — consistent with contracts/topics.md full schema
- [X] T018 [US3] Implement `calibration_reset_callback()` in `firmware/main/calibration.c` with signature `void calibration_reset_callback(const void *req, void *res)` (rclc service callback — `req`/`res` are cast to `std_srvs__srv__Empty_Request*` / `std_srvs__srv__Empty_Response*` but can be ignored for Empty type): open `rover_cfg` NVS handle, call `nvs_erase_key()` for each of 8 keys (`cal_dir_0..3`, `cal_spd_0..3`) treating `ESP_ERR_NVS_NOT_FOUND` as success, call single `nvs_commit()`, reset `g_cal_params.dir_sign[]` to all +1, reset `g_cal_params.speed_scale[]` to all 1.0f, set `g_cal_params.calibrated = false`, close handle
- [X] T019 [US3] **Prerequisite**: verify `rclc_executor_init(..., 4, ...)` is already in place in `firmware/main/uros_transport.c` (from 008 T005 — current disk value is `3` and must be at `4` first). Then: declare `rcl_service_t reset_service` and `std_srvs__srv__Empty_Request/Response` variables; increment executor capacity from **4 to 5** in `rclc_executor_init()` call; call `rclc_service_init_default(&reset_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty), "/calibration_reset")` in `uros_init()`; call `rclc_executor_add_service(&executor, &reset_service, &reset_request, &reset_response, calibration_reset_callback)` (contracts/topics.md registration pattern)

**Checkpoint**: US3 fully functional. `/firmware_status` JSON shows calibration arrays. `ros2 service call /calibration_reset std_srvs/srv/Empty` returns, next status shows all defaults.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Hardening, logging completeness, and quickstart validation.

- [X] T020 [P] Add `#include "calibration.h"` to `firmware/main/encoder.c`, `firmware/main/velocity_subscriber.c`, and `firmware/main/app_main.c`; verify no implicit declaration warnings in `idf.py build` output
- [X] T021 [P] Verify `firmware/main/app_main.c` boot sequence matches R-006 ordering exactly: `motor_init` → `motor_stop_all` → TWDT configure → `nvs_config_load` → `calibration_params_load` → `encoder_init` → `encoder_start` → calibration trigger check → `calibration_run` (if triggered) → `watchdog_init` → `uros_transport_hw_init` → retry loop
- [X] T022 Run quickstart.md US1 verification: trigger calibration, drive forward, confirm all `/wheel_velocities` positive (SC-001)
- [X] T023 Run quickstart.md US2 verification: command equal speeds, confirm ±5% spread across wheels (SC-002)
- [X] T024 Run quickstart.md US3 verification: inspect `/firmware_status` JSON fields, call `/calibration_reset` service, confirm defaults restored (SC-006)
- [X] T025 [P] Verify NVS persistence across reboot: after calibration, power-cycle rover, confirm `/wheel_velocities` signs and speed uniformity unchanged (SC-003)
- [X] T026 [US1] Add motion guard at the start of `calibration_run()` in `firmware/main/calibration.c`: check `fabsf(g_encoder_velocities[i]) > 0.5f` for any wheel (0.5 rad/s > noise floor ~0.1 rad/s); if any wheel is moving, log `ESP_LOGE(TAG, "CAL: rover not stationary — calibration aborted")` and return without running any motor bursts (edge case: calibration during motion)
- [X] T027 [P] Run SC-005 straight-line drift test: after calibration applied, drive 1 m forward and measure lateral deviation — must be ≤ 5 cm (SC-005); document result in test notes
- [X] T028 [P] Align ROS mecanum sign conventions between the standalone simulation path and the hardware path so RViz wheel motion matches encoder feedback for strafe and rotation
- [X] T029 [P] Harden hardware synchronization path: disable synthetic command loopback by default in `src/ecza_description/scripts/wheel_bridge.py`, switch `config/controllers.yaml` `open_loop` to `false` for encoder-based odometry, and reduce command callback logging pressure in `firmware/main/velocity_subscriber.c`

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)           → no dependencies
Phase 2 (Foundational)    → depends on Phase 1 — BLOCKS Phases 3, 4, 5
Phase 3 (US1 — P1 MVP)    → depends on Phase 2
Phase 4 (US2 — P2)        → depends on Phase 2; integrates with Phase 3 (calibration_run extension)
Phase 5 (US3 — P3)        → depends on Phase 2; independent of Phases 3 and 4
Phase 6 (Polish)          → depends on all desired stories complete
```

### User Story Dependencies

- **US1 (P1)**: Requires Phase 2. Standalone once T004–T008 done. encoder.c change (T009) independent of US2/US3.
- **US2 (P2)**: Requires Phase 2. T013 (velocity_subscriber.c) independent of US1 tasks. T014–T015 extend `calibration_run()` body — must follow US1 T010–T011 (speed pass appended after direction pass in same function).
- **US3 (P3)**: Requires Phase 2 (for `g_cal_params`). T016–T017 (status_reporter) independent of US1/US2. T018–T019 (service) independent of US1/US2.

### Parallel Opportunities

Within Phase 1: T002 and T003 can run in parallel (different files).
Within Phase 2: T004–T007 can all run in parallel (new files or isolated sections); T008 depends on T004–T007.
Within Phase 3: T009 (encoder.c) can run in parallel with T010–T012 (calibration.c body).
Within Phase 5: T016 and T017 can run in parallel; T018 and T019 can run in parallel after T018 is declared.

---

## Parallel Example: Phase 2 (Foundational)

```
Parallel group A (all independent new-file work):
  T004 — Create calibration.h
  T005 — Create calibration.c with calibration_params_load()
  T006 — Implement cal_nvs_keys_absent() in calibration.c
  T007 — Implement cal_gpio_trigger_active() in calibration.c

Sequential gate:
  T008 — Wire calibration_params_load() into app_main.c (depends on T004–T007)
```

## Parallel Example: Phase 5 (US3)

```
Parallel group B (different files):
  T016 — Add fields to status_reporter.h
  T017 — Extend status_serialize() in status_reporter.c
  T018 — Implement calibration_reset_callback() in calibration.c

Sequential gate:
  T019 — Wire service into uros_transport.c (depends on T018 for callback symbol)
```

---

## Implementation Strategy

### MVP First (US1 Only — Phase 1 + 2 + 3)

1. Complete Phase 1: T001–T003
2. Complete Phase 2: T004–T008 (CRITICAL)
3. Complete Phase 3: T009–T012
4. **STOP and VALIDATE** using quickstart.md US1 section
5. Flash and demo: all wheels report positive velocity on forward command

### Incremental Delivery

- **After Phase 3 (MVP)**: direction-corrected `/wheel_velocities` ✅
- **Add Phase 4 (US2)**: speed-uniform rover, ±5% across wheels ✅
- **Add Phase 5 (US3)**: calibration visible in `/firmware_status`, reset service ✅
- **Phase 6**: verify NVS persistence and integration across all stories

### Single Developer Sequence

```
T001 → T002‖T003 → T004‖T005‖T006‖T007 → T008 →
T009‖T010 → T011 → T012 →               ← Phase 3 (US1 MVP)
T013 → T014 → T015 →                    ← Phase 4 (US2)
T016‖T017‖T018 → T019 →                 ← Phase 5 (US3)
T020‖T021 → T026 → T022 → T023 → T024 → T025‖T027   ← Phase 6 (Polish)
```

---

## Task Count Summary

| Phase | Tasks | User Story |
|-------|-------|-----------|
| Phase 1 — Setup | 3 (T001–T003) | — |
| Phase 2 — Foundational | 5 (T004–T008) | — |
| Phase 3 — US1 (P1 MVP) | 4 (T009–T012) | US1 |
| Phase 4 — US2 (P2) | 3 (T013–T015) | US2 |
| Phase 5 — US3 (P3) | 4 (T016–T019) | US3 |
| Phase 6 — Polish | 8 (T020–T027) | US1 (T026) |
| **Total** | **27** | |

**Parallel opportunities**: T002‖T003, T004‖T005‖T006‖T007, T009‖T010, T016‖T017‖T018, T025‖T027
**MVP scope**: T001–T012 + T026 (13 tasks, Phases 1–3 + motion guard)
