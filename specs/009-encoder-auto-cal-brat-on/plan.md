# Implementation Plan: 009-encoder-auto-cal-brat-on

**Branch**: `009-encoder-auto-cal-brat-on` | **Date**: 2026-03-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-encoder-auto-cal-brat-on/spec.md`

---

## Summary

Read encoder direction and speed at runtime; compute per-wheel correction factors;
persist to NVS for automatic application on every subsequent boot. Depends on
feature 008 (encoder feedback) for `g_encoder_velocities[]` globals and PCNT hardware.

**Technical approach**: Single blocking `calibration_run()` executed in the TWDT-safe
pre-subscription window. GPIO 22 trigger (active-low) + optional `CONFIG_CALIBRATE_ON_BOOT`
Kconfig flag. NVS keys `cal_dir_0..3` (int8) and `cal_spd_0..3` (u32+memcpy float)
under `rover_cfg` namespace. US3 (P3) adds `/calibration_reset` micro-ROS service
and cal fields to `/firmware_status` JSON.

---

## Technical Context

**Language/Version**: C17 (ESP-IDF v5.x, ESP32-S3-WROOM-1)
**Primary Dependencies**: ESP-IDF (NVS, GPTimer, LEDC), micro-ROS (rclc, rclc_executor, std_srvs/srv/Empty)
**Storage**: NVS flash — `rover_cfg` namespace, 8 new keys (12 entries total, 2% of 756 capacity)
**Testing**: Manual hardware verification (see quickstart.md) — no unit test framework in firmware
**Target Platform**: ESP32-S3-WROOM-1 bare-metal, FreeRTOS
**Project Type**: Embedded firmware component
**Performance Goals**: Calibration completes in ≤ 30 s (SC-004); ≤ 350 ms settling delay per wheel burst (R-003)
**Constraints**: TWDT-safe (no watchdog subscription during calibration); ISR-safe `int8_t` reads of `dir_sign`; scale clamp [0.5, 2.0]; NVS key ≤ 15 chars (all 9 chars ✅)
**Scale/Scope**: 4 wheels × 2 passes (direction + speed) × 2 duty levels = 16 motor bursts (~6 s total)

---

## Constitution Check

*Evaluated against `.specify/memory/constitution.md`*

| § | Rule | Status | Notes |
|---|------|--------|-------|
| I | Single responsibility per module | ✅ PASS | `calibration.c` handles only calibration logic; scale/direction application delegated to existing modules |
| II | No global mutable state without justification | ✅ PASS | `g_cal_params` is justified: read from ISR context by encoder.c; must be accessible without function-call overhead |
| III | NVS writes must be atomic (commit after all writes) | ✅ PASS | R-004: single `nvs_commit()` after all 4 key writes per pass |
| IV | Transport uses WiFi UDP | ⚠️ PRE-EXISTING | USB CDC-ACM transport (same as 008). Not introduced by this feature. See 008 plan.md §IV. |
| V | No blocking in ISR or timer callbacks | ✅ PASS | `calibration_run()` only in app_main task (pre-subscription); reset callback in executor task (NVS write safe) |
| VI | Kconfig for all hardware-specific constants | ✅ PASS | GPIO 22, burst ms, duty levels all behind Kconfig symbols with sensible defaults |

**Post-design re-check**: No new violations introduced. §IV PRE-EXISTING unchanged.

---

## Project Structure

### Documentation (this feature)

```text
specs/009-encoder-auto-cal-brat-on/
├── plan.md              ← This file
├── spec.md              ← Feature spec (US1–3, FR-001–012, SC-001–006)
├── research.md          ← Phase 0 output (R-001–R-007)
├── data-model.md        ← Phase 1 output
├── quickstart.md        ← Phase 1 output
├── contracts/
│   └── topics.md        ← Phase 1 output
└── tasks.md             ← Phase 2 output (NOT created by /speckit.plan)
```

### Source Code (firmware/main/)

```text
firmware/main/
├── calibration.h          [NEW] CalibrationParams struct, g_cal_params extern, API
├── calibration.c          [NEW] calibration_params_load, calibration_run,
│                                calibration_reset_callback, cal_gpio_trigger_active
├── encoder.c              [MOD 008→009] apply dir_sign in velocity_sample_cb ISR
├── velocity_subscriber.c  [MOD] apply speed_scale before speed_to_duty
├── app_main.c             [MOD] add calibration_params_load, encoder_init/start,
│                                calibration trigger check, calibration_run calls
├── status_reporter.h      [MOD 008→009] add cal_direction[4], cal_speed_scale[4]
├── status_reporter.c      [MOD 008→009] extend status_serialize JSON output
├── nvs_config.h           [MOD] add NVS_KEY_CAL_DIR_0..3, NVS_KEY_CAL_SPD_0..3
├── Kconfig.projbuild      [MOD] add "Rover Calibration" menu
├── CMakeLists.txt         [MOD] add calibration.c to SRCS
└── uros_transport.c       [MOD US3] executor capacity 4→5
```

**Structure Decision**: Single project — firmware is a monolithic ESP-IDF application.
All new calibration logic encapsulated in `calibration.h/.c`; integration points
(encoder.c, velocity_subscriber.c, status_reporter.h/.c) receive minimal targeted
modifications as documented in data-model.md.

---

## Complexity Tracking

No constitution violations introduced by this feature. §IV is pre-existing (008).
No justification table needed.
