# Research: 009-encoder-auto-cal-brat-on

**Phase**: 0 — Unknowns resolved before design
**Date**: 2026-03-06

---

## R-001: Calibration Trigger Mechanism

**Question**: How to trigger calibration without a ROS 2 agent? Which GPIO is safe for
a hold-at-boot button? How does the Kconfig `CONFIG_CALIBRATE_ON_BOOT` pattern work?

**Decision**: Two trigger paths — (a) `CONFIG_CALIBRATE_ON_BOOT` auto-runs when NVS
keys are absent; (b) **GPIO 22** held LOW at boot (active-low, internal pull-up) forces
calibration unconditionally. Both are detected in `app_main()` after `motor_init()` and
before the micro-ROS retry loop. Safe window: TWDT configured but `app_main` task NOT
yet subscribed — no watchdog risk.

**GPIO 22 rationale**: All other safe digital GPIOs on ESP32-S3-WROOM-1 are taken:
strapping (0,3,45,46), USB-JTAG (19,20), SPI flash/PSRAM (26–37), motors (4–7,15–18),
encoders (8–14,21). GPIO 22 (and 23 as backup) are free.

**NVS absence check**: `nvs_get_i8(h, "cal_dir_0", &dummy)` returning
`ESP_ERR_NVS_NOT_FOUND` is sufficient — keys are written atomically at end of
calibration, so absence of one implies absence of all.

**Kconfig block**: New `menu "Rover Calibration"` in `Kconfig.projbuild` with:
- `CONFIG_CALIBRATE_ON_BOOT` (bool, default n)
- `CONFIG_CAL_TRIGGER_GPIO` (int, default 22)
- `CONFIG_CAL_BURST_MS` (int, default 500)
- `CONFIG_CAL_DUTY_LOW_PCT` (int, default 30) — direction detection + speed point 1
- `CONFIG_CAL_DUTY_HIGH_PCT` (int, default 70) — speed point 2

**app_main placement** (after `motor_init()`, before `uros_transport_hw_init()`):
```c
#if CONFIG_CALIBRATE_ON_BOOT
    bool run_cal = cal_gpio_trigger_active() || cal_nvs_keys_absent();
#else
    bool run_cal = cal_gpio_trigger_active();
#endif
if (run_cal) calibration_run();
```

**Alternatives considered**:
- Serial command over USB CDC: rejected — TinyUSB not yet initialised at calibration
  window; sharing the CDC pipe with micro-ROS would require a protocol mux.
- ROS 2 service only: rejected — requires agent connection; useless at commissioning.

---

## R-002: TWDT Safety During Blocking Calibration

**Question**: Does a 6-second blocking calibration sequence risk a TWDT (task watchdog
timer) panic?

**Decision**: **No risk — no TWDT changes needed.** The `app_main` task is not subscribed
to TWDT until `esp_task_wdt_add(NULL)` inside the while-loop after `uros_init()` succeeds.
Calibration runs before this subscription. `vTaskDelay()` yields the scheduler, so IDLE
tasks on both cores continue to feed the TWDT normally.

**Key constraint**: Do NOT call `esp_task_wdt_add(NULL)` before `calibration_run()`.
The placement is already correct by the existing `app_main` structure.

**Total calibration time budget**: 4 wheels × (1 direction pass + 2 speed passes) ×
500 ms per pass = ~6 s worst case, well within SC-004 target of 30 s.

---

## R-003: Speed Calibration Settling Time

**Question**: How long to wait after commanding a duty level before reading encoder
velocity for the GB37-520?

**Decision**: **350 ms** (300 ms from 5× mechanical time constant + 50 ms margin).

**Derivation**: GB37-520 with 45:1 gearbox has output shaft mechanical time constant
τ_m ≈ 60–80 ms (empirical for this motor class). 5×τ_m = 300–400 ms for 99.3% settling.
Using 350 ms ensures steady-state is reached across manufacturing variation.

**Practical timing per measurement point**:
- `motor_set_pwm(ch, 0, duty)` → `vTaskDelay(350ms)` → read `g_encoder_velocities[i]`
  → `motor_stop_all()` → `vTaskDelay(150ms)` (coast-down between wheels)

**SNR validation at 30% duty** (direction detection):
- 30% of 1023 ticks → ~5.6 rad/s → Δcount ≈ 35 ticks per 20 ms PCNT interval
- Noise floor = 2 ticks → SNR ≈ 17× — reliable direction detection on first sample

**SNR at 70% duty** (speed profiling): ~13 rad/s → Δcount ≈ 82 ticks per interval.

**Implementation note**: Read `g_encoder_velocities[i]` (the feature 008 volatile global)
— do NOT read raw PCNT counts in `calibration.c`. This inherits noise-floor suppression
for free and avoids a second PCNT dependency in the calibration module.

---

## R-004: NVS Storage — int8 and Float Keys

**Question**: Which NVS API for int8 direction signs? Which for float scale factors?
How to selectively erase calibration keys?

**Decision — int8 (direction signs)**: `nvs_set_i8` / `nvs_get_i8`. Type-safe, 1 entry,
correct for signed ±1 values. Available in ESP-IDF v5.x `nvs.h`.

**Decision — float (scale factors)**: `nvs_set_u32` + `memcpy` (IEEE-754 bit cast).
Prefer `u32` over `nvs_set_blob` for new keys — uses 1 NVS entry vs 2 for blob,
halving storage cost. Pattern:
```c
float f = 1.0f; uint32_t bits; memcpy(&bits, &f, 4); nvs_set_u32(h, key, bits);
uint32_t bits; nvs_get_u32(h, key, &bits); memcpy(&f, &bits, 4);
```
Existing `max_speed_rads` key uses blob — keep blob for backward compat; new `cal_spd_*`
keys use u32.

**Decision — selective erase**: `nvs_erase_key(h, key)` per key, single `nvs_commit`
after all 8 erases. `ESP_ERR_NVS_NOT_FOUND` is non-fatal (key never written) — treat
as success.

**Key names — all valid**: `cal_dir_0`–`cal_dir_3`, `cal_spd_0`–`cal_spd_3` are all
9 characters. NVS hard limit is 15 characters. ✅

**Capacity**: 8 new keys cost ~12 NVS entries (4 i8 + 4 u32) out of ~756 available in
the default 24 KB NVS partition. Existing keys use ~5 entries. Total ~17 entries = 2%
utilisation. No concern.

---

## R-005: micro-ROS Service for Calibration Reset (US3)

**Question**: How to implement a micro-ROS service for calibration reset? Does it
require increasing `RMW_UXRCE_MAX_SERVICES`?

**Decision**: Use `std_srvs/srv/Empty` (not `Trigger` — avoids heap string allocation in
response). API:
1. `rclc_service_init_default(&svc, &node, TYPE_SUPPORT, "/calibration_reset")`
2. `rclc_executor_add_service(&executor, &svc, &req, &res, callback)`

**RMW_UXRCE_MAX_SERVICES**: Currently `1` in `colcon.meta`. Adding one service exactly
fills the slot — **no change to colcon.meta needed**. The T002 rebuild from feature 008
(which changes MAX_PUBLISHERS 2→3) will pick up the existing MAX_SERVICES=1 correctly.

**Executor capacity**: Feature 008 raises capacity to 4. This feature adds 1 more for
the service → capacity must be **5** in `uros_transport.c`. (Only needed for US3/P3.)

**Threading**: Service callback runs synchronously in the executor spin loop — same
task as subscriptions and timers. No mutex needed for `CalibrationParams` mutation.
NVS operations (~5 ms) are safe without TWDT concern.

**Alternatives considered**:
- `std_srvs/Trigger`: rejected — `Trigger.Response.message` is a `rosidl_runtime_c__String`
  requiring heap allocation; unnecessary overhead for a no-parameter reset.
- FreeRTOS notification via GPIO ISR: rejected — adds concurrency; service is P3 so
  micro-ROS availability is acceptable.

---

## R-006: Interaction with Feature 008 Encoder Globals

**Question**: How does the calibration routine safely read `g_encoder_velocities[]`
during the blocking calibration phase (before the micro-ROS executor is running)?

**Decision**: Safe — the GPTimer ISR from feature 008 runs independently of the executor.
`encoder_init()` and `encoder_start()` are called once before the retry loop in `app_main`.
After `encoder_start()`, the GPTimer fires every 20 ms and writes `g_encoder_velocities[]`
continuously. The calibration routine (also before the retry loop) can read these globals
after a 350 ms settle — the ISR has had 17+ timer ticks to stabilise the velocity reading.

**Ordering in app_main** (with both feature 008 and 009):
```
motor_init()
motor_stop_all()
TWDT configure
nvs_config_load()
calibration_params_load()    ← 009: load stored params into g_cal_params
encoder_init()               ← 008: PCNT units configured, not yet counting
encoder_start()              ← 008: PCNT starts counting, GPTimer ISR starts
[calibration trigger check]  ← 009: GPIO + NVS key check
calibration_run()            ← 009: blocking; reads g_encoder_velocities[] after settle
watchdog_init()
uros_transport_hw_init()
[retry loop]
```

**Key invariant**: `g_cal_params` must be loaded BEFORE `encoder_start()` so the
direction signs are in place when the first ISR fires. Velocity output from the very
first GPTimer callback will already be sign-corrected.

---

## R-007: Scale Factor Application Point

**Question**: Where in the existing velocity pipeline should the speed scaling factor
be applied — in `encoder.c` (velocity output) or `velocity_subscriber.c` (command input)?

**Decision**: **Two separate application points**:
- **Direction sign** (`g_cal_params.dir_sign[i]`): applied in `encoder.c`
  `velocity_sample_cb()` when computing `ω`, BEFORE writing to `g_encoder_velocities[]`.
  This ensures the published `/wheel_velocities` topic always reflects physically correct
  direction.
- **Speed scale factor** (`g_cal_params.speed_scale[i]`): applied in
  `velocity_subscriber.c` `velocity_callback()`, AFTER receiving the command and BEFORE
  `speed_to_duty()`. This corrects the commanded speed to account for per-wheel motor
  variance.

**Rationale**: Keeping corrections close to their data source (encoder output for
direction, command input for speed) minimises the blast radius of changes and aligns
with the HAL principle — calibration is part of the hardware translation layer, not
the kinematic layer.

**Alternatives considered**: Applying scale in `encoder.c` to the velocity output:
rejected — then the `/wheel_velocities` topic would show scale-corrected values, making
it harder to diagnose the raw hardware behaviour. The controller's feedback loop should
see real measured velocities, not pre-scaled ones; the scale correction belongs on the
command side.
