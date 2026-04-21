# Data Model: 009-encoder-auto-cal-brat-on

**Phase**: 1 — Design
**Date**: 2026-03-06
**Depends on**: research.md

---

## Entities

### CalibrationParams

Per-wheel calibration correction factors, loaded from NVS at boot and applied
to the velocity pipeline. Shared globally across `encoder.c`, `velocity_subscriber.c`,
`calibration.c`, and `status_reporter.c`.

| Field | Type | Default | Notes |
|-------|------|---------|-------|
| `dir_sign[4]` | `int8_t[4]` | `[+1,+1,+1,+1]` | Per-wheel direction multiplier. +1 = no inversion; -1 = invert encoder output. Index = MotorChannel (FL=0,FR=1,RL=2,RR=3). |
| `speed_scale[4]` | `float[4]` | `[1.0,1.0,1.0,1.0]` | Per-wheel speed scaling factor, range [0.5, 2.0]. Applied to incoming velocity commands before PWM conversion. |
| `calibrated` | `bool` | `false` | true when at least direction calibration data was loaded from NVS (not defaults). Used for logging only. |

**Declared in**: `calibration.h` as a typedef struct.
**Defined as global in**: `calibration.c`:
```c
volatile CalibrationParams g_cal_params;
```
**Initialised by**: `calibration_params_load()` — called in `app_main` before
`encoder_start()`.

---

### CalibrationResult

Transient output of a single-wheel calibration run. Populated during `calibration_run()`,
not persisted beyond the function call. Used to compute storage values and log results.

| Field | Type | Notes |
|-------|------|-------|
| `wheel_id` | `MotorChannel` (0–3) | Which wheel was calibrated |
| `detected_direction` | `int8_t` | +1 or -1: sign of encoder count after forward burst |
| `measured_speed_low` | `float` | rad/s at `CONFIG_CAL_DUTY_LOW_PCT` (30%) |
| `measured_speed_high` | `float` | rad/s at `CONFIG_CAL_DUTY_HIGH_PCT` (70%) |
| `computed_scale` | `float` | Mean of (commanded/measured) at both duty levels; clamped to [0.5, 2.0] |
| `valid` | `bool` | false if encoder produced zero counts (motor/encoder fault) |

---

### NVS Calibration Keys

Stored under existing `rover_cfg` NVS namespace. All keys are 9 chars (well under 15-char limit).

| Key | NVS type | C type | Default | Notes |
|-----|----------|--------|---------|-------|
| `cal_dir_0` | `NVS_TYPE_I8` | `int8_t` | absent → +1 | Front-Left direction sign |
| `cal_dir_1` | `NVS_TYPE_I8` | `int8_t` | absent → +1 | Front-Right direction sign |
| `cal_dir_2` | `NVS_TYPE_I8` | `int8_t` | absent → +1 | Rear-Left direction sign |
| `cal_dir_3` | `NVS_TYPE_I8` | `int8_t` | absent → +1 | Rear-Right direction sign |
| `cal_spd_0` | `NVS_TYPE_U32` | `float` (bit-cast) | absent → 1.0 | Front-Left speed scale |
| `cal_spd_1` | `NVS_TYPE_U32` | `float` (bit-cast) | absent → 1.0 | Front-Right speed scale |
| `cal_spd_2` | `NVS_TYPE_U32` | `float` (bit-cast) | absent → 1.0 | Rear-Left speed scale |
| `cal_spd_3` | `NVS_TYPE_U32` | `float` (bit-cast) | absent → 1.0 | Rear-Right speed scale |

**Storage cost**: 12 NVS entries out of ~756 available. No capacity concern.

---

### FirmwareStatus (extension)

The existing `FirmwareStatus` struct in `status_reporter.h` gains two new fields (US3).

**Existing fields** (unchanged):
```c
float    commanded_speeds[4];
bool     watchdog_timed_out;
bool     motor_faults[4];
uint64_t uptime_ms;
uint32_t malformed_msg_count;
// + encoder_counts[4], encoder_velocities[4], encoder_faults[4]  ← from feature 008
```

**New fields** (added by this feature):
| Field | Type | Notes |
|-------|------|-------|
| `cal_direction[4]` | `int8_t[4]` | Active direction signs (+1 or -1 per wheel) |
| `cal_speed_scale[4]` | `float[4]` | Active speed scale factors per wheel |

**JSON extension** (added to `status_serialize()`):
```json
"cal_direction":   [1, -1, 1, 1],
"cal_speed_scale": [1.02, 0.98, 1.05, 0.97]
```

---

## State Transitions

### Calibration Lifecycle

```
UNCALIBRATED (defaults in use)
  │  calibration_params_load() — NVS absent → default +1 and 1.0 loaded
  │  g_cal_params.calibrated = false
  ▼
DEFAULTS_LOADED
  │  calibration_run() — triggered by GPIO or CONFIG_CALIBRATE_ON_BOOT
  ▼
DIRECTION_PASS (4 wheels × 1 burst)
  │  For each wheel:
  │    motor_set_pwm → vTaskDelay(350ms) → read g_encoder_velocities[i]
  │    → detect sign → motor_stop_all() → vTaskDelay(150ms)
  │  Writes cal_dir_0..3 to NVS; updates g_cal_params.dir_sign[]
  ▼
SPEED_PASS (4 wheels × 2 duty levels)  [US2]
  │  For each wheel, at 30% and 70% duty:
  │    motor_set_pwm → vTaskDelay(350ms) → read g_encoder_velocities[i]
  │    → compute scale → motor_stop_all() → vTaskDelay(150ms)
  │  Writes cal_spd_0..3 to NVS; updates g_cal_params.speed_scale[]
  ▼
CALIBRATED
  │  g_cal_params.calibrated = true
  │  All subsequent velocity computations and commands use stored params
  │
  │  [calibration reset: GPIO trigger or ROS 2 service /calibration_reset]
  ▼
UNCALIBRATED (defaults restored in memory + NVS keys erased)
```

**Key invariant**: `g_cal_params` is always in a valid state. Even if NVS is absent
or corrupt, defaults are applied and the rover is operational.

---

## Velocity Pipeline (with calibration applied)

```
Incoming command (/wheel_velocities_cmd_f32)
  └── velocity_callback()
        ├── clamp_speed(msg[i])
        ├── × g_cal_params.speed_scale[i]       ← [NEW] speed correction applied here
        ├── speed_to_duty()
        └── motor_set_pwm()

GPTimer ISR (encoder.c velocity_sample_cb)
  └── Δcount → ω (raw rad/s)
        ├── × g_cal_params.dir_sign[i]           ← [NEW] direction correction applied here
        ├── noise_floor check
        ├── clamp to MAX_SPEED_RAD_S
        └── g_encoder_velocities[i]
              └── wheel_publisher timer → /wheel_velocities
```

---

## Relationships

```
app_main
  ├── motor_init() + motor_stop_all()        [existing]
  ├── nvs_config_load()                      [existing]
  ├── calibration_params_load()              [NEW — loads g_cal_params from NVS]
  ├── encoder_init() + encoder_start()       [feature 008]
  ├── [calibration trigger check]            [NEW]
  ├── calibration_run()                      [NEW — optional, blocking]
  └── [retry loop]
        ├── velocity_subscriber_init()       [existing, modified for speed scale]
        ├── status_reporter_init()           [existing, modified for cal fields]
        ├── wheel_publisher_init()           [feature 008]
        └── calibration_reset_service_init() [NEW — US3/P3]

g_cal_params (global CalibrationParams)
  ├── written by: calibration_params_load(), calibration_run(), reset callback
  ├── read by:    encoder.c ISR (dir_sign), velocity_subscriber.c (speed_scale),
  │               status_reporter.c (both fields for JSON)
  └── persisted in: NVS "rover_cfg" namespace (8 keys)
```

---

## Shared Global (ISR-safe)

```c
volatile CalibrationParams g_cal_params;
```

Declared in `calibration.h`, defined in `calibration.c`.

The GPTimer ISR reads `g_cal_params.dir_sign[i]` on every tick. This is a single
`int8_t` read — atomic on Xtensa LX7 (aligned byte reads are atomic). No critical
section needed for direction sign reads.

The `speed_scale[4]` float array is read only in the executor task context
(velocity_callback), never from ISR context. No concurrent write possible — scale
is written only in `calibration_run()` (pre-loop, before ISR is running on scale)
or the reset callback (in executor task). Safe without mutex.

---

## Validation Rules

| Rule | Entity | Condition |
|------|--------|-----------|
| Direction must be ±1 | CalibrationParams | `dir_sign[i] == +1 || dir_sign[i] == -1` |
| Scale in valid range | CalibrationParams | `0.5f ≤ speed_scale[i] ≤ 2.0f` |
| Zero-count → skip | CalibrationResult | `valid = false` if encoder count = 0 after burst |
| Clamp before store | CalibrationResult | `computed_scale = fmaxf(0.5f, fminf(2.0f, raw_scale))` |
