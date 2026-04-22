# Data Model: 008-encoder-feedback

**Phase**: 1 — Design
**Date**: 2026-03-06
**Depends on**: research.md

---

## Entities

### EncoderChannel

Represents a single physical quadrature encoder (one per wheel).
Owns one PCNT unit with two channels for full 4X decoding.

| Field | Type | Source | Notes |
|-------|------|--------|-------|
| `wheel_id` | `MotorChannel` (0–3) | compile-time | FL=0, FR=1, RL=2, RR=3 — matches existing motor convention |
| `pcnt_unit` | `pcnt_unit_handle_t` | allocated at init | one unit per wheel; ESP32-S3 has exactly 4 |
| `pcnt_chan_a` | `pcnt_channel_handle_t` | allocated at init | edge=pin_a, level=pin_b |
| `pcnt_chan_b` | `pcnt_channel_handle_t` | allocated at init | edge=pin_b, level=pin_a |
| `pin_a` | `int` (GPIO) | Kconfig / `encoder.h` defaults | FL:8, FR:10, RL:12, RR:14 |
| `pin_b` | `int` (GPIO) | Kconfig / `encoder.h` defaults | FL:9, FR:11, RL:13, RR:21 |
| `last_count` | `int` (int32) | updated each velocity sample | previous PCNT accumulator value |
| `fault` | `bool` | set by velocity sampler | true when computed velocity exceeds MAX_SPEED_RAD_S (implausible tick rate — clamp event). A disconnected pin produces zero counts, not a fault. |

**Initialisation invariant**: `pcnt_unit_get_count()` returns 0 immediately after
`pcnt_unit_clear_count()` and before `pcnt_unit_start()`. `last_count` is set to 0
at init so the first sample produces Δcount = 0 (zero initial velocity).

---

### EncoderConfig

Compile-time parameters shared by all four encoders. Set via Kconfig.

| Field | Type | Default | Kconfig symbol | Notes |
|-------|------|---------|----------------|-------|
| `cpr` | `int` | 1980 | `CONFIG_ENCODER_CPR` | Counts per output shaft revolution: 11 lines × 4 (quad) × 45 (gear) = 1980. Confirmed at bring-up. |
| `noise_floor_ticks` | `int` | 2 | `CONFIG_ENCODER_NOISE_FLOOR` | Δcount below this → report 0.0 rad/s |
| `sample_period_ms` | `int` | 20 | `CONFIG_ENCODER_SAMPLE_PERIOD_MS` | GPTimer alarm period; must match controller_manager update_rate (50 Hz) |
| `glitch_filter_ns` | `int` | 1000 | `CONFIG_ENCODER_GLITCH_NS` | PCNT hardware glitch filter; suppresses sub-1µs noise from PWM EMI |

---

### WheelVelocity

Derived output of the velocity sampler. Computed for all 4 wheels each period.
Published as `std_msgs/Float32MultiArray` on `/wheel_velocities`.

| Field | Type | Notes |
|-------|------|-------|
| `value_rad_s[4]` | `float[4]` | signed; index = MotorChannel; positive = forward |
| `clamped[4]` | `bool[4]` | true if raw velocity exceeded MAX_SPEED_RAD_S |
| `noise_suppressed[4]` | `bool[4]` | true if Δcount < noise_floor_ticks → output forced to 0.0 |

**Published field**: only `value_rad_s[4]` goes on the wire. `clamped` and
`noise_suppressed` are internal state used to populate `FirmwareStatus`.

---

### FirmwareStatus (extension)

The existing `FirmwareStatus` struct in `status_reporter.h` gains two new fields.

**Existing fields** (unchanged):
```c
float    commanded_speeds[4];
bool     watchdog_timed_out;
bool     motor_faults[4];
uint64_t uptime_ms;
uint32_t malformed_msg_count;
```

**New fields** (added by this feature):
| Field | Type | Notes |
|-------|------|-------|
| `encoder_counts[4]` | `int32_t[4]` | cumulative signed tick count per wheel |
| `encoder_faults[4]` | `bool[4]` | true when computed velocity exceeded MAX_SPEED_RAD_S (clamp event — implausible tick rate). Does NOT fire for open/disconnected pins (those show zero counts). |
| `encoder_velocities[4]` | `float[4]` | last computed rad/s per wheel (mirrors published values) |

**JSON extension** (added to `status_serialize()`):
```json
"encoder_counts":     [12345, 12300, 12350, 12280],
"encoder_velocities": [3.14, 3.12, 3.15, 3.10],
"encoder_faults":     [false, false, false, false]
```

---

## State Transitions

### Encoder Subsystem Lifecycle

```
UNINIT
  │  encoder_init() — PCNT units allocated, channels configured,
  │                   glitch filter set, watch points added, cleared
  ▼
READY (counting stopped)
  │  encoder_start() — pcnt_unit_enable() + pcnt_unit_start() for all 4 units
  ▼
COUNTING
  │  GPTimer fires every 20 ms → velocity_sample_cb() reads Δcount,
  │  computes ω, posts to g_encoder_velocities[], updates g_encoder_counts[]
  │
  │  wheel_publisher timer fires every 20 ms → reads g_encoder_velocities[],
  │  publishes /wheel_velocities
  │
  │  [micro-ROS disconnect]
  ▼
COUNTING (no publish)
  │  PCNT continues accumulating; GPTimer continues sampling;
  │  g_encoder_velocities[] updated in place; publisher timer inactive
  │  (executor not spinning)
  │
  │  [micro-ROS reconnect → create_entities()]
  ▼
COUNTING (publishing resumed)
  │
  │  [uros_fini() / reconnect teardown]
  ▼
COUNTING (no publish) ← encoder hardware always running, only publish stops
```

**Key invariant**: encoder hardware (PCNT + GPTimer) is initialised once in
`app_main` before the micro-ROS retry loop and is never stopped or re-initialised.
Only the micro-ROS publisher is torn down and recreated on reconnect.

---

## Relationships

```
app_main
  ├── motor_init()           [existing]
  ├── encoder_init()         [NEW — one-time, before retry loop]
  ├── encoder_start()        [NEW — one-time, starts PCNT counting]
  └── [retry loop]
        ├── velocity_subscriber_init()   [existing]
        ├── status_reporter_init()       [existing]
        └── wheel_publisher_init()       [NEW — registers timer in executor]

EncoderChannel[4]  ──owns──► pcnt_unit × 1
                             pcnt_chan_a × 1  (edge=A, level=B)
                             pcnt_chan_b × 1  (edge=B, level=A)
                   ──reads──► GPTimer alarm callback (ISR)
                              → Δcount → WheelVelocity.value_rad_s[]

wheel_publisher    ──reads──► g_encoder_velocities[4]  (volatile float[4])
                  ──publishes► /wheel_velocities (Float32MultiArray)

status_reporter    ──reads──► g_encoder_counts[4]      (volatile int32_t[4])
                              g_encoder_velocities[4]
                              g_encoder_faults[4]       (volatile bool[4])
```

---

## Shared Globals (ISR-safe)

The GPTimer ISR callback writes; the rclc_timer callbacks and status_reporter read.
All are accessed atomically (single 32-bit aligned write is atomic on Xtensa LX7):

```c
volatile float   g_encoder_velocities[4];  // rad/s, FL FR RL RR
volatile int32_t g_encoder_counts[4];      // cumulative ticks, FL FR RL RR
volatile bool    g_encoder_faults[4];      // implausible velocity flag
```

Declared in `encoder.h`, defined in `encoder.c`.

---

## Validation Rules

| Rule | Entity | Condition |
|------|--------|-----------|
| CPR must be positive | EncoderConfig | `cpr > 0` — asserted at init |
| Noise floor < 1 full encoder step | EncoderConfig | `noise_floor_ticks < cpr / gear_ratio * 4` — asserted at init |
| GPIO pins must not overlap motor pins | EncoderChannel | Compile-time: encoder GPIOs (8–14, 21) are disjoint from motor GPIOs (4–7, 15–18) |
| Max ticks per period | EncoderChannel | `MAX_TICKS = MAX_SPEED_RAD_S * cpr / (2π * 1000 / sample_period_ms)`; if `|Δcount| > MAX_TICKS * 1.5` → set `encoder_fault[i] = true`, clamp output |
