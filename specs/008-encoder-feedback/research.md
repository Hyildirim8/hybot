# Research: 008-encoder-feedback

**Phase**: 0 — Unknowns resolved before design
**Date**: 2026-03-06

---

## R-001: ESP32-S3 PCNT Units — Availability and Quadrature Capability

**Question**: Does the ESP32-S3 have enough hardware PCNT units for 4 encoders? Can
one unit decode full A+B quadrature direction-aware?

**Decision**: YES — ESP32-S3 has exactly **4 PCNT units**, each with **2 channels**.
Four encoders use all 4 units exactly. Use both channels of each unit for 4X
quadrature decoding. Use `accum_count = 1` in `pcnt_unit_config_t` for unlimited
cumulative counting without overflow handling code.

**Rationale**: ESP-IDF v5.x `driver/pulse_cnt.h` supports direction-aware quadrature
using both channels of a single unit: channel A uses edge=A level=B, channel B uses
edge=B level=A. This gives ×4 resolution (all edges counted). The `accum_count=1`
flag enables an internal 32-bit software accumulator triggered by hardware watchpoints
at ±32767 — `pcnt_unit_get_count()` then returns a continuously accumulating signed
integer, transparent to the caller.

**Alternatives considered**:
- Software GPIO interrupts: rejected — misses pulses at high speeds; not deterministic.
- MCPWM capture timer: measures timestamps between pulses, not direction-aware; wrong
  for bidirectional quadrature.
- Third-party `esp-iot-solution` rotary encoder wrapper: adds managed component
  dependency for no benefit; the raw `driver/pulse_cnt.h` API is complete and well
  documented in the official `examples/peripherals/pcnt/rotary_encoder` example.

**Key API** (ESP-IDF v5.x):
- Header: `#include "driver/pulse_cnt.h"`
- CMake: `PRIV_REQUIRES esp_driver_pcnt`
- `pcnt_new_unit()`, `pcnt_new_channel()`, `pcnt_channel_set_edge_action()`,
  `pcnt_channel_set_level_action()`, `pcnt_unit_set_glitch_filter()`,
  `pcnt_unit_add_watch_point()`, `pcnt_unit_enable()`, `pcnt_unit_start()`,
  `pcnt_unit_get_count()`

**Constraint**: PCNT is a scarce resource — only 4 units available. Do not use PCNT
for any other purpose in this firmware.

---

## R-002: GPIO Pin Safety for Encoder Assignments

**Question**: Are GPIO 8–14 and 21 safe to use as PCNT inputs on ESP32-S3-WROOM-1?

**Decision**: **ALL SAFE** — none of the 8 encoder pins (8, 9, 10, 11, 12, 13, 14, 21)
conflict with strapping pins, USB-JTAG, SPI flash, or PSRAM on the ESP32-S3-WROOM-1
module.

**Rationale**: PCNT inputs are routed through the GPIO Matrix and accept any digital
GPIO. GPIO 8–14 and 21 are in the clean zone: above strapping pins (0, 3, 45, 46),
below USB-JTAG (19, 20), and well below SPI flash/PSRAM (26–37). No conflicts found.

**Alternatives considered**: N/A — pins were specified by hardware wiring; the research
validates they are safe.

---

## R-003: GB37-520 Encoder Specification (CPR and Gearbox Ratio)

**Question**: What is the confirmed counts-per-revolution at the output shaft for the
GB37-520 motor?

**Decision**: Use **1980 counts per output shaft revolution** as the default, derived
from: 11-line hall-effect encoder × 4 (quadrature) × 45:1 gearbox = 1980 CPR.
This value is configurable via Kconfig to allow correction after hardware measurement.

**Rationale**: The GB37-520 is a family name. The encoder (11 lines) is consistent
across all GB37 variants. The gearbox ratio is model-dependent — "520" in the model
name refers to the encoder type, not the ratio. Common ratios: 30:1, 45:1, 60:1,
90:1. The **45:1 ratio is the most prevalent** for the standard GB37-520 variant used
in rover projects at ~200 RPM no-load speed (matching the rover's 18.75 rad/s output
= ~179 RPM which aligns with 45:1 at 8000 RPM motor speed).

**Tick rate validation**: At 18.75 rad/s (max speed) and 1980 CPR:
- Ticks/sec = (18.75 / 2π) × 1980 ≈ **5907 ticks/sec per wheel**
- ESP32 PCNT maximum input frequency: >1 MHz — no issue whatsoever.

**Confidence note**: The gearbox ratio should be confirmed by mechanical measurement
(count ticks per full output shaft revolution) during hardware bring-up. If ratio
differs, only the `ENCODER_CPR` Kconfig default needs updating — no logic changes.

**Alternatives considered**: A-only (×2) counting would halve CPR to 990, reducing
velocity resolution at low speeds. 4X quadrature at 1980 CPR provides ~1.86 mrad/s
resolution at 50 Hz sample rate — adequate for closed-loop control.

---

## R-004: micro-ROS Publisher Count Limit

**Question**: Can the existing micro-ROS setup support a second publisher? What changes
are needed?

**Decision**: **Yes, with one required change** — increase `RMW_UXRCE_MAX_PUBLISHERS`
from `2` to `3` in `firmware/components/micro_ros_espidf_component/colcon.meta` and
rebuild `libmicroros.a`.

**Rationale**: `colcon.meta` currently sets `RMW_UXRCE_MAX_PUBLISHERS=2`. The firmware
already uses: 1× `/firmware_status` publisher. Adding `/wheel_velocities` hits the
ceiling exactly at 2. A third publisher (diagnostics, future-proofing) requires 3.
Setting to 3 costs ~600 bytes of SRAM and provides one spare slot. Total SRAM budget
remains safe: ~80–100 KB free heap after all allocations.

**Additional changes required**:
1. `uros_transport.c` line 112–113: executor capacity `3` → `4`
   (handles: 1 subscription + 1 status timer + 1 wheel publisher timer = 3; +1 spare)
2. `app_main.c` `SPIN_TIMEOUT_NS` `50 ms` → `15 ms` to reliably deliver 50 Hz timer
   callbacks (current 50 ms timeout means a 20 ms timer fires at most once per spin)
3. `wheel_publisher.c`: use **static buffer** for `Float32MultiArray.data` — do NOT
   call `__fini()` on a static-buffer message (would free a BSS pointer)

**Alternatives considered**:
- FreeRTOS task calling `rcl_publish()` directly: rejected — `rcl_publish()` is not
  thread-safe against the executor; concurrent calls race on the XRCE output buffer.
- Separate micro-ROS node on a second executor: rejected — excessive complexity,
  memory cost, and violates Constitution Principle VI (simplicity).

---

## R-005: Velocity Computation Method

**Question**: Timer-based periodic sampling vs. capture-timer for velocity computation?

**Decision**: **GPTimer periodic alarm + PCNT count delta**. Sample at 50 Hz (20 ms
period). `pcnt_unit_get_count()` is explicitly ISR-safe and can be called from the
GPTimer alarm callback.

**Formula**:
```
ω (rad/s) = Δcount × 2π / (CPR × Δt_s)
```
Where `Δcount` = count difference between samples, `CPR` = 1980, `Δt_s` = 0.02 s.

**Noise suppression**: If `|Δcount| < NOISE_FLOOR` (default: 2 ticks), output 0.0.
At 1980 CPR and 50 Hz, 2 ticks = 0.03 rad/s noise floor — below any meaningful
commanded velocity.

**Rationale**: Fixed Δt simplifies the math and integrates naturally with the
`controller_manager` at 50 Hz. Works correctly at zero speed (Δcount = 0 → ω = 0).
Capture-timer approaches fail at zero speed (no pulses → no event) and require complex
timeout logic.

**Alternatives considered**: PCNT watchpoint interrupt + timestamp: more precise but
fails at near-zero speed; adds ISR-to-task complexity; not needed for motor control.

---

## R-006: Sign Convention Alignment

**Question**: How to ensure encoder velocity sign matches the motor command sign?

**Decision**: Configure PCNT channels so that **positive count = forward wheel
rotation** = the same direction produced by `LPWM` driven, `RPWM = 0` in `motor.h`.

**Rationale**: `motor.h` documents: `speed > 0 → LPWM driven, RPWM = 0 (forward)`.
The PCNT channel edge/level action configuration determines which direction increments
vs. decrements. Channel A and B wiring order must be physically confirmed during
bring-up. If counts decrement during forward motion, swap A and B pin assignments in
`encoder.h` for the affected wheel — no logic change required.

**Alternatives considered**: Software sign inversion table (like the old
`WHEEL_DIR_SIGN` approach): rejected — creates a hidden convention requiring
documentation; physical channel swap is self-documenting.

---

## R-007: ROS-Side Changes for Closed-Loop Odometry

**Question**: What ROS-side changes are needed to use encoder feedback for odometry?

**Decision**: Only **one parameter change** in `config/controllers.yaml`:
`open_loop: true` → `open_loop: false`. No changes to `wheel_bridge.py`, no changes
to `hardware.launch.py`, no new ROS packages required.

**Rationale**: `wheel_bridge.py` already has the full pipeline:
- Subscribes to `/wheel_velocities` (Float32MultiArray)
- Converts to JointState on `/wheel_velocities_js`
- `topic_based_ros2_control` feeds this into the controller state interfaces
- `mecanum_drive_controller` computes odometry from state interfaces when `open_loop: false`

The loopback (command → state when no ESP32 data) will be automatically superseded
when real `/wheel_velocities` data arrives — the existing `ESP32_TIMEOUT = 1.0 s`
watchdog already handles the transition gracefully.

**The `open_loop: false` change MUST be deferred** until encoder feedback is verified
correct in hardware bring-up (US1 acceptance criteria passed). Enabling it prematurely
with incorrect CPR or sign conventions produces actively wrong odometry, which is
worse than open-loop.
