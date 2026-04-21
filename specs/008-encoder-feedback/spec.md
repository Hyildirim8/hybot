# Feature Specification: Wheel Encoder Feedback

**Feature Branch**: `008-encoder-feedback`
**Created**: 2026-03-06
**Status**: Draft

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Real-Time Wheel Velocity Feedback (Priority: P1) 🎯 MVP

The operator drives the rover with the joystick. The firmware reads all four
quadrature encoders in real time and publishes actual wheel angular velocities
(in rad/s) over micro-ROS so the ROS 2 control stack receives real encoder data
instead of silence.

**Why this priority**: Without this, the entire encoder hardware is unused.
All higher-level improvements (closed-loop control, accurate odometry) depend
on this data being available and correct.

**Independent Test**: With the rover stationary, spin one wheel by hand.
The `/wheel_velocities` topic must show a non-zero velocity for that wheel and
near-zero for the other three. Drive forward — all four values must be positive
and roughly equal.

**Acceptance Scenarios**:

1. **Given** the rover is stationary, **When** the firmware starts, **Then**
   `/wheel_velocities` publishes `[0.0, 0.0, 0.0, 0.0]` at the configured rate.

2. **Given** a wheel is spinning at a known speed, **When** reading `/wheel_velocities`,
   **Then** the corresponding index reports a value within ±10% of the true speed.

3. **Given** the rover drives forward at full speed, **When** reading `/wheel_velocities`,
   **Then** all four values are positive and within ±15% of each other.

4. **Given** one encoder cable is disconnected, **When** reading `/wheel_velocities`,
   **Then** that wheel reports 0.0 (no spurious noise counts), the other three
   continue reporting correctly.

---

### User Story 2 — Closed-Loop Odometry in RViz (Priority: P2)

The operator observes the rover in RViz. With encoder feedback enabled, the
`odom→base_link` transform is computed from measured wheel velocities rather than
open-loop commands. The robot pose in RViz visually matches the real robot's
position and heading with noticeably lower drift.

**Why this priority**: Encoder feedback is only valuable if it improves the
odometry seen by the operator and Nav2. Requires US1 to be complete.

**Independent Test**: Drive the rover in a 1 m straight line and stop.
With `open_loop: false` in the controller config, the RViz odometry arrow must
show ≤ 5 cm positional error and ≤ 5° heading error compared to the physical
robot position.

**Acceptance Scenarios**:

1. **Given** `open_loop` is set to `false`, **When** the rover drives forward 1 m,
   **Then** the `/odom` pose `x` increases by 0.9–1.1 m and yaw remains within ±5°.

2. **Given** the rover drives in a 1 m square (4 × 90° turns), **When** returning
   to the start, **Then** the odometry position error is ≤ 15 cm.

3. **Given** the rover is commanded to rotate 90° in place, **When** the motion
   completes, **Then** the `/odom` heading matches the physical heading within ±5°.

---

### User Story 3 — Encoder Diagnostics (Priority: P3)

The operator or developer can inspect per-wheel encoder counts, tick rates, and
fault flags from a status topic to diagnose wiring problems, misconfigured
pulses-per-revolution, or a broken encoder channel without needing a separate
oscilloscope.

**Why this priority**: Useful for commissioning and fault isolation, but the
system is fully functional without it.

**Independent Test**: Spin one wheel by hand five complete revolutions. The
firmware status report for that wheel must show a tick count close to
5 × PPR × 4. Stop the wheel — tick rate drops to 0 within 200 ms.

**Acceptance Scenarios**:

1. **Given** the firmware is running, **When** subscribing to the firmware status
   topic, **Then** the payload includes per-wheel encoder counts and velocities.

2. **Given** a wheel is spun by hand and stopped, **When** reading status,
   **Then** the velocity entry for that wheel drops to 0 within 200 ms of stopping.

3. **Given** one encoder channel produces a tick rate that exceeds the physical
   maximum (implausible reading — e.g. due to severe EMI or a shorted pin), **When**
   reading status, **Then** that wheel's velocity is clamped to `MAX_SPEED_RAD_S`
   and its `encoder_fault` flag is set `true`.

   > **Note on wiring faults (open/stuck-low pin)**: A disconnected or stuck-low
   > A or B pin does not set `encoder_fault`; it causes zero counts (velocity = 0.0).
   > Distinguish from a truly stationary wheel by comparing commanded speed to
   > measured speed: if commanded ≠ 0 but measured = 0, suspect a wiring fault.
   > Use `encoder_counts` trending to confirm.

---

### Edge Cases

- **Speed clamp**: If a velocity computation exceeds `MAX_SPEED_RAD_S` (hardware
  maximum), the published value MUST be clamped, not propagated as-is.
- **Noise / PWM EMI**: Single spurious pulses from motor PWM noise MUST NOT cause
  velocity spikes; a minimum tick-count threshold per measurement interval suppresses
  noise-floor artifacts.
- **Direction reversals**: Rapid command direction changes MUST be tracked correctly
  without count loss or direction sign errors.
- **Agent disconnect**: Encoder counters MUST continue accumulating during a
  micro-ROS disconnect; publishing resumes immediately on reconnection without
  losing direction tracking.
- **Very low speeds**: The system MUST distinguish genuine slow motion from encoder
  noise. Below the noise-floor threshold, the output MUST be reported as 0.0.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The firmware MUST read quadrature (A+B) encoder signals for all four
  wheels using hardware pulse-counter peripherals (not software GPIO interrupts)
  to prevent missed counts at high speeds.

- **FR-002**: The firmware MUST use the following GPIO pin assignments for encoders:
  - Front-Left:  A = GPIO 8,  B = GPIO 9
  - Front-Right: A = GPIO 10, B = GPIO 11
  - Rear-Left:   A = GPIO 12, B = GPIO 13
  - Rear-Right:  A = GPIO 14, B = GPIO 21

- **FR-003**: The firmware MUST compute wheel angular velocity (rad/s) from encoder
  tick counts using the measured pulse count per time interval, the encoder
  pulses-per-revolution (at the output shaft), and the elapsed time, yielding a
  signed result where positive = forward.

- **FR-004**: The encoder pulses-per-revolution and gearbox ratio MUST be
  configurable at compile time with defaults matching the GB37-520 motor spec
  (11-line encoder, quadrature ×4 = 44 counts/motor revolution, 45:1 gearbox,
  giving 1980 counts per output shaft revolution).

- **FR-005**: The firmware MUST publish wheel velocities as a four-element array
  (Float32MultiArray) on topic `/wheel_velocities` in the order
  `[FL, FR, RL, RR]`, matching the existing motor command topic convention.

- **FR-006**: The velocity publish rate MUST default to 50 Hz and MUST be
  configurable at compile time to match the ROS 2 controller manager update rate.

- **FR-007**: Published velocities MUST be clamped to the hardware maximum speed.
  A velocity reading outside this range is treated as a fault, not a valid
  measurement, and MUST be clamped before publishing.

- **FR-008**: The velocity computation MUST suppress noise: if the absolute tick
  count `|Δcount|` during a measurement interval is below a configurable
  noise-floor threshold (default: 2 ticks per interval), the output for that
  wheel MUST be 0.0. The absolute value ensures noise suppression applies
  equally to forward and reverse motion.

- **FR-009**: Encoder signal accumulation MUST continue during micro-ROS agent
  disconnection. Velocity publishing MUST resume immediately on reconnection
  without loss of count state or direction sign.

- **FR-010**: Velocity sign MUST match motor command sign convention: positive
  output velocity corresponds to forward wheel rotation (the same direction
  produced when the motor driver forward input is asserted).

- **FR-011**: The `open_loop` parameter in the ROS 2 controller configuration
  MUST be changed to `false` once encoder feedback is verified stable, so that
  the differential-drive/mecanum controller uses measured velocities for odometry
  instead of open-loop command replay.

- **FR-012**: The firmware diagnostic report MUST be extended to include per-wheel
  `encoder_counts` (cumulative signed 32-bit integer), per-wheel `encoder_velocities`
  (last computed rad/s), and per-wheel `encoder_fault` boolean flags (set `true`
  when a computed velocity exceeds `MAX_SPEED_RAD_S`, indicating an implausible
  reading that was clamped), alongside existing commanded-speed values.

### Key Entities

- **Encoder** — One per wheel. Quadrature (A+B) signal pair connected to a hardware
  pulse-counter unit. Attributes: A-pin, B-pin, cumulative count, last-sample count,
  last-sample timestamp, fault flag.

- **WheelVelocity** — Derived measurement per wheel. Attributes: rad/s (signed),
  clamped flag, noise-suppressed flag.

- **EncoderConfig** — Compile-time parameters shared by all four encoders.
  Attributes: counts-per-output-revolution, noise-floor threshold, publish period.

## Success Criteria *(mandatory)*

- **SC-001**: Each wheel's velocity channel responds independently — spinning a
  single wheel by hand changes only that channel; the other three remain at 0.0
  (no cross-talk).

- **SC-002**: Velocity readings are within ±10% of a reference speed measured
  independently (e.g., timing wheel revolutions manually) across the full operating
  range from 0.5 rad/s to maximum speed.

- **SC-003**: With `open_loop: false`, driving in a straight line over 1 m produces
  an odometry positional error ≤ 5 cm (target), compared to ≥ 15 cm typical of
  open-loop mecanum odometry.

- **SC-004**: Encoder velocity data reaches the ROS topic within 25 ms of the
  corresponding wheel motion (50 Hz update latency budget).

- **SC-005**: No encoder-related firmware crash, watchdog reset, or count corruption
  occurs during 30 minutes of continuous operation including full-speed bursts,
  rapid direction reversals, and a simulated micro-ROS disconnect/reconnect cycle.

- **SC-006**: Spinning any wheel 10 full revolutions by hand produces an accumulated
  count of 19 800 ± 20 ticks (1 980 counts/rev × 10 revolutions), confirming correct
  PPR configuration.

## Assumptions

1. **Encoder spec**: GB37-520 motors have 11-line hall-effect encoders in quadrature
   mode (44 counts/motor shaft revolution) and a 45:1 gearbox, yielding
   1980 counts per output shaft revolution. If the actual gearbox ratio differs,
   only the compile-time `PPR` parameter needs updating.

2. **GPIO availability**: GPIO pins 8–14 and 21 are unassigned in the current
   firmware (`motor.h`) and are free for encoder use.

3. **Hardware pulse counters**: The ESP32-S3 provides at least 4 hardware pulse-counter
   units capable of full quadrature (A+B direction-aware) decoding. If unit count
   is insufficient, A-only counting with software direction tracking is an acceptable
   fallback (deferred to planning).

4. **wheel_bridge.py loopback**: The existing loopback (command → state when no
   ESP32 feedback arrives) will be automatically superseded once the ESP32 publishes
   real `/wheel_velocities` data, because the existing `ESP32_TIMEOUT` watchdog
   already handles the transition. No changes to `wheel_bridge.py` logic are
   expected beyond setting `open_loop: false`.

5. **Controller compatibility**: `mecanum_drive_controller` in ROS 2 Humble accepts
   state feedback via the `topic_based_ros2_control` state interface through the
   existing `/wheel_velocities_js` JointState path. No changes to the hardware
   launch file or bridge node are expected.
