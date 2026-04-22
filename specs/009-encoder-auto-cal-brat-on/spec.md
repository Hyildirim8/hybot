# Feature Specification: Encoder Auto-Calibration

**Feature Branch**: `009-encoder-auto-cal-brat-on`
**Created**: 2026-03-06
**Status**: Draft

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Automatic Encoder Direction Calibration (Priority: P1) 🎯 MVP

The developer runs a one-time calibration routine on the physical rover.
The firmware commands each wheel forward briefly, reads the resulting encoder
count direction, determines whether the sign is correct, and writes a per-wheel
direction correction (`+1` or `-1`) to NVS flash. On every subsequent boot, the
firmware loads these corrections automatically — no manual pin-swap or source
code edit is required.

**Why this priority**: Without correct encoder sign conventions, all velocity
feedback (feature 008) is either partially or fully inverted, making closed-loop
odometry actively harmful. Direction correctness is the single prerequisite for
everything else in this feature.

**Independent Test**: Install the rover with encoder cables in any orientation.
Trigger the calibration routine. Drive forward — all four `/wheel_velocities`
values must be positive without any manual Kconfig or wiring change.

**Acceptance Scenarios**:

1. **Given** the rover is stationary and calibration is triggered, **When** the
   firmware spins each wheel forward in sequence for a short burst, **Then** it
   records the encoder count direction per wheel and stores a per-wheel direction
   sign (`+1` or `-1`) to NVS.

2. **Given** direction calibration data is stored in NVS, **When** the firmware
   boots, **Then** it loads the direction signs and applies them to velocity
   computations, requiring no code changes.

3. **Given** an encoder cable is physically reversed (A↔B) after calibration,
   **When** calibration is re-run, **Then** the new direction is detected and the
   stored sign is overwritten; the next boot applies the updated value.

4. **Given** NVS has no calibration data (first boot or after NVS erase),
   **When** the firmware starts, **Then** all wheels default to direction `+1`
   and the rover remains functional for uncalibrated testing.

---

### User Story 2 — Speed Profile Calibration (Priority: P2)

The developer runs a speed calibration routine that commands each wheel through
two representative PWM duty levels, measures the resulting encoder velocity, and
stores a per-wheel speed scaling factor to NVS. On every boot, the firmware
applies these factors so all four wheels reach the same actual speed when given
equal commands — reducing straight-line drift caused by motor manufacturing
tolerances.

**Why this priority**: Even with direction correct, mismatched wheel speeds
cause the rover to drift. Speed calibration directly improves straight-line
tracking and odometry accuracy without requiring matched motors.

**Independent Test**: Run speed calibration. Command all four wheels at the same
speed. Measure encoder velocities — all four must be within ±5% of each other
at steady state, compared to ±20% typical of uncalibrated motors.

**Acceptance Scenarios**:

1. **Given** speed calibration is running, **When** each wheel is commanded at
   two duty levels, **Then** the firmware measures encoder velocity at each level,
   computes a per-wheel scaling factor, and stores it to NVS.

2. **Given** speed scaling factors are stored, **When** the firmware boots and
   receives a velocity command, **Then** it applies the per-wheel scale factor
   before PWM duty conversion so actual wheel speed matches the command.

3. **Given** NVS has no speed calibration data, **When** the firmware starts,
   **Then** it uses scale `1.0` for all wheels (no correction), preserving
   existing behaviour.

4. **Given** a computed scale factor falls outside `[0.5, 2.0]`, **When**
   calibration completes, **Then** the factor is clamped to the nearest bound,
   a per-wheel warning is logged (possible mechanical fault), and the clamped
   value is stored.

---

### User Story 3 — Calibration Status Visibility and Reset (Priority: P3)

The operator can read the currently active calibration parameters from the
firmware status topic without a debugger, and can trigger a calibration reset
to restore factory defaults — useful when miscalibrated parameters cause
unexpected behaviour.

**Why this priority**: Calibration is invisible unless exposed. Without
visibility, a wrong calibration factor is indistinguishable from a hardware
fault. Useful for commissioning but the rover is fully functional without it.

**Independent Test**: Read `/firmware_status` — the JSON includes
`cal_direction[4]` and `cal_speed_scale[4]`. After a calibration reset, both
arrays show all `+1` and `1.0` respectively.

**Acceptance Scenarios**:

1. **Given** calibration data is loaded, **When** subscribing to `/firmware_status`,
   **Then** the JSON payload includes `cal_direction[4]` (int: +1 or -1) and
   `cal_speed_scale[4]` (float near 1.0).

2. **Given** a calibration reset is triggered, **When** the firmware handles it,
   **Then** NVS calibration keys are erased, in-memory values reset to defaults,
   and the next status publish shows all `+1` directions and all `1.0` scales.

3. **Given** only direction calibration was stored (no speed data), **When**
   reading status, **Then** direction fields reflect stored values and speed
   fields show `1.0` (default) with no crash or undefined behaviour.

---

### Edge Cases

- **Calibration during motion**: If the rover is already moving when calibration
  is triggered, calibration MUST be rejected and an error logged. Calibration
  requires the rover to be stationary.

- **Motor not responding**: If a wheel produces zero encoder counts during the
  calibration burst (broken motor or disconnected encoder), calibration for that
  wheel MUST be skipped, the direction default (`+1`) retained, and a per-wheel
  warning logged. Other wheels proceed normally.

- **NVS write failure**: If NVS write fails during calibration, the in-memory
  calibration values MUST still apply for the current session; the error is
  logged. The rover must not be left in an inoperable state.

- **Calibration without micro-ROS agent**: The calibration routine MUST be
  executable with no active micro-ROS agent, since commissioning typically
  happens before the full ROS 2 stack is running.

- **Scale factor saturation**: If measured speed is near-zero (motor stalled),
  the computed scale factor would be infinite. The firmware MUST clamp to
  `[0.5, 2.0]` and log a warning for the affected wheel.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The firmware MUST implement a calibration routine that commands
  each wheel individually in the forward direction for a bounded duration
  (configurable, default 500 ms) and reads the resulting encoder count change
  to determine actual rotation direction per wheel.

- **FR-002**: The firmware MUST store per-wheel direction signs (`+1` or `-1`)
  in NVS under the existing `rover_cfg` namespace using keys `cal_dir_0` through
  `cal_dir_3` (one per wheel, names ≤ 15 chars per NVS constraint).

- **FR-003**: The firmware MUST store per-wheel speed scaling factors (float,
  clamped to `[0.5, 2.0]`) in NVS using keys `cal_spd_0` through `cal_spd_3`.

- **FR-004**: On every boot, the firmware MUST load direction signs and speed
  scaling factors from NVS before the micro-ROS retry loop starts and apply them
  to all subsequent velocity computations and motor commands.

- **FR-005**: If NVS calibration keys are absent, the firmware MUST apply safe
  defaults: direction `+1` and speed scale `1.0` for all wheels, without requiring
  operator intervention.

- **FR-006**: The calibration routine MUST be triggerable without a ROS 2 agent
  connection via a Kconfig-selectable `CONFIG_CALIBRATE_ON_BOOT` mode (runs
  automatically on **any boot** where NVS calibration keys are absent) and a
  reserved GPIO hold-at-boot method.

- **FR-007**: Direction signs MUST be applied to encoder velocity output before
  publishing on `/wheel_velocities`, ensuring physically correct sign convention
  regardless of encoder cable orientation.

- **FR-008**: Speed scaling factors MUST be applied to incoming wheel velocity
  commands before PWM duty conversion so that commanded rad/s maps to actual
  measured rad/s per wheel.

- **FR-009**: The calibration routine MUST log per-wheel results (detected
  direction, measured speed at each duty level, computed scale factor) at INFO
  level so results are visible in serial logs and Docker container logs.

- **FR-010**: Active calibration parameters MUST be included in `/firmware_status`
  JSON as `cal_direction[4]` and `cal_speed_scale[4]` (additive extension —
  no existing fields changed).

- **FR-011**: The firmware MUST support a calibration reset that erases the eight
  NVS calibration keys and reverts in-memory values to defaults, without a full
  NVS partition erase.

- **FR-012**: Speed scaling factors MUST be clamped to `[0.5, 2.0]` before
  storage. Clamped values MUST trigger a per-wheel WARN log identifying a
  possible mechanical fault.

### Key Entities

- **CalibrationParams** — Per-wheel calibration state loaded from NVS at boot.
  Attributes: `dir_sign[4]` (int8: +1 or -1), `speed_scale[4]` (float: 0.5–2.0),
  `calibrated` (bool: false when defaults are in use).

- **CalibrationResult** — Output of one calibration run for a single wheel.
  Attributes: `wheel_id`, `detected_direction` (+1 or -1), `measured_speeds[]`
  (floats at each duty level), `computed_scale` (float), `valid` (bool).

- **NVS Calibration Keys** — Under `rover_cfg` namespace:
  `cal_dir_0`–`cal_dir_3` (int8 per wheel), `cal_spd_0`–`cal_spd_3` (float
  per wheel). All key names ≤ 15 chars (NVS hard constraint).

## Success Criteria *(mandatory)*

- **SC-001**: After direction calibration, driving forward produces all four
  positive `/wheel_velocities` values regardless of physical encoder cable
  orientation — with zero manual code or Kconfig changes.

- **SC-002**: After speed calibration, all four wheels reach the same measured
  angular velocity within ±5% **of the commanded angular velocity** when given
  equal commands, across the range 2–15 rad/s.

- **SC-003**: Calibration parameters survive a power cycle — rover behaviour on
  the second boot is identical to immediately after calibration completed.

- **SC-004**: The full calibration routine (direction + speed for all 4 wheels)
  completes in under 30 seconds from trigger to NVS write complete.

- **SC-005**: With calibration applied, straight-line odometry drift over 1 m
  is ≤ 5 cm (meeting the target from feature 008 SC-003), confirming calibration
  does not degrade the closed-loop odometry baseline.

- **SC-006**: After a calibration reset, the rover behaves identically to an
  uncalibrated first boot — all defaults in effect, no stale values persisted.

## Assumptions

1. **Feature 008 prerequisite**: `008-encoder-feedback` is implemented and
   merged. The `g_encoder_velocities[]` globals and PCNT hardware must be
   available before auto-calibration is meaningful.

2. **NVS capacity**: The existing `rover_cfg` namespace has capacity for 8
   additional keys (4× direction int8 + 4× speed float). NVS partition (default
   24 KB) supports hundreds of keys — no capacity concern.

3. **Calibration duty level**: 30% PWM for direction detection and 30%/70% for
   speed profiling are safe for GB37-520 motors. A 500 ms burst at 30% duty
   causes ~3–5 cm wheel travel on the ground — acceptable if the rover has
   clearance.

4. **Linear speed response**: GB37-520 motors with BTS7960B drivers have a
   sufficiently linear speed-to-duty relationship between 20–80% duty that a
   two-point scaling factor adequately characterises the per-wheel response.
   Non-linearity at extremes is within the ±5% SC-002 tolerance.

5. **Calibration trigger**: `CONFIG_CALIBRATE_ON_BOOT` auto-runs calibration
   only when NVS calibration keys are absent, preventing unintended re-runs on
   every reboot after initial setup.

6. **ROS 2 service for reset**: The calibration reset service (US3) is
   implemented as a micro-ROS service, only available when the agent is
   connected. An alternative GPIO or serial reset method is a P3 stretch goal.
