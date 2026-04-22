# Quickstart: 008-encoder-feedback

**Audience**: Developer implementing this feature
**Prerequisites**: Feature is implemented and firmware flashed to the rover
**Date**: 2026-03-06

---

## Verify US1: Encoder Velocity Publishing

### Step 1 — Start the ROS 2 stack

```bash
# On the Raspberry Pi (or host PC with rover connected)
docker compose up -d
```

Confirm containers are healthy:
```bash
docker compose ps
```

### Step 2 — Start the micro-ROS agent

The `micro_ros_agent` container must be running and connected to the ESP32.
Verify the ESP32 is detected:

```bash
docker compose logs micro_ros_agent | tail -20
# Expected: "Create session successfully"
```

### Step 3 — Verify /wheel_velocities is being published

```bash
ros2 topic hz /wheel_velocities
# Expected: ~50 Hz  (average rate: 50.0)

ros2 topic echo /wheel_velocities
# Expected at rest: data: [0.0, 0.0, 0.0, 0.0]
```

### Step 4 — Spin-test each wheel individually

Lift a wheel off the ground (or have an assistant spin it by hand).
Watch the topic:

```bash
ros2 topic echo /wheel_velocities --once
```

Spin FL (index 0) — expected: `data[0]` non-zero, others near 0.0
Spin FR (index 1) — expected: `data[1]` non-zero, others near 0.0
Spin RL (index 2) — expected: `data[2]` non-zero, others near 0.0
Spin RR (index 3) — expected: `data[3]` non-zero, others near 0.0

**Sign check**: Spin forward (same direction as commanded forward motion).
All spinning-forward wheels must report **positive** values.

### Step 5 — Check encoder diagnostics

```bash
ros2 topic echo /firmware_status
```

Confirm the JSON includes `encoder_counts`, `encoder_velocities`, and `encoder_faults`:
```json
{
  "encoder_counts": [123, 456, 789, 321],
  "encoder_velocities": [0.0, 0.0, 0.0, 0.0],
  "encoder_faults": [false, false, false, false]
}
```

Spin a wheel 10 revolutions by hand. `encoder_counts[i]` should increase by ~1980 ticks.

---

## Verify US1 Accuracy (SC-002)

SC-002 requires velocity readings within ±10% of a reference speed across 0.5–18.75 rad/s.
Use SC-006 (10-revolution count) as a proxy for accuracy — if tick counts are correct,
velocity computation is correct by construction (same CPR and Δt values).

### Reference speed derivation via timed revolution count

1. Spin one wheel at a steady speed by hand for exactly 10 revolutions.
2. Record elapsed time `T_s` with a stopwatch.
3. Reference speed: $\omega_{ref} = \frac{10 \times 2\pi}{T_s}$ rad/s
4. Record the `/wheel_velocities` average during the same interval:
   ```bash
   ros2 topic echo /wheel_velocities --field data[0]  # adjust index per wheel
   ```
5. Confirm measured value is within ±10% of $\omega_{ref}$.

For slow speeds (~0.5 rad/s), use the tick accumulation directly:
- At 50 Hz, 0.5 rad/s → Δcount ≈ `0.5 × 1980 / (2π × 50)` ≈ 3 ticks per sample.
- Confirm `encoder_counts` increments by ~3 per 20 ms interval (observable via
  `ros2 topic echo /firmware_status` at 1 Hz, or derive from bag recording).

### Alternative: Compare commanded vs. measured at steady state

Drive the rover forward by joystick at roughly half speed for ~5 seconds.
Record a bag:

```bash
ros2 bag record /wheel_velocities /wheel_velocities_cmd_f32 -o encoder_test
```

Play back and compare commanded vs. measured:
```bash
ros2 bag play encoder_test
ros2 topic echo /wheel_velocities
ros2 topic echo /wheel_velocities_cmd_f32
```

Measured velocities should track commanded velocities within ±15% during
steady-state motion. Note: this validates system-level behaviour (motor +
encoder together), not encoder accuracy in isolation. Use the timed-revolution
method above for a pure encoder accuracy measurement (SC-002 ±10% criterion).

---

## Verify US2: Enable Closed-Loop Odometry

⚠️ **Do this only after US1 acceptance criteria are fully passed.**

### Step 1 — Enable closed-loop in the controller config

Edit `config/controllers.yaml`:
```yaml
open_loop: false   # was: true
```

Rebuild the Docker image:
```bash
docker compose build
docker compose down && docker compose up -d
```

### Step 2 — Drive a 1 m straight line

With RViz open (Fixed Frame: `odom`):
1. Mark the rover's current position on the floor.
2. Drive forward approximately 1 m using the joystick.
3. Stop. Read the odometry:

```bash
ros2 topic echo /odom --once | grep -A5 "position:"
```

Expected: `x` in the range 0.90–1.10 m, `y` ≈ 0.0, yaw change ≤ 5°.

### Step 3 — Record for documentation

```bash
ros2 bag record /odom /cmd_vel /wheel_velocities /firmware_status -o closed_loop_test
```

Attach this bag to the PR as required by the Constitution (Development Workflow §4).

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `/wheel_velocities` not published | `RMW_UXRCE_MAX_PUBLISHERS` still at 2 | Check `colcon.meta`, rebuild libmicroros |
| All velocities stuck at 0.0 | PCNT units not starting | Check `encoder_init()` ESP_ERROR_CHECK log output |
| One wheel always 0.0 | Encoder cable disconnected or pin wrong | Verify GPIO wiring; check `/firmware_status` `encoder_faults` (note: a disconnected pin shows 0.0 velocity but does NOT set `encoder_fault` — fault flag fires only on implausibly high counts) |
| Commanded speed ≠ 0 but measured = 0 | Open-circuit encoder wiring | Disconnected A or B pin → no counts → velocity reads 0.0. Wiggle connector. `encoder_fault` will be false in this case. |
| Values are negative when driving forward | A/B pin swap on that wheel | Swap `pin_a`/`pin_b` for that wheel in `encoder.h` |
| Velocities much too high (~100× expected) | CPR too low; gearbox ratio wrong | Measure actual CPR by hand, update `CONFIG_ENCODER_CPR` |
| Rate drops below 50 Hz | `SPIN_TIMEOUT_NS` still 50 ms | Set to `(15ULL * 1000000ULL)` in `app_main.c` |
| Watchdog resets during fast motion | Timer ISR taking too long | Ensure `pcnt_unit_get_count()` only (no floats) in ISR; compute floats in rclc timer callback |

---

## rosbag2 Recording Reference

Per Constitution Principle V, record these topics during hardware test sessions:

```bash
ros2 bag record \
  /cmd_vel \
  /joy \
  /wheel_velocities \
  /wheel_velocities_cmd_f32 \
  /odom \
  /firmware_status \
  /diagnostics \
  -o rover_encoder_$(date +%Y%m%d_%H%M%S)
```
