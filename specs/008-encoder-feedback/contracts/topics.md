# Topic Contracts: 008-encoder-feedback

**Format**: micro-ROS (publisher) + ROS 2 topic (consumer)
**Date**: 2026-03-06

This document defines the ROS 2 topic interface exposed and consumed by this
feature. These contracts govern the wire format, topic names, QoS, and message
layout for all inter-component communication.

---

## Published: `/wheel_velocities`

**Direction**: ESP32 firmware → ROS 2 graph (via micro-ROS agent)
**Purpose**: Real-time per-wheel angular velocity feedback for ros2_control
**Status**: NEW (firmware currently has 0 publishers on this topic)

| Property | Value |
|----------|-------|
| Topic name | `/wheel_velocities` |
| Message type | `std_msgs/msg/Float32MultiArray` |
| Publisher | ESP32 firmware (`wheel_publisher.c`) |
| Consumers | `wheel_bridge.py`, `rosbag2` recording |
| Rate | 50 Hz (20 ms period) |
| QoS reliability | BEST_EFFORT |
| QoS durability | VOLATILE |
| QoS history | KEEP_LAST(1) |

### Message Layout

```
Float32MultiArray
├── layout: MultiArrayLayout     (empty — not populated by firmware)
└── data:   float32[4]
            [0] = FL  front-left  wheel velocity (rad/s)
            [1] = FR  front-right wheel velocity (rad/s)
            [2] = RL  rear-left   wheel velocity (rad/s)
            [3] = RR  rear-right  wheel velocity (rad/s)
```

**Sign convention**: positive = forward wheel rotation
(matches motor command sign: `speed > 0 → LPWM driven, RPWM = 0`)

**Range**: `[-18.75, +18.75]` rad/s (clamped to `MAX_SPEED_RAD_S`)

**Zero condition**: value is 0.0 when `|Δticks| < NOISE_FLOOR` (default: 2 ticks)
or when the motor is stationary.

### Constraints

- Array size is ALWAYS exactly 4. Consumers MUST reject messages with `data.size ≠ 4`.
- `layout` fields are all zero/empty. Consumers MUST NOT depend on layout metadata.
- Publishing is contingent on micro-ROS agent connection. If the agent is
  disconnected, no messages are published. Consumers MUST handle topic silence
  gracefully (e.g., fall back to open-loop when no message received within timeout).

---

## Consumed (unchanged): `/wheel_velocities_cmd_f32`

**Direction**: ROS 2 graph → ESP32 firmware (via micro-ROS agent)
**Purpose**: Per-wheel velocity commands from ros2_control
**Status**: EXISTING — no changes to this contract

| Property | Value |
|----------|-------|
| Topic name | `/wheel_velocities_cmd_f32` |
| Message type | `std_msgs/msg/Float32MultiArray` |
| Publisher | `wheel_bridge.py` (translates JointState commands) |
| Consumer | ESP32 firmware (`velocity_subscriber.c`) |
| Rate | 50 Hz (driven by controller_manager update_rate) |
| QoS | BEST_EFFORT / VOLATILE / KEEP_LAST(1) |

Layout identical to `/wheel_velocities` above (same index order and sign convention).

---

## Published (extended): `/firmware_status`

**Direction**: ESP32 firmware → ROS 2 graph
**Purpose**: Diagnostic snapshot for operator and developer tooling
**Status**: EXISTING — JSON schema extended with encoder fields

| Property | Value |
|----------|-------|
| Topic name | `/firmware_status` |
| Message type | `std_msgs/msg/String` (JSON payload) |
| Rate | 1 Hz |
| QoS | BEST_EFFORT / VOLATILE / KEEP_LAST(1) |

### JSON Schema (updated)

```json
{
  "commanded_speeds":   [0.0, 0.0, 0.0, 0.0],
  "watchdog_state":     "active",
  "motor_faults":       [false, false, false, false],
  "uptime_ms":          12345,
  "malformed_msg_count": 0,

  "encoder_counts":     [0, 0, 0, 0],
  "encoder_velocities": [0.0, 0.0, 0.0, 0.0],
  "encoder_faults":     [false, false, false, false]
}
```

**New fields** (this feature):
- `encoder_counts[4]`: cumulative signed tick count since boot, per wheel
- `encoder_velocities[4]`: last sampled rad/s per wheel (same values as `/wheel_velocities`)
- `encoder_faults[4]`: set `true` when computed velocity exceeded `MAX_SPEED_RAD_S` (clamp event). A disconnected/open-circuit encoder pin produces zero counts and does NOT set this flag.

### Backward Compatibility

Consumers of `/firmware_status` that do not consume the new fields are unaffected.
The new fields are additive. No existing field is renamed, retyped, or removed.

---

## Downstream: `/odom` (indirect — not a direct contract of this feature)

`mecanum_drive_controller` publishes `/odom` based on wheel state. This feature
changes the **input** to that computation by enabling `open_loop: false`. The `/odom`
topic interface itself is unchanged; only the accuracy of its data improves.

| Property | Value |
|----------|-------|
| Topic | `/odom` |
| Message type | `nav_msgs/msg/Odometry` |
| Publisher | `mecanum_drive_controller` (unchanged) |
| Change | Accuracy improvement: odometry computed from measured velocities, not commands |

**Gating**: `open_loop: false` MUST NOT be set until US1 acceptance criteria are
verified. Premature activation with wrong CPR or wrong sign produces worse odometry
than open-loop.
