#!/usr/bin/env python3
"""
wheel_bridge.py — Bridges topic_based_ros2_control ↔ ESP32 micro-ROS.

topic_based_ros2_control publishes joint commands as sensor_msgs/JointState
and expects state feedback as sensor_msgs/JointState.

The ESP32 firmware uses std_msgs/Float32MultiArray for both directions:
  /wheel_velocities_cmd  → Float32MultiArray [fl, fr, rl, rr] rad/s  (commands)
  /wheel_velocities      → Float32MultiArray [fl, fr, rl, rr] rad/s  (state)

This bridge converts between the two formats so they can interoperate.

Subscriptions:
  /wheel_velocities_cmd  (sensor_msgs/JointState)   ← from topic_based_ros2_control
  /wheel_velocities_raw  (std_msgs/Float32MultiArray) ← from ESP32

Publishers:
  /wheel_velocities_cmd_f32  (std_msgs/Float32MultiArray) → to ESP32
  /wheel_velocities          (sensor_msgs/JointState)     → to topic_based_ros2_control

Joint order (must match firmware velocity_subscriber.c):
  index 0 = fl_wheel_joint
  index 1 = fr_wheel_joint
  index 2 = rl_wheel_joint
  index 3 = rr_wheel_joint
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import time


JOINT_NAMES = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]

# Legacy motor wiring correction for commands:
# FL/RL motor channels were wired opposite to the controller's positive direction.
# Keep this correction on the command path so the rover still drives correctly.
DEFAULT_COMMAND_DIR_SIGN = [-1.0, 1.0, -1.0, 1.0]

# Encoder sign is now corrected in firmware calibration, so state should pass
# through unchanged by default. This keeps /odom and RViz aligned with reality.
DEFAULT_STATE_DIR_SIGN = [1.0, 1.0, 1.0, 1.0]

# Per-wheel gain on reported encoder velocity. Neutral by default; set in
# config/rover_params.yaml from a measured straight run. See the
# state_scale_factors declaration below for the measurement and rationale.
DEFAULT_STATE_SCALE = [1.0, 1.0, 1.0, 1.0]

# Seconds without real ESP32 data before state is considered stale
ESP32_TIMEOUT = 1.0

# Keep sending last command to ESP32 to avoid stutter on brief upstream gaps
# Must be shorter than the firmware watchdog timeout (1000 ms) so zeros arrive
# before the watchdog fires when the joystick goes idle.
CMD_HOLD_TIMEOUT = 0.4
# 40 Hz keeps command latency low enough for Nav2 to feel responsive while
# remaining comfortably under the firmware watchdog window.
CMD_PUB_RATE_HZ = 40.0

# Clamp only tiny per-wheel commands produced by noise. The previous threshold
# was large enough to make low-speed Nav2 motion feel sluggish.
CMD_DEADBAND_RAD_S = 0.12

# QoS to match topic_based_ros2_control (RELIABLE) and ESP32 micro-ROS (BEST_EFFORT)
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
# RELIABLE with depth=1: guarantees delivery to the XRCE agent without building
# a retransmit backlog (depth=1 means only the newest message is ever queued).
cmd_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class WheelBridge(Node):
    def __init__(self):
        super().__init__("wheel_bridge")

        # Hardware mode default: do not synthesize wheel state from commands.
        # This keeps RViz/odometry aligned with real encoder feedback.
        self.declare_parameter("loopback_without_esp32", False)
        self.declare_parameter("command_deadband_rad_s", CMD_DEADBAND_RAD_S)
        self.declare_parameter("command_direction_signs", DEFAULT_COMMAND_DIR_SIGN)
        self.declare_parameter("state_direction_signs", DEFAULT_STATE_DIR_SIGN)
        # Per-wheel gain on the ESP32's reported velocity, applied after the
        # sign correction. Separate from state_direction_signs because
        # _normalise_signs() deliberately collapses every value to exactly
        # +/-1.0, so a scale factor cannot be smuggled in there.
        #
        # Why this exists: on a straight drive all four mecanum wheels must
        # turn at the same rate, but the encoders disagree left vs right.
        # Measured 2026-08-13 over a clean 3 m straight run (raw topic,
        # sign-corrected, pre-scaling):
        #     FL  95.44   FR 116.37   RL  95.33   RR 114.31  rad
        #     left 95.39  right 115.34  ->  right/left = 1.209
        # The commands for the same run were balanced (left 103.12 vs right
        # 107.75, ratio 1.045) and rf2o put the heading change at -5.1 deg
        # over 2 m, so the robot really did go straight and the wheels really
        # did turn together — the disagreement is in the measurement.
        #
        # That imbalance is the entire source of the phantom yaw in /odom:
        # mecanum yaw is wz ~ (-FL + FR - RL + RR), so a left/right offset
        # integrates directly into heading. On an earlier 2 m run the encoder
        # claimed +29.0 deg while rf2o measured -5.1 deg, and feeding the
        # measured imbalance through the yaw formula predicts +30.3 deg.
        # odom_angular_scale was masking this, not fixing it.
        self.declare_parameter("state_scale_factors", DEFAULT_STATE_SCALE)
        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_base", 0.38)
        self.declare_parameter("wheel_separation_width", 0.26)
        self.declare_parameter("odom_linear_scale", 1.0)
        self.declare_parameter("odom_lateral_scale", 1.0)
        self.declare_parameter("odom_angular_scale", 1.0)
        self.declare_parameter("odom_linear_deadband", 0.0)
        self.declare_parameter("odom_lateral_deadband", 0.0)
        self.declare_parameter("odom_angular_deadband", 0.0)
        self._loopback_without_esp32 = bool(
            self.get_parameter("loopback_without_esp32").value
        )
        self._command_deadband = float(
            self.get_parameter("command_deadband_rad_s").value
        )
        self._command_dir_signs = self._normalise_signs(
            self.get_parameter("command_direction_signs").value,
            DEFAULT_COMMAND_DIR_SIGN,
        )
        self._state_dir_signs = self._normalise_signs(
            self.get_parameter("state_direction_signs").value,
            DEFAULT_STATE_DIR_SIGN,
        )
        self._state_scales = self._normalise_scales(
            self.get_parameter("state_scale_factors").value,
            DEFAULT_STATE_SCALE,
        )
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheel_base = float(self.get_parameter("wheel_base").value)
        self._wheel_separation_width = float(
            self.get_parameter("wheel_separation_width").value
        )
        self._odom_linear_scale = float(self.get_parameter("odom_linear_scale").value)
        self._odom_lateral_scale = float(self.get_parameter("odom_lateral_scale").value)
        self._odom_angular_scale = float(self.get_parameter("odom_angular_scale").value)
        self._odom_linear_deadband = float(
            self.get_parameter("odom_linear_deadband").value
        )
        self._odom_lateral_deadband = float(
            self.get_parameter("odom_lateral_deadband").value
        )
        self._odom_angular_deadband = float(
            self.get_parameter("odom_angular_deadband").value
        )
        self._kinematic_k = (
            (self._wheel_base / 2.0) + (self._wheel_separation_width / 2.0)
        )

        # Timestamp of last real ESP32 state message (for loopback fallback)
        self._last_esp32_time: float = 0.0
        self._last_cmd_time: float = 0.0
        self._last_cmd: list[float] = [0.0, 0.0, 0.0, 0.0]

        # Command direction: JointState (from ros2_control) → Float32MultiArray (to ESP32)
        self._cmd_sub = self.create_subscription(
            JointState,
            "/wheel_velocities_cmd",
            self._on_cmd,
            reliable_qos,
        )
        # RELIABLE depth=1: guarantees each command arrives at the XRCE agent.
        # depth=1 prevents stale retransmit backlog from bursting into ESP32 CDC.
        self._cmd_pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_velocities_cmd_f32",
            cmd_qos,
        )

        # State direction: Float32MultiArray (from ESP32) → JointState (to ros2_control)
        self._state_sub = self.create_subscription(
            Float32MultiArray,
            "/wheel_velocities",
            self._on_state,
            best_effort_qos,
        )
        self._state_pub = self.create_publisher(
            JointState,
            "/wheel_velocities_js",
            reliable_qos,
        )

        self._cmd_timer = self.create_timer(1.0 / CMD_PUB_RATE_HZ, self._publish_cmd_tick)

        self.get_logger().info("wheel_bridge ready")

    @staticmethod
    def _normalise_signs(value, fallback: list[float]) -> list[float]:
        signs = []
        if isinstance(value, (list, tuple)):
            raw = list(value)
        else:
            raw = []

        for index, default in enumerate(fallback):
            try:
                item = float(raw[index])
            except (IndexError, TypeError, ValueError):
                item = default
            signs.append(-1.0 if item < 0.0 else 1.0)
        return signs

    @staticmethod
    def _normalise_scales(value, fallback: list[float]) -> list[float]:
        """Like _normalise_signs but keeps magnitude — that is the whole point.

        Only the sign is dropped (direction belongs to state_direction_signs)
        and non-positive or unparseable entries fall back to the default, so a
        typo cannot silently zero out a wheel's feedback.
        """
        raw = list(value) if isinstance(value, (list, tuple)) else []
        scales = []
        for index, default in enumerate(fallback):
            try:
                item = abs(float(raw[index]))
            except (IndexError, TypeError, ValueError):
                item = default
            scales.append(item if item > 0.0 else default)
        return scales

    def _make_joint_state(self, velocities: list) -> JointState:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.velocity = [float(v) for v in velocities]
        js.position = [0.0] * 4  # topic_based needs position interface too
        js.effort = []
        return js

    def _on_cmd(self, msg: JointState) -> None:
        """Store latest JointState velocity command; the timer tick sends it.
        Optional loopback mode can echo commands as synthetic state when
        ESP32 feedback is absent (disabled by default for real hardware sync).
        """
        vel_map = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        velocities = [float(vel_map.get(name, 0.0)) for name in JOINT_NAMES]

        velocities = [
            velocities[i] * self._command_dir_signs[i]
            for i in range(4)
        ]

        # Reject low-amplitude leakage so intended zero-wheel commands remain zero.
        if self._command_deadband > 0.0:
            velocities = [
                0.0 if abs(v) < self._command_deadband else v
                for v in velocities
            ]

        self._last_cmd = velocities
        self._last_cmd_time = time.monotonic()
        # Do NOT publish here — the 10 Hz timer tick is the sole sender so the
        # USB CDC / XRCE pipe never gets burst-flooded by upstream control rate.

        # Optional loopback: only for bench/sim diagnostics.
        if (
            self._loopback_without_esp32
            and time.monotonic() - self._last_esp32_time > ESP32_TIMEOUT
        ):
            self._state_pub.publish(self._make_joint_state(velocities))

    def _publish_f32(self, velocities: list[float]) -> None:
        out = Float32MultiArray()
        out.data = [float(v) for v in velocities]
        self._cmd_pub.publish(out)

    def _publish_cmd_tick(self) -> None:
        # Always publish — either the held command or zeros.
        # This continuously resets the firmware watchdog (1000 ms) regardless
        # of whether cmd_vel is arriving, preventing spurious motor stops.
        if time.monotonic() - self._last_cmd_time <= CMD_HOLD_TIMEOUT:
            self._publish_f32(self._last_cmd)
        else:
            self._last_cmd = [0.0, 0.0, 0.0, 0.0]
            self._publish_f32(self._last_cmd)

    def _scale_state_for_odometry(self, velocities: list[float]) -> list[float]:
        if self._wheel_radius <= 0.0 or self._kinematic_k <= 0.0:
            return velocities

        fl, fr, rl, rr = velocities
        r = self._wheel_radius
        k = self._kinematic_k

        vx = (r / 4.0) * (fl + fr + rl + rr)
        # Mathematical inverse of this function's own wheel reconstruction
        # below (fl=vx-vy-kwz, fr=vx+vy+kwz, rl=vx+vy-kwz, rr=vx-vy+kwz).
        # 2026-08-01: this used to be negated ("-fl+fr-rl+rr flipped to
        # fl-fr-rl+rr") to supposedly correct a REP-103 mismatch — verified
        # live that this made /odom's vy sign opposite the commanded (and
        # physically executed) direction. Removed the flip.
        vy = (r / 4.0) * (-fl + fr + rl - rr)
        wz = (r / (4.0 * k)) * (-fl + fr - rl + rr)

        vx *= self._odom_linear_scale
        vy *= self._odom_lateral_scale
        wz *= self._odom_angular_scale

        if abs(vx) < self._odom_linear_deadband:
            vx = 0.0
        if abs(vy) < self._odom_lateral_deadband:
            vy = 0.0
        if abs(wz) < self._odom_angular_deadband:
            wz = 0.0

        return [
            (vx - vy - (k * wz)) / r,
            (vx + vy + (k * wz)) / r,
            (vx + vy - (k * wz)) / r,
            (vx - vy + (k * wz)) / r,
        ]

    def _on_state(self, msg: Float32MultiArray) -> None:
        """Convert Float32MultiArray wheel velocities → JointState for ros2_control."""
        self._last_esp32_time = time.monotonic()
        data = list(msg.data) if msg.data else []
        data = (data + [0.0] * 4)[:4]
        data = [
            data[i] * self._state_dir_signs[i] * self._state_scales[i]
            for i in range(4)
        ]
        data = self._scale_state_for_odometry(data)
        self._state_pub.publish(self._make_joint_state(data))


def main(args=None):
    rclpy.init(args=args)
    node = WheelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
