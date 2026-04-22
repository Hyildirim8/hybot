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

# Per-wheel direction correction from interactive single-wheel testimony:
# slot0->FL was inverted, slot1->FR correct, slot2->RL inverted, slot3->RR correct.
# Apply to both command and state so control/odometry remain consistent.
WHEEL_DIR_SIGN = [-1.0, 1.0, -1.0, 1.0]

# Seconds without real ESP32 data before state is considered stale
ESP32_TIMEOUT = 1.0

# Keep sending last command to ESP32 to avoid stutter on brief upstream gaps
# Must be shorter than the firmware watchdog timeout (1000 ms) so zeros arrive
# before the watchdog fires when the joystick goes idle.
CMD_HOLD_TIMEOUT = 0.4
# 25 Hz: 40 ms period gives comfortable margin below the 1000 ms watchdog while
# keeping USB CDC-ACM / XRCE load low.  RELIABLE QoS depth=1 guarantees each
# message is delivered without accumulating a retransmit backlog.
CMD_PUB_RATE_HZ = 25.0

# Clamp small per-wheel commands produced by kinematics/controller noise.
# This preserves intended pure diagonal/arc patterns by forcing near-zero
# wheel commands to exactly zero before they reach firmware.
CMD_DEADBAND_RAD_S = 0.45

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
        self._loopback_without_esp32 = bool(
            self.get_parameter("loopback_without_esp32").value
        )
        self._command_deadband = float(
            self.get_parameter("command_deadband_rad_s").value
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
            velocities[i] * WHEEL_DIR_SIGN[i]
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

    def _on_state(self, msg: Float32MultiArray) -> None:
        """Convert Float32MultiArray wheel velocities → JointState for ros2_control."""
        self._last_esp32_time = time.monotonic()
        data = list(msg.data) if msg.data else []
        data = (data + [0.0] * 4)[:4]
        data = [
            data[i] * WHEEL_DIR_SIGN[i]
            for i in range(4)
        ]
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
