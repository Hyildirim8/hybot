"""
teleop_node.py — Joy → cmd_vel for ecza-robotu mecanum rover.

Reads sensor_msgs/Joy and publishes geometry_msgs/Twist on /cmd_vel.

Parameters (from rover_params.yaml):
  axis_linear_x       (int, default 1)  — stick axis for forward/backward
  axis_linear_y       (int, default 0)  — stick axis for left/right strafe
  axis_angular_z      (int, default 3)  — stick axis for yaw rotation
  enable_button       (int, default 5)  — dead-man button index
  require_enable_button (bool, default true)
  max_linear_speed    (float, default 0.5) m/s
  max_angular_speed   (float, default 1.0) rad/s
  joy_deadzone        (float, default 0.05) [0,1)
  joy_watchdog_timeout_ms (int, default 500) ms
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_z", 3)
        self.declare_parameter("enable_button", 5)
        self.declare_parameter("require_enable_button", True)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("joy_deadzone", 0.05)
        self.declare_parameter("joy_watchdog_timeout_ms", 500)

        self._ax_lx = self.get_parameter("axis_linear_x").value
        self._ax_ly = self.get_parameter("axis_linear_y").value
        self._ax_az = self.get_parameter("axis_angular_z").value
        self._btn_en = self.get_parameter("enable_button").value
        self._require_en = self.get_parameter("require_enable_button").value
        self._max_lin = self.get_parameter("max_linear_speed").value
        self._max_ang = self.get_parameter("max_angular_speed").value
        self._deadzone = self.get_parameter("joy_deadzone").value
        timeout_ms = self.get_parameter("joy_watchdog_timeout_ms").value

        # ── Publishers / Subscribers ──────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self._joy_cb, 10)

        # ── Watchdog timer ────────────────────────────────────────────────
        self._last_joy = self.get_clock().now()
        self._watchdog = self.create_timer(
            timeout_ms / 1000.0, self._watchdog_cb
        )

        self.get_logger().info(
            f"teleop_node ready — axes lx={self._ax_lx} ly={self._ax_ly} "
            f"az={self._ax_az}, enable_btn={self._btn_en}, "
            f"require_enable={self._require_en}"
        )

    # ── Helpers ───────────────────────────────────────────────────────────

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        # Rescale so output starts at 0 just past the deadzone edge
        sign = math.copysign(1.0, value)
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy = self.get_clock().now()

        # Dead-man switch check
        enabled = True
        if self._require_en:
            if self._btn_en >= len(msg.buttons) or not msg.buttons[self._btn_en]:
                enabled = False

        if not enabled:
            self._publish_zero()
            return

        def axis(idx: int) -> float:
            if idx >= len(msg.axes):
                return 0.0
            return self._apply_deadzone(msg.axes[idx])

        twist = Twist()
        twist.linear.x = axis(self._ax_lx) * self._max_lin
        twist.linear.y = axis(self._ax_ly) * self._max_lin
        twist.angular.z = axis(self._ax_az) * self._max_ang
        self._cmd_pub.publish(twist)

    def _watchdog_cb(self) -> None:
        elapsed = (self.get_clock().now() - self._last_joy).nanoseconds * 1e-9
        timeout = self.get_parameter("joy_watchdog_timeout_ms").value / 1000.0
        if elapsed > timeout:
            self.get_logger().warn(
                f"joy watchdog: no /joy for {elapsed:.1f}s — publishing zero",
                throttle_duration_sec=5.0,
            )
            self._publish_zero()

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
