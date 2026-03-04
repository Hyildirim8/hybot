"""
teleop_node.py — Joy → cmd_vel for ecza-robotu mecanum rover.

Reads sensor_msgs/Joy and publishes geometry_msgs/Twist on /cmd_vel.

Modes
-----
  TELEOP (default) — joystick drives the robot via /cmd_vel.
  AUTO             — joystick output is muted; Nav2 owns /cmd_vel.
                     Press btn_auto_mode again to return to TELEOP.

On TELEOP → AUTO:  publishes zero Twist so the robot stops before Nav2 takes over.
On AUTO → TELEOP:  publishes zero Twist to cancel any ongoing Nav2 motion.

The current mode is broadcast on /autonomous_mode (std_msgs/Bool, latched)
  True  = autonomous (Nav2 in control)
  False = teleop     (joystick in control)

Parameters (from rover_params.yaml):
  axis_linear_x         (int, default 1)    — stick axis for forward/backward
  axis_linear_y         (int, default 0)    — stick axis for left/right strafe
  axis_angular_z        (int, default 3)    — stick axis for yaw rotation
  enable_button         (int, default 5)    — dead-man button (R1)
  require_enable_button (bool, default false)
  btn_strafe_left       (int, default 6)    — full-speed strafe left  (LT)
  btn_strafe_right      (int, default 7)    — full-speed strafe right (RT)
  btn_auto_mode         (int, default 9)    — toggle autonomous mode  (Start)
  max_linear_speed      (float, default 1.5) m/s
  max_angular_speed     (float, default 3.0) rad/s
  joy_deadzone          (float, default 0.05)
  joy_watchdog_timeout_ms (int, default 500) ms
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_z", 3)
        self.declare_parameter("enable_button", 5)
        self.declare_parameter("require_enable_button", True)
        self.declare_parameter("btn_strafe_left", 6)
        self.declare_parameter("btn_strafe_right", 7)
        self.declare_parameter("btn_auto_mode", 9)   # Start button
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("joy_deadzone", 0.05)
        self.declare_parameter("joy_watchdog_timeout_ms", 500)

        self._ax_lx = self.get_parameter("axis_linear_x").value
        self._ax_ly = self.get_parameter("axis_linear_y").value
        self._ax_az = self.get_parameter("axis_angular_z").value
        self._btn_en = self.get_parameter("enable_button").value
        self._require_en = self.get_parameter("require_enable_button").value
        self._btn_sl = self.get_parameter("btn_strafe_left").value
        self._btn_sr = self.get_parameter("btn_strafe_right").value
        self._btn_auto = self.get_parameter("btn_auto_mode").value
        self._max_lin = self.get_parameter("max_linear_speed").value
        self._max_ang = self.get_parameter("max_angular_speed").value
        self._deadzone = self.get_parameter("joy_deadzone").value
        timeout_ms = self.get_parameter("joy_watchdog_timeout_ms").value

        # ── Autonomous mode state ─────────────────────────────────────────
        self._autonomous = False          # False = TELEOP, True = AUTO
        self._prev_auto_btn = False       # edge-detect: avoid holding-button repeat

        # ── Publishers / Subscribers ──────────────────────────────────────
        # Use BEST_EFFORT QoS to match mecanum_drive_controller's subscription
        # (ros2_control ChainableControllerInterface uses BEST_EFFORT for reference topics)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", best_effort_qos)

        # Latched publisher so new subscribers always get the current mode.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(Bool, "autonomous_mode", latched_qos)

        self._joy_sub = self.create_subscription(Joy, "joy", self._joy_cb, 10)

        # Publish initial mode (TELEOP) immediately so late subscribers see it.
        self._publish_mode()

        # ── Watchdog timer ────────────────────────────────────────────────
        self._last_joy = self.get_clock().now()
        self._watchdog = self.create_timer(
            timeout_ms / 1000.0, self._watchdog_cb
        )

        self.get_logger().info(
            f"teleop_node ready — axes lx={self._ax_lx} ly={self._ax_ly} "
            f"az={self._ax_az}, enable_btn={self._btn_en}, "
            f"strafe_left_btn={self._btn_sl}, strafe_right_btn={self._btn_sr}, "
            f"auto_mode_btn={self._btn_auto}, require_enable={self._require_en}"
        )

    # ── Helpers ───────────────────────────────────────────────────────────

    def _publish_mode(self) -> None:
        """Publish current mode on /autonomous_mode (latched)."""
        msg = Bool()
        msg.data = self._autonomous
        self._mode_pub.publish(msg)

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        # Rescale so output starts at 0 just past the deadzone edge
        sign = math.copysign(1.0, value)
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy = self.get_clock().now()

        # ── Autonomous mode toggle (edge-detect to avoid repeat on hold) ──
        auto_btn_now = (
            self._btn_auto < len(msg.buttons)
            and bool(msg.buttons[self._btn_auto])
        )
        if auto_btn_now and not self._prev_auto_btn:
            self._autonomous = not self._autonomous
            self._publish_mode()
            # Always stop the robot on any transition so neither Nav2 nor
            # the joystick leaves the wheels spinning during the handoff.
            self._publish_zero()
            mode_str = "AUTONOMOUS (Nav2)" if self._autonomous else "TELEOP (joystick)"
            self.get_logger().info(f"Mode → {mode_str}")
        self._prev_auto_btn = auto_btn_now

        # In AUTO mode the joystick is completely muted — Nav2 owns /cmd_vel.
        if self._autonomous:
            return

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

        def btn(idx: int) -> bool:
            return idx < len(msg.buttons) and bool(msg.buttons[idx])

        # ── Forward / backward (stick) ─────────────────────────────────
        vx = axis(self._ax_lx) * self._max_lin

        # ── Strafe: stick axis PLUS dedicated buttons (additive, clamped) ─
        # Left stick X: pushing right = axis +1 = robot strafe right = vy -1
        # Button LT(6) → strafe left:  FL+ FR- RL- RR+  (vy = -max_lin)
        # Button RT(7) → strafe right: FL- FR+ RL+ RR-  (vy = +max_lin)
        # NOTE: vy sign is negated vs naive ROS +y=left convention because the
        # mecanum_drive_controller uses the opposite vy sign internally for this
        # rover's roller orientation.
        vy_stick = axis(self._ax_ly) * self._max_lin
        vy_btn = 0.0
        if btn(self._btn_sl):
            vy_btn -= self._max_lin   # strafe left:  FL+ FR- RL- RR+
        if btn(self._btn_sr):
            vy_btn += self._max_lin   # strafe right: FL- FR+ RL+ RR-
        vy = max(-self._max_lin, min(self._max_lin, vy_stick + vy_btn))

        # ── Rotation: right stick Y — push up = CCW (+wz) ─────────────
        wz = axis(self._ax_az) * self._max_ang

        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = wz
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
