"""
teleop_node.py — Joy → cmd_vel for ecza-robotu mecanum rover.

Reads sensor_msgs/Joy and publishes geometry_msgs/Twist to the controller reference.

Modes
-----
  TELEOP (default) — joystick drives the controller reference directly.
    AUTO             — joystick output is muted; Nav2 /cmd_vel_nav is forwarded
                                         to the controller command output.
                     Press btn_auto_mode again to return to TELEOP.

On TELEOP → AUTO:  publishes zero Twist so the robot stops before Nav2 takes over.
On AUTO → TELEOP:  publishes zero Twist to cancel any ongoing Nav2 motion.

The current mode is broadcast on /autonomous_mode (std_msgs/Bool, latched)
  True  = autonomous (Nav2 in control)
  False = teleop     (joystick in control)

Parameters (from rover_params.yaml):
  axis_linear_x         (int, default 1)    — stick axis for forward/backward
  axis_linear_y         (int, default 0)    — stick axis for left/right strafe
    axis_angular_z        (int, default 2)    — stick axis for yaw rotation
  enable_button         (int, default 5)    — dead-man button (R1)
  require_enable_button (bool, default false)
  btn_strafe_left       (int, default 6)    — full-speed strafe left  (LT)
  btn_strafe_right      (int, default 7)    — full-speed strafe right (RT)
    btn_auto_mode         (int, default 9)    — toggle autonomous mode  (Start)
    btn_auto_mode_alt     (int, default -1)   — optional alternate toggle button
    btn_auto_mode_candidates (int[], default [9, 8]) — accepted toggle buttons
    nav_cmd_topic         (str, default /cmd_vel_nav) — Nav2 command topic in AUTO mode
    start_in_autonomous   (bool, default true) — startup in AUTO so first Start enables joystick
  max_linear_speed      (float, default 1.5) m/s
  max_angular_speed     (float, default 3.0) rad/s
  joy_deadzone          (float, default 0.05)
  joy_watchdog_timeout_ms (int, default 500) ms
  reject_extreme_axis_startup (bool, default true) — ignore invalid startup
                     frames where all mapped joystick axes report ±1.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_z", 2)
        self.declare_parameter("enable_button", 5)
        self.declare_parameter("enable_button_alt", -1)
        self.declare_parameter("require_enable_button", True)
        self.declare_parameter("btn_strafe_left", 6)
        self.declare_parameter("btn_strafe_right", 7)
        self.declare_parameter("btn_auto_mode", 9)   # Start button
        self.declare_parameter("btn_auto_mode_alt", -1)
        self.declare_parameter("btn_auto_mode_candidates", [9, 8])
        self.declare_parameter("auto_toggle_debounce_ms", 400)  # minimum interval between mode toggles
        self.declare_parameter("nav_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("start_in_autonomous", True)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("joy_deadzone", 0.05)
        self.declare_parameter("joy_watchdog_timeout_ms", 500)
        self.declare_parameter("reject_extreme_axis_startup", True)
        self.declare_parameter("enable_scan_safety", True)
        self.declare_parameter("enable_scan_safety_in_auto", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("front_obstacle_stop_distance", 0.45)
        self.declare_parameter("front_obstacle_clear_distance", 0.75)
        self.declare_parameter("front_obstacle_angle_deg", 35.0)
        self.declare_parameter("avoidance_turn_speed", 0.8)
        self.declare_parameter("avoidance_strafe_speed", 0.25)
        self.declare_parameter("lidar_angle_offset_deg", 0.0)

        self._ax_lx = self.get_parameter("axis_linear_x").value
        self._ax_ly = self.get_parameter("axis_linear_y").value
        self._ax_az = self.get_parameter("axis_angular_z").value
        self._btn_en = self.get_parameter("enable_button").value
        self._btn_en_alt = self.get_parameter("enable_button_alt").value
        self._require_en = self.get_parameter("require_enable_button").value
        self._btn_sl = self.get_parameter("btn_strafe_left").value
        self._btn_sr = self.get_parameter("btn_strafe_right").value
        self._btn_auto = self.get_parameter("btn_auto_mode").value
        self._btn_auto_alt = self.get_parameter("btn_auto_mode_alt").value
        self._btn_auto_candidates = self._normalise_button_candidates(
            self.get_parameter("btn_auto_mode_candidates").value
        )
        self._auto_toggle_debounce_ms = int(self.get_parameter("auto_toggle_debounce_ms").value)
        self._nav_cmd_topic = self.get_parameter("nav_cmd_topic").value
        self._start_in_autonomous = self.get_parameter("start_in_autonomous").value
        self._max_lin = self.get_parameter("max_linear_speed").value
        self._max_ang = self.get_parameter("max_angular_speed").value
        self._deadzone = self.get_parameter("joy_deadzone").value
        self._reject_extreme_axis_startup = bool(
            self.get_parameter("reject_extreme_axis_startup").value
        )
        self._enable_scan_safety = bool(self.get_parameter("enable_scan_safety").value)
        self._enable_scan_safety_in_auto = bool(
            self.get_parameter("enable_scan_safety_in_auto").value
        )
        self._scan_topic = self.get_parameter("scan_topic").value
        self._front_stop_distance = float(
            self.get_parameter("front_obstacle_stop_distance").value
        )
        self._front_clear_distance = float(
            self.get_parameter("front_obstacle_clear_distance").value
        )
        self._front_angle_rad = math.radians(
            float(self.get_parameter("front_obstacle_angle_deg").value)
        )
        self._avoidance_turn_speed = float(
            self.get_parameter("avoidance_turn_speed").value
        )
        self._avoidance_strafe_speed = float(
            self.get_parameter("avoidance_strafe_speed").value
        )
        self._lidar_angle_offset_rad = math.radians(
            float(self.get_parameter("lidar_angle_offset_deg").value)
        )
        timeout_ms = self.get_parameter("joy_watchdog_timeout_ms").value

        # ── Autonomous mode state ─────────────────────────────────────────
        self._autonomous = bool(self._start_in_autonomous)  # False = TELEOP, True = AUTO
        self._prev_auto_btn = False       # edge-detect across primary/alt buttons
        self._last_mode_toggle_time = 0.0
        self._axes_ready = not self._reject_extreme_axis_startup
        self._front_blocked = False
        self._front_obstacle_distance = math.inf
        self._left_clearance = math.inf
        self._right_clearance = math.inf

        # ── Publishers / Subscribers ──────────────────────────────────────
        # Use BEST_EFFORT QoS to match mecanum_drive_controller's subscription
        # (ros2_control ChainableControllerInterface uses BEST_EFFORT for reference topics)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_pub = self.create_publisher(
            Twist, "/controller_manager/reference_unstamped", best_effort_qos
        )

        # Latched publisher so new subscribers always get the current mode.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(Bool, "autonomous_mode", latched_qos)

        self._joy_sub = self.create_subscription(Joy, "joy", self._joy_cb, 10)
        # Match Nav2/controller cmd_vel reliability in containerized deployments.
        self._nav_sub = self.create_subscription(
            Twist, self._nav_cmd_topic, self._nav_cmd_cb, best_effort_qos
        )
        self._scan_sub = None
        if self._enable_scan_safety:
            self._scan_sub = self.create_subscription(
                LaserScan, self._scan_topic, self._scan_cb, best_effort_qos
            )

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
            f"enable_btn_alt={self._btn_en_alt}, "
            f"strafe_left_btn={self._btn_sl}, strafe_right_btn={self._btn_sr}, "
            f"auto_mode_buttons={self._auto_button_indices()}, "
            f"nav_cmd_topic={self._nav_cmd_topic}, start_in_autonomous={self._start_in_autonomous}, "
            f"require_enable={self._require_en}, "
            f"reject_extreme_axis_startup={self._reject_extreme_axis_startup}, "
            f"scan_safety={self._enable_scan_safety}({self._scan_topic}, "
            f"auto={self._enable_scan_safety_in_auto}, "
            f"stop={self._front_stop_distance:.2f}m, clear={self._front_clear_distance:.2f}m, "
            f"front={math.degrees(self._front_angle_rad):.0f}deg)"
        )

    # ── Helpers ───────────────────────────────────────────────────────────

    def _publish_mode(self) -> None:
        """Publish current mode on /autonomous_mode (latched)."""
        msg = Bool()
        msg.data = self._autonomous
        self._mode_pub.publish(msg)

    def _normalise_button_candidates(self, value) -> list:
        candidates = []
        if isinstance(value, (list, tuple)):
            source = value
        else:
            source = [value]
        for item in source:
            try:
                idx = int(item)
            except (TypeError, ValueError):
                continue
            if idx >= 0 and idx not in candidates:
                candidates.append(idx)
        return candidates

    def _auto_button_indices(self) -> list:
        indices = []
        for idx in (self._btn_auto, self._btn_auto_alt, *self._btn_auto_candidates):
            try:
                idx = int(idx)
            except (TypeError, ValueError):
                continue
            if idx >= 0 and idx not in indices:
                indices.append(idx)
        return indices

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        # Rescale so output starts at 0 just past the deadzone edge
        sign = math.copysign(1.0, value)
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    def _raw_axis(self, msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    def _axis_startup_frame_is_invalid(self, msg: Joy) -> bool:
        """Detect joystick startup frames that report every mapped axis at ±1."""
        mapped = [self._ax_lx, self._ax_ly, self._ax_az]
        values = [self._raw_axis(msg, idx) for idx in mapped if idx >= 0]
        if not values:
            return False
        return all(abs(abs(value) - 1.0) <= 1e-6 for value in values)

    def _button_pressed(self, msg: Joy, idx: int) -> bool:
        return idx >= 0 and idx < len(msg.buttons) and bool(msg.buttons[idx])

    def _scan_cb(self, msg: LaserScan) -> None:
        min_distance = math.inf
        left_clearance = math.inf
        right_clearance = math.inf
        half_angle = max(0.0, self._front_angle_rad / 2.0)

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < max(0.0, msg.range_min) or distance > msg.range_max:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Apply lidar mounting offset: sensor may be rotated relative to base_link.
            # lidar_angle_offset_deg=180 means sensor faces backward — subtract π so
            # that sensor angle ±π maps to robot front (angle 0 in shifted frame).
            shifted = math.atan2(
                math.sin(angle - self._lidar_angle_offset_rad),
                math.cos(angle - self._lidar_angle_offset_rad),
            )
            if abs(shifted) <= half_angle:
                min_distance = min(min_distance, float(distance))

            # Left/right clearance sectors use shifted angle so they are in robot frame.
            # positive shifted angle = robot left (+Y), negative = robot right (-Y).
            if 0.0 <= shifted <= math.radians(110.0):
                left_clearance = min(left_clearance, float(distance))
            elif -math.radians(110.0) <= shifted < 0.0:
                right_clearance = min(right_clearance, float(distance))

        self._front_obstacle_distance = min_distance
        self._front_blocked = min_distance <= self._front_stop_distance
        self._left_clearance = left_clearance
        self._right_clearance = right_clearance

    def _apply_scan_safety(self, twist: Twist) -> Twist:
        if not self._enable_scan_safety or twist.linear.x <= 0.0:
            return twist

        if self._autonomous and not self._enable_scan_safety_in_auto:
            return twist

        if not self._front_blocked:
            return twist

        safe = Twist()
        safe.linear.x = 0.0
        safe.linear.y = twist.linear.y
        safe.angular.z = twist.angular.z

        if self._autonomous:
            turn_left = self._left_clearance >= self._right_clearance
            side_clearance = self._left_clearance if turn_left else self._right_clearance
            direction = 1.0 if turn_left else -1.0

            safe.angular.z = direction * self._avoidance_turn_speed
            if side_clearance >= self._front_clear_distance:
                safe.linear.y = direction * self._avoidance_strafe_speed

        self.get_logger().warn(
            f"front obstacle at {self._front_obstacle_distance:.2f}m; "
            f"{'avoiding' if self._autonomous else 'blocking forward command'} "
            f"(left={self._left_clearance:.2f}m right={self._right_clearance:.2f}m)",
            throttle_duration_sec=1.0,
        )
        return safe

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy = self.get_clock().now()

        # ── Autonomous mode toggle (edge-detect to avoid repeat on hold) ──
        # Toggle mode only on explicitly configured button indices.
        auto_btn_now = any(
            self._button_pressed(msg, idx) for idx in self._auto_button_indices()
        )
        now_s = time.monotonic()
        debounce_s = max(0.0, float(self._auto_toggle_debounce_ms) / 1000.0)
        if auto_btn_now and not self._prev_auto_btn and (now_s - self._last_mode_toggle_time) >= debounce_s:
            self._autonomous = not self._autonomous
            self._last_mode_toggle_time = now_s
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

        if not self._axes_ready:
            if self._axis_startup_frame_is_invalid(msg):
                self.get_logger().warn(
                    "joy axes look uninitialised at startup; publishing zero until sticks move",
                    throttle_duration_sec=5.0,
                )
                self._publish_zero()
                return
            self._axes_ready = True

        # Dead-man switch check
        enabled = True
        if self._require_en:
            primary = self._btn_en < len(msg.buttons) and bool(msg.buttons[self._btn_en])
            alt = self._btn_en_alt >= 0 and self._btn_en_alt < len(msg.buttons) and bool(msg.buttons[self._btn_en_alt])
            if not (primary or alt):
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
        # ROS base_link convention (REP-103): +Y = left, -Y = right.
        # Strafe direction is intentionally inverted here to match the rover's
        # observed hardware convention for left/right motion.
        # Button LT(6) → strafe left
        # Button RT(7) → strafe right
        vy_stick = axis(self._ax_ly) * self._max_lin
        vy_btn = 0.0
        if btn(self._btn_sl):
            vy_btn -= self._max_lin
        if btn(self._btn_sr):
            vy_btn += self._max_lin
        vy = max(-self._max_lin, min(self._max_lin, vy_stick + vy_btn))

        # ── Rotation: right stick X — push left/right = CCW/CW (+/-wz) ─
        wz = axis(self._ax_az) * self._max_ang

        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = wz
        self._cmd_pub.publish(self._apply_scan_safety(twist))

    def _nav_cmd_cb(self, msg: Twist) -> None:
        # Forward Nav2 velocity commands only while in AUTO mode.
        if self._autonomous:
            self._cmd_pub.publish(self._apply_scan_safety(msg))

    def _watchdog_cb(self) -> None:
        # In AUTO mode, Nav2 commands are forwarded from _nav_cmd_cb, so the
        # joy watchdog must not inject zero commands.
        if self._autonomous:
            return
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
