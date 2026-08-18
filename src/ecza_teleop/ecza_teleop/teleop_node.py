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
    angular_invert        (bool, default false) — flip joystick yaw direction
  enable_button         (int, default 5)    — dead-man button (R1)
  require_enable_button (bool, default false)
  axis_dpad_x           (int, default 4)    — D-pad horizontal: rear-axle pivot (RL/RR differential)
  axis_dpad_y           (int, default 5)    — D-pad vertical: front-axle pivot (FL/FR differential)
  btn_pivot_left        (int, default 6)    — LT: right-side wheels only forward (wide turn)
  btn_pivot_right       (int, default 7)    — RT: left-side wheels only forward (wide turn)
    btn_auto_mode         (int, default 9)    — toggle autonomous mode  (Start)
    btn_auto_mode_alt     (int, default -1)   — optional alternate toggle button
  btn_auto_mode_candidates (int[], default [9, 8]) — accepted toggle buttons
    nav_cmd_topic         (str, default /cmd_vel_nav) — Nav2 command topic in AUTO mode
    start_in_autonomous   (bool, default true) — startup in AUTO so first Start enables joystick
    auto_strafe_invert    (bool, default false) — flip AUTO lateral commands only if the rover moves reversed
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.srv import CancelGoal
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
        self.declare_parameter("angular_invert", False)
        self.declare_parameter("enable_button", 5)
        self.declare_parameter("enable_button_alt", -1)
        self.declare_parameter("require_enable_button", True)
        self.declare_parameter("axis_dpad_x", 4)
        self.declare_parameter("axis_dpad_y", 5)
        self.declare_parameter("btn_pivot_left", 6)
        self.declare_parameter("btn_pivot_right", 7)
        self.declare_parameter("wheel_base", 0.38)
        self.declare_parameter("wheel_separation_width", 0.26)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("pivot_wheel_speed_rad_s", 10.0)
        self.declare_parameter("btn_auto_mode", 9)   # Start button
        self.declare_parameter("btn_auto_mode_alt", -1)
        self.declare_parameter("btn_auto_mode_candidates", [9, 8])
        self.declare_parameter("auto_toggle_debounce_ms", 400)  # minimum interval between mode toggles
        self.declare_parameter("mode_handoff_stop_s", 0.35)
        self.declare_parameter("nav_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("auto_enable_on_nav_cmd", True)
        self.declare_parameter("auto_enable_reinstate_quiet_s", 3.0)
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
        self.declare_parameter("auto_emergency_stop_distance", 0.30)
        self.declare_parameter("avoidance_turn_speed", 0.8)
        self.declare_parameter("avoidance_strafe_speed", 0.25)
        self.declare_parameter("lidar_angle_offset_deg", 0.0)
        self.declare_parameter("strafe_invert", False)
        self.declare_parameter("auto_strafe_invert", False)

        self._ax_lx = self.get_parameter("axis_linear_x").value
        self._ax_ly = self.get_parameter("axis_linear_y").value
        self._ax_az = self.get_parameter("axis_angular_z").value
        self._angular_invert = bool(self.get_parameter("angular_invert").value)
        self._btn_en = self.get_parameter("enable_button").value
        self._btn_en_alt = self.get_parameter("enable_button_alt").value
        self._require_en = self.get_parameter("require_enable_button").value
        self._ax_dpad_x = self.get_parameter("axis_dpad_x").value
        self._ax_dpad_y = self.get_parameter("axis_dpad_y").value
        self._btn_pivot_left = self.get_parameter("btn_pivot_left").value
        self._btn_pivot_right = self.get_parameter("btn_pivot_right").value
        self._pivot_wheel_base = float(self.get_parameter("wheel_base").value)
        self._pivot_wheel_sep = float(self.get_parameter("wheel_separation_width").value)
        self._pivot_wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._pivot_wheel_speed = float(self.get_parameter("pivot_wheel_speed_rad_s").value)
        self._btn_auto = self.get_parameter("btn_auto_mode").value
        self._btn_auto_alt = self.get_parameter("btn_auto_mode_alt").value
        self._btn_auto_candidates = self._normalise_button_candidates(
            self.get_parameter("btn_auto_mode_candidates").value
        )
        self._auto_toggle_debounce_ms = int(self.get_parameter("auto_toggle_debounce_ms").value)
        self._mode_handoff_stop_s = float(self.get_parameter("mode_handoff_stop_s").value)
        self._nav_cmd_topic = self.get_parameter("nav_cmd_topic").value
        self._auto_enable_on_nav_cmd = bool(
            self.get_parameter("auto_enable_on_nav_cmd").value
        )
        self._auto_reinstate_quiet_s = float(
            self.get_parameter("auto_enable_reinstate_quiet_s").value
        )
        self._start_in_autonomous = self.get_parameter("start_in_autonomous").value
        self._max_lin = self.get_parameter("max_linear_speed").value
        self._max_ang = self.get_parameter("max_angular_speed").value

        # ── Axle/side pivot geometry ─────────────────────────────────────
        # LT/RT and D-pad drive one wheel pair forward while leaving the
        # opposite pair at zero, by combining a linear component with a
        # rotation component in the exact ratio (wz = linear / pivot_k)
        # that the mecanum IK needs to cancel the idle pair to zero.
        # pivot_k matches wheel_bridge.py's kinematic_k = wheel_base/2 + wheel_separation_width/2.
        # pivot_wheel_speed_rad_s is the target speed for the ACTIVE wheel pair;
        # keep it well under the firmware's MAX_SPEED_RAD_S (18.75) — the active
        # pair's demand is vx+k*wz combined, not vx or wz alone, so it saturates
        # at roughly half the speed a pure-vx or pure-wz command would tolerate.
        self._pivot_k = max(
            1e-6, (self._pivot_wheel_base / 2.0) + (self._pivot_wheel_sep / 2.0)
        )
        self._pivot_v = self._pivot_wheel_speed * self._pivot_wheel_radius
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
        self._auto_emergency_stop_dist = float(
            self.get_parameter("auto_emergency_stop_distance").value
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
        self._strafe_invert = bool(self.get_parameter("strafe_invert").value)
        self._auto_strafe_invert = bool(
            self.get_parameter("auto_strafe_invert").value
        )
        timeout_ms = self.get_parameter("joy_watchdog_timeout_ms").value

        # ── Autonomous mode state ─────────────────────────────────────────
        self._autonomous = bool(self._start_in_autonomous)  # False = TELEOP, True = AUTO
        self._prev_auto_btn = False       # edge-detect across primary/alt buttons
        self._last_mode_toggle_time = 0.0
        # Set when the operator explicitly leaves AUTO. Without it, a Nav2 goal
        # that is still running keeps publishing motion, and _nav_cmd_cb's
        # auto_enable_on_nav_cmd flips straight back to AUTO — so Start looks
        # like it does nothing and the operator can never take over while a
        # goal is active (observed live 2026-08-11: "Mode → TELEOP" followed
        # ~1 s later by "Nav2 command received -> AUTONOMOUS", every time).
        self._auto_enable_inhibited = False
        self._last_nav_motion_time = 0.0
        self._handoff_stop_until = 0.0
        # Manuel TELEOP geçişinden sonra auto_enable_on_nav_cmd soğuma süresi:
        # smoother kuyruğundaki Nav2 komutları Start'a basıldıktan hemen sonra
        # modu tekrar AUTO'ya çeviriyordu → kullanıcı AUTO'dan çıkamıyordu.
        self._nav_auto_enable_blocked_until = 0.0
        # Manuel TELEOP kilidi: Start ile TELEOP'a geçiş MUTLAKTIR. Hiçbir Nav2
        # komutu modu geri AUTO'ya çeviremez. Kilit yalnızca iki açık kullanıcı
        # eylemiyle kalkar: Start'a tekrar basmak veya A ile keşfi açmak
        # (slam_manager/exploring=True). Soğuma süresi tek başına yetmiyordu —
        # komut akışı 5 s'den uzun sürünce mod yine geri dönüyordu.
        self._manual_teleop_lock = False
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
        # /cmd_vel uses RELIABLE so Isaac Sim's default-QoS subscriber receives it.
        # (BEST_EFFORT publisher + RELIABLE subscriber = QoS incompatibility → no delivery)
        reliable_cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", reliable_cmd_qos)

        # Latched publisher so new subscribers always get the current mode.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(Bool, "autonomous_mode", latched_qos)

        self._joy_sub = self.create_subscription(Joy, "joy", self._joy_cb, 10)
        # Match Nav2/controller cmd_vel reliability in containerized deployments.
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._nav_sub = self.create_subscription(
            Twist, self._nav_cmd_topic, self._nav_cmd_cb, reliable_qos
        )
        # Cancel-goal service clients (see _cancel_nav_goals). These are the
        # action servers' own cancel services, so goals started from RViz —
        # which this node never gets a handle for — can still be dropped.
        self._nav_cancel_clients = [
            self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal"),
            self.create_client(
                CancelGoal, "/navigate_through_poses/_action/cancel_goal"
            ),
        ]

        self._scan_sub = None
        if self._enable_scan_safety:
            self._scan_sub = self.create_subscription(
                LaserScan, self._scan_topic, self._scan_cb, best_effort_qos
            )
        # RViz "2D Nav Goal" publishes to /goal_pose — treat this as an explicit
        # user intent to navigate. Clear the manual teleop lock and enter AUTO mode
        # so the robot actually responds even if the user previously pressed Start
        # to return to TELEOP mode.
        self._goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_pose_cb, reliable_qos
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
            f"dpad_axes=({self._ax_dpad_x},{self._ax_dpad_y}), "
            f"pivot_left_btn={self._btn_pivot_left}, pivot_right_btn={self._btn_pivot_right}, "
            f"auto_mode_buttons={self._auto_button_indices()}, "
            f"nav_cmd_topic={self._nav_cmd_topic}, start_in_autonomous={self._start_in_autonomous}, "
            f"require_enable={self._require_en}, "
            f"reject_extreme_axis_startup={self._reject_extreme_axis_startup}, "
            f"scan_safety={self._enable_scan_safety}({self._scan_topic}, "
            f"auto={self._enable_scan_safety_in_auto}, "
            f"stop={self._front_stop_distance:.2f}m, clear={self._front_clear_distance:.2f}m, "
            f"front={math.degrees(self._front_angle_rad):.0f}deg), "
            f"strafe_invert={self._strafe_invert}, "
            f"angular_invert={self._angular_invert}, "
            f"auto_strafe_invert={self._auto_strafe_invert}"
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

    def _cancel_nav_goals(self) -> None:
        """Ask Nav2 to drop whatever goal is running.

        An all-zero goal_info means "cancel every accepted goal" per the
        action_msgs/CancelGoal contract. Fire-and-forget: this runs inside the
        joy callback, so it must not block waiting for a reply, and there is
        nothing useful to do with the result either way.
        """
        request = CancelGoal.Request()
        for client in self._nav_cancel_clients:
            if not client.service_is_ready():
                continue
            client.call_async(request)

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
        front_total = 0   # ön konideki tüm ışınlar (geçersizler dahil)
        front_valid = 0   # ön konide geçerli mesafe döndüren ışınlar
        front_region_min = math.inf  # ±60° içindeki en yakın GEÇERLİ ışın

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Apply lidar mounting offset: sensor may be rotated relative to base_link.
            # lidar_angle_offset_deg=180 means sensor faces backward — subtract π so
            # that sensor angle ±π maps to robot front (angle 0 in shifted frame).
            shifted = math.atan2(
                math.sin(angle - self._lidar_angle_offset_rad),
                math.cos(angle - self._lidar_angle_offset_rad),
            )
            in_front = abs(shifted) <= half_angle
            if in_front:
                front_total += 1

            if not math.isfinite(distance):
                continue
            if distance < max(0.0, msg.range_min) or distance > msg.range_max:
                continue

            if in_front:
                front_valid += 1
                min_distance = min(min_distance, float(distance))
            if abs(shifted) <= math.radians(60.0):
                front_region_min = min(front_region_min, float(distance))

            # Left/right clearance sectors use shifted angle so they are in robot frame.
            # positive shifted angle = robot left (+Y), negative = robot right (-Y).
            if 0.0 <= shifted <= math.radians(110.0):
                left_clearance = min(left_clearance, float(distance))
            elif -math.radians(110.0) <= shifted < 0.0:
                right_clearance = min(right_clearance, float(distance))

        # Duvara yapışık körlük: ön konideki ışınların neredeyse tamamı geçersizse
        # VE ±60° içindeki en yakın geçerli ışın < 0.45 m ise duvar lidar min
        # menzilinin içindedir → önü sıfır mesafede DOLU say (acil durdurma).
        # Yakınlık kanıtı şart: açık alanda uzak/koyu yüzeyler de geçersiz ışın
        # döndürür — kanıtsız körlük "0.10 m engel" sanılıp robotu donduruyordu.
        if (front_total >= 12
                and front_valid <= max(2, int(front_total * 0.10))
                and front_region_min < 0.45):
            min_distance = min(min_distance, 0.10)

        self._front_obstacle_distance = min_distance
        if min_distance <= self._front_stop_distance:
            self._front_blocked = True
        elif min_distance >= self._front_clear_distance:
            self._front_blocked = False
        self._left_clearance = left_clearance
        self._right_clearance = right_clearance

    def _apply_scan_safety(self, twist: Twist) -> Twist:
        if not self._enable_scan_safety:
            return twist

        if self._autonomous:
            if self._enable_scan_safety_in_auto and twist.linear.x > 0.0 and self._front_blocked:
                safe = Twist()
                safe.linear.x = 0.0
                safe.linear.y = twist.linear.y
                turn_dir = 1.0 if self._left_clearance >= self._right_clearance else -1.0
                requested_turn = twist.angular.z
                if abs(requested_turn) < self._avoidance_turn_speed:
                    requested_turn = turn_dir * self._avoidance_turn_speed
                safe.angular.z = requested_turn
                self.get_logger().warn(
                    f"AUTO ön engel {self._front_obstacle_distance:.2f}m; ileri kesildi, açık tarafa dönülüyor "
                    f"(sol={self._left_clearance:.2f}m sağ={self._right_clearance:.2f}m)",
                    throttle_duration_sec=1.0,
                )
                return safe

            # Emergency stop remains as a second layer for very close obstacles.
            if twist.linear.x > 0.0 and self._front_obstacle_distance <= self._auto_emergency_stop_dist:
                safe = Twist()
                safe.linear.x = 0.0
                safe.linear.y = twist.linear.y
                turn_dir = 1.0 if self._left_clearance >= self._right_clearance else -1.0
                safe.angular.z = turn_dir * self._avoidance_turn_speed
                self.get_logger().warn(
                    f"AUTO acil durum dur: önde {self._front_obstacle_distance:.2f}m "
                    f"(insan/dinamik engel); açık tarafa dönülüyor",
                    throttle_duration_sec=1.0,
                )
                return safe
            return twist

        # Teleop mode: normal scan safety with configured threshold.
        if twist.linear.x <= 0.0 or not self._front_blocked:
            return twist

        safe = Twist()
        safe.linear.x = 0.0
        safe.linear.y = twist.linear.y
        safe.angular.z = twist.angular.z
        self.get_logger().warn(
            f"Ön engel {self._front_obstacle_distance:.2f}m; ileri engellendi "
            f"(sol={self._left_clearance:.2f}m sağ={self._right_clearance:.2f}m)",
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
            self._handoff_stop_until = now_s + max(0.0, self._mode_handoff_stop_s)
            if self._autonomous:
                # Operator handed control back on purpose — let Nav2 drive.
                self._auto_enable_inhibited = False
            else:
                # Operator takeover: block auto re-entry and tell Nav2 to drop
                # the running goal, otherwise it keeps replanning against a
                # target the operator has already overridden.
                self._auto_enable_inhibited = True
                self._last_nav_motion_time = now_s
                self._cancel_nav_goals()
            self._publish_mode()
            # Always stop the robot on any transition so neither Nav2 nor
            # the joystick leaves the wheels spinning during the handoff.
            self._publish_zero()
            mode_str = "AUTONOMOUS (Nav2)" if self._autonomous else "TELEOP (joystick)"
            self.get_logger().info(f"Mode → {mode_str}")
        self._prev_auto_btn = auto_btn_now

        # In AUTO mode Nav2 owns the controller, but a held dead-man plus stick
        # movement is treated as an operator takeover back to TELEOP.
        if self._autonomous:
            # 1.5 saniyelik koruma: START'a basıldıktan hemen sonra joystick
            # inputu hâlâ basılıysa anında TELEOP'a dönmesini önler.
            since_toggle = now_s - self._last_mode_toggle_time
            if since_toggle > 1.5 and self._operator_takeover_requested(msg):
                self._autonomous = False
                self._last_mode_toggle_time = now_s
                self._manual_teleop_lock = True
                self._nav_auto_enable_blocked_until = now_s + 5.0
                self._publish_mode()
                self._publish_zero()
                self.get_logger().warn("Operator joystick takeover -> TELEOP")
            else:
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
        if now_s < self._handoff_stop_until:
            self._publish_zero()
            return

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

        # ── Strafe: left stick X only ──────────────────────────────────────
        # strafe_invert=false → positive vy = RIGHT (hardware convention on this rover)
        # strafe_invert=true  → positive vy = LEFT  (ROS REP-103 convention)
        # Flip strafe_invert in rover_params.yaml if the stick direction is reversed.
        ss = -1.0 if self._strafe_invert else 1.0
        vy = ss * axis(self._ax_ly) * self._max_lin

        # ── Rotation: right stick X (analog) ───────────────────────────────
        # wz > 0 = CCW, wz < 0 = CW (REP-103; verified against real wheel spin).
        angular_sign = 1.0 if self._angular_invert else -1.0
        wz = angular_sign * axis(self._ax_az) * self._max_ang

        def dpad(idx: int, sign: float) -> bool:
            return idx < len(msg.axes) and (msg.axes[idx] * sign) > 0.5

        # ── Digital axle/side pivots: one wheel pair forward, the other
        # pair held at zero (not a simple in-place spin — see teleop-button-
        # mapping memory / rover_params.yaml comments for the wheel-pair map).
        # 2026-08-01: directions reported reversed from the intended wheel-pair
        # map (user mis-described the reference diagram) — every branch below
        # is negated relative to the first implementation so each physical
        # button/direction now drives the opposite wheel pair.
        pv, pk = self._pivot_v, self._pivot_k
        if btn(self._btn_pivot_left):     # LT → right side only forward (wide turn)
            vx += pv / 2.0
            wz += pv / (2.0 * pk)
        if btn(self._btn_pivot_right):    # RT → left side only forward (wide turn)
            vx += pv / 2.0
            wz += -pv / (2.0 * pk)
        if dpad(self._ax_dpad_x, -1.0):   # D-pad LEFT → rear axle: RR fwd, RL back
            vy += -pv / 2.0
            wz += pv / (2.0 * pk)
        if dpad(self._ax_dpad_x, 1.0):    # D-pad RIGHT → rear axle: RL fwd, RR back
            vy += pv / 2.0
            wz += -pv / (2.0 * pk)
        if dpad(self._ax_dpad_y, 1.0):    # D-pad UP → front axle: FR fwd, FL back
            vy += pv / 2.0
            wz += pv / (2.0 * pk)
        if dpad(self._ax_dpad_y, -1.0):   # D-pad DOWN → front axle: FL fwd, FR back
            vy += -pv / 2.0
            wz += -pv / (2.0 * pk)

        vx = max(-self._max_lin, min(self._max_lin, vx))
        vy = max(-self._max_lin, min(self._max_lin, vy))
        wz = max(-self._max_ang, min(self._max_ang, wz))

        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = wz
        self._publish_cmd(self._apply_scan_safety(twist))

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """RViz 2D Nav Goal: açık navigasyon isteği — manuel kilidi kaldır, AUTO'ya geç."""
        if self._manual_teleop_lock:
            self._manual_teleop_lock = False
            self.get_logger().info("RViz hedef alındı — manuel TELEOP kilidi kaldırıldı")
        if not self._autonomous:
            self._autonomous = True
            self._last_mode_toggle_time = time.monotonic()
            self._handoff_stop_until = 0.0
            self._publish_mode()
            self.get_logger().info("RViz hedef → AUTONOMOUS (Nav2)")

    def _exploring_cb(self, msg: Bool) -> None:
        """A butonu ile keşif açıldı: manuel kilidi kaldır, AUTO'ya geç."""
        if not msg.data:
            return
        self._manual_teleop_lock = False
        if not self._autonomous:
            self._autonomous = True
            self._last_mode_toggle_time = time.monotonic()
            self._handoff_stop_until = 0.0
            self._publish_mode()
            self.get_logger().info("Keşif açıldı (A) -> AUTONOMOUS (Nav2)")

    def _nav_cmd_cb(self, msg: Twist) -> None:
        if self._twist_has_motion(msg):
            self._last_nav_motion_time = time.monotonic()

        if (
            self._auto_enable_on_nav_cmd
            and not self._autonomous
            and not self._auto_enable_inhibited
            and self._twist_has_motion(msg)
            and time.monotonic() >= self._nav_auto_enable_blocked_until
        ):
            self._autonomous = True
            self._last_mode_toggle_time = time.monotonic()
            self._handoff_stop_until = 0.0
            self._publish_mode()
            self.get_logger().info(
                "Nav2 command received -> AUTONOMOUS (Nav2) "
                f"(vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} wz={msg.angular.z:.3f})"
            )

        # Forward Nav2 velocity commands only while in AUTO mode.
        if self._autonomous and time.monotonic() >= self._handoff_stop_until:
            cmd = Twist()
            cmd.linear.x = msg.linear.x
            cmd.linear.y = -msg.linear.y if self._auto_strafe_invert else msg.linear.y
            cmd.linear.z = msg.linear.z
            cmd.angular.x = msg.angular.x
            cmd.angular.y = msg.angular.y
            cmd.angular.z = msg.angular.z
            self._publish_cmd(self._apply_scan_safety(cmd))

    def _twist_has_motion(self, msg: Twist) -> bool:
        return (
            abs(msg.linear.x) > 1e-4
            or abs(msg.linear.y) > 1e-4
            or abs(msg.angular.z) > 1e-4
        )

    def _watchdog_cb(self) -> None:
        # Lift the operator-takeover inhibit once Nav2 has actually gone quiet
        # (goal cancelled or finished). A later goal can then auto-engage AUTO
        # again as usual — the inhibit only has to outlive the goal that was
        # running when the operator took over.
        if self._auto_enable_inhibited and not self._autonomous:
            quiet_for = time.monotonic() - self._last_nav_motion_time
            if quiet_for >= self._auto_reinstate_quiet_s:
                self._auto_enable_inhibited = False
                self.get_logger().info(
                    "Nav2 sustu — yeni hedefle otomatik AUTO geçişi tekrar aktif"
                )

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
        self._publish_cmd(Twist())

    def _publish_cmd(self, twist: Twist) -> None:
        self._cmd_pub.publish(twist)
        self._cmd_vel_pub.publish(twist)

    def _operator_takeover_requested(self, msg: Joy) -> bool:
        # Manual mode no longer requires LB/RB, so resting joystick drift should
        # not kick AUTO back to TELEOP. Use Start for explicit mode handoff.
        if not self._require_en:
            return False

        enabled = self._button_pressed(msg, self._btn_en) or self._button_pressed(
            msg, self._btn_en_alt
        )
        if not enabled:
            return False

        active_axes = (
            abs(self._apply_deadzone(self._raw_axis(msg, self._ax_lx))) > 0.0
            or abs(self._apply_deadzone(self._raw_axis(msg, self._ax_ly))) > 0.0
            or abs(self._apply_deadzone(self._raw_axis(msg, self._ax_az))) > 0.0
            or abs(self._apply_deadzone(self._raw_axis(msg, self._ax_dpad_x))) > 0.0
            or abs(self._apply_deadzone(self._raw_axis(msg, self._ax_dpad_y))) > 0.0
        )
        active_buttons = self._button_pressed(msg, self._btn_pivot_left) or self._button_pressed(
            msg, self._btn_pivot_right
        )
        return active_axes or active_buttons


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
