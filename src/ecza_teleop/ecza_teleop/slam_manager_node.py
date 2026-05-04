"""
slam_manager_node.py — SLAM haritalama ve otonom frontier keşfi yöneticisi.

Joystick tuşları (Logitech F710, D-modu):
  A (btn 1) — otonom frontier keşfini aç/kapat
  Y (btn 3) — mevcut SLAM haritasını /maps/ dizinine kaydet

Keşif modunda robot:
  1. /map yayınından bilinmeyen alanlara sınır (frontier) hücreleri bulur
  2. En büyük frontier kümesinin merkezine Nav2 hedefi gönderir
  3. Hedefe ulaşıldığında / başarısız olduğunda bir sonraki frontier'a geçer
  4. Tüm yüzey kaplanınca veya A'ya tekrar basılınca durur

Gereksinimler:
  - slam_toolbox çalışıyor olmalı  (/slam_toolbox/save_map servisi)
  - Nav2 çalışıyor olmalı           (navigate_to_pose action server)
  - Keşif için robot AUTO modunda olmalı  (Start butonu ile geçiş)
"""

import math
import os
import threading
import time
import datetime
from typing import List, Optional, Tuple

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Joy, LaserScan
from nav2_msgs.srv import SaveMap as Nav2SaveMap
from slam_toolbox.srv import SaveMap as SlamToolboxSaveMap
from std_msgs.msg import Bool, String

from nav2_msgs.action import NavigateToPose


class SlamManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_manager")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter("btn_save_map", 3)          # Y butonu
        self.declare_parameter("btn_explore_toggle", 1)    # A butonu
        self.declare_parameter("btn_reset_explore", 2)     # B butonu — keşfi sıfırla
        self.declare_parameter("btn_reset_pose", 10)        # L3  — AMCL poz sıfırlama
        self.declare_parameter("btn_debounce_ms", 400)
        self.declare_parameter("visited_penalty_radius_m", 2.0)  # oda hafızası yarıçapı
        self.declare_parameter("map_save_dir", "/maps")
        self.declare_parameter("frontier_min_cells", 8)    # küçük frontierları atla
        self.declare_parameter("frontier_obstacle_padding_m", 0.35)
        self.declare_parameter("failed_goal_blacklist_radius_m", 0.45)
        self.declare_parameter("max_failed_goals", 20)
        self.declare_parameter("goal_timeout_s", 45.0)     # hedefe max bekleme
        self.declare_parameter("explore_interval_s", 3.0)  # frontier güncelleme periyodu
        self.declare_parameter("lidar_angle_offset_deg", 0.0)
        self.declare_parameter("direct_explore_fallback", True)
        self.declare_parameter("direct_explore_speed", 0.18)
        self.declare_parameter("direct_explore_turn_speed", 0.45)
        self.declare_parameter("direct_explore_backup_speed", 0.12)
        self.declare_parameter("direct_explore_strafe_speed", 0.30)
        self.declare_parameter("direct_explore_stop_distance", 0.55)
        self.declare_parameter("direct_explore_escape_distance", 0.35)
        self.declare_parameter("direct_explore_front_angle_deg", 45.0)
        self.declare_parameter("min_side_clearance_m", 0.35)
        self.declare_parameter("direct_explore_kp", 1.8)          # P-kazancı
        self.declare_parameter("direct_explore_lookahead_deg", 140.0)  # açık yön arama açısı
        self.declare_parameter("direct_explore_react_distance", 1.5)   # engel tepki mesafesi
        self.declare_parameter("direct_explore_recovery_backup_time_s", 0.9)
        self.declare_parameter("direct_explore_recovery_rotate_time_s", 1.1)
        self.declare_parameter("strafe_invert", False)

        self._btn_save       = self.get_parameter("btn_save_map").value
        self._btn_explore    = self.get_parameter("btn_explore_toggle").value
        self._btn_reset      = self.get_parameter("btn_reset_explore").value
        self._btn_pose_reset = self.get_parameter("btn_reset_pose").value
        self._debounce_ms    = self.get_parameter("btn_debounce_ms").value
        self._visited_penalty_radius_sq = (
            float(self.get_parameter("visited_penalty_radius_m").value) ** 2
        )
        self._map_dir      = self.get_parameter("map_save_dir").value
        self._min_frontier = self.get_parameter("frontier_min_cells").value
        self._frontier_padding_m = float(
            self.get_parameter("frontier_obstacle_padding_m").value
        )
        self._blacklist_radius_m = float(
            self.get_parameter("failed_goal_blacklist_radius_m").value
        )
        self._max_failed_goals = int(self.get_parameter("max_failed_goals").value)
        self._goal_timeout = self.get_parameter("goal_timeout_s").value
        explore_interval   = self.get_parameter("explore_interval_s").value
        self._direct_fallback_enabled = bool(
            self.get_parameter("direct_explore_fallback").value
        )
        self._direct_speed = float(self.get_parameter("direct_explore_speed").value)
        self._direct_turn_speed = float(
            self.get_parameter("direct_explore_turn_speed").value
        )
        self._direct_backup_speed = float(
            self.get_parameter("direct_explore_backup_speed").value
        )
        self._direct_strafe_speed = float(
            self.get_parameter("direct_explore_strafe_speed").value
        )
        self._direct_stop_distance = float(
            self.get_parameter("direct_explore_stop_distance").value
        )
        self._direct_escape_distance = float(
            self.get_parameter("direct_explore_escape_distance").value
        )
        self._direct_front_angle_rad = math.radians(
            float(self.get_parameter("direct_explore_front_angle_deg").value)
        )
        self._lidar_angle_offset_rad = math.radians(
            float(self.get_parameter("lidar_angle_offset_deg").value)
        )
        self._min_side_clearance = float(
            self.get_parameter("min_side_clearance_m").value
        )
        self._direct_kp = float(self.get_parameter("direct_explore_kp").value)
        self._direct_lookahead_rad = math.radians(
            float(self.get_parameter("direct_explore_lookahead_deg").value)
        )
        self._direct_react_distance = float(
            self.get_parameter("direct_explore_react_distance").value
        )
        self._recovery_backup_time = float(
            self.get_parameter("direct_explore_recovery_backup_time_s").value
        )
        self._recovery_rotate_time = float(
            self.get_parameter("direct_explore_recovery_rotate_time_s").value
        )
        _strafe_invert = bool(self.get_parameter("strafe_invert").value)
        # strafe_invert=false → positive vy = RIGHT (hardware), true → positive vy = LEFT (ROS)
        self._strafe_h = 1.0 if not _strafe_invert else -1.0

        # ── Durum ─────────────────────────────────────────────────────────────
        self._exploring: bool              = False
        self._map: Optional[OccupancyGrid] = None
        self._autonomous: bool             = False
        self._goal_handle                  = None
        self._active_goal: Optional[Tuple[float, float]] = None
        self._goal_sent_at: float          = 0.0
        self._side_left_clearance: float   = math.inf   # 50°–90° sol taraf
        self._side_right_clearance: float  = math.inf   # 50°–90° sağ taraf
        self._back_obstacle_distance: float = math.inf  # arka mesafe (150°–180°)
        self._escape_turn_sign: float      = 0.0        # kalıcı dönüş yönü (sallanmayı önle)
        self._escape_turn_set_at: float    = 0.0
        self._frontier_target_angle: Optional[float] = None  # robot frame frontier açısı
        self._best_open_angle: float       = 0.0        # P-kontrolcü hedef açısı (±70°, robot çerçevesi)
        self._best_escape_angle: float     = 0.0        # Kaçış için en açık yön (360°, ön öncelikli)
        self._smooth_angle: float          = 0.0        # düşük geçişli filtrelenmiş hedef açı
        self._recovery_active: bool        = False
        self._recovery_started_at: float   = 0.0
        self._recovery_turn_sign: float    = 0.0
        # Velocity smoothing — exponential low-pass filter (alpha=0.55 at 10 Hz)
        # Eliminates sharp step-changes in velocity commands; PID-like ramp behaviour.
        self._smooth_vx: float = 0.0
        self._smooth_vy: float = 0.0
        self._smooth_wz: float = 0.0
        self._SMOOTH_ALPHA: float = 0.55   # lower = smoother, higher = more responsive
        self._failed_goals: List[Tuple[float, float]] = []
        self._direct_explore_active: bool   = False
        self._frontier_lock = threading.Lock()
        self._frontier_thread: Optional[threading.Thread] = None
        self._frontier_generation: int = 0
        self._pending_frontier: Optional[Tuple[float, float]] = None
        self._front_obstacle_distance: float = math.inf
        self._left_clearance: float         = math.inf
        self._right_clearance: float        = math.inf
        self._front_sparse: bool            = False   # dar koni ışınları seyrekse ince engel var
        self._prev_save_btn: bool          = False
        self._prev_explore_btn: bool       = False
        self._prev_reset_btn: bool         = False
        self._prev_pose_btn: bool          = False
        self._last_btn_t: dict             = {"save": 0.0, "explore": 0.0, "reset": 0.0, "pose": 0.0}
        self._visited_positions: List[Tuple[float, float]] = []
        self._last_visited_t: float        = 0.0

        # ── QoS profilleri ────────────────────────────────────────────────────
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._status_pub    = self.create_publisher(String, "slam_manager/status", latched)
        self._exploring_pub = self.create_publisher(Bool,   "slam_manager/exploring", latched)
        self._cmd_pub          = self.create_publisher(Twist,  "/cmd_vel_nav", best_effort)
        self._initialpose_pub  = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(Joy,          "joy",             self._joy_cb,  10)
        self.create_subscription(OccupancyGrid, "/map",           self._map_cb,  latched)
        self.create_subscription(Bool,          "autonomous_mode", self._mode_cb, latched)
        self.create_subscription(LaserScan,     "/scan",          self._scan_cb, best_effort)

        # ── TF (frontier yön hesaplama) ───────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Servis istemcisi (harita kaydet) ──────────────────────────────────
        self._slam_save_cli = self.create_client(SlamToolboxSaveMap, "/slam_toolbox/save_map")
        self._nav2_save_cli = self.create_client(Nav2SaveMap, "/map_saver/save_map")

        # ── Nav2 aksiyon istemcisi (keşif hedefleri) ─────────────────────────
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Keşif zamanlayıcısı ───────────────────────────────────────────────
        self._explore_timer = self.create_timer(explore_interval, self._explore_tick)
        self._direct_timer = self.create_timer(0.1, self._direct_explore_tick)

        self._pub_status("Hazır: A=keşif aç/kapat  Y=harita kaydet")
        self.get_logger().info(
            "slam_manager_node başlatıldı "
            f"(direct_fallback={self._direct_fallback_enabled}, "
            f"speed={self._direct_speed:.2f}, turn={self._direct_turn_speed:.2f})"
        )

    # ── Joystick geri çağrısı ─────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        now = time.monotonic()
        deb = self._debounce_ms / 1000.0

        # Y butonu → harita kaydet
        save_on = self._btn_save < len(msg.buttons) and bool(msg.buttons[self._btn_save])
        if save_on and not self._prev_save_btn and (now - self._last_btn_t["save"]) >= deb:
            self._last_btn_t["save"] = now
            self._save_map()
        self._prev_save_btn = save_on

        # A butonu → keşif modunu aç/kapat
        exp_on = self._btn_explore < len(msg.buttons) and bool(msg.buttons[self._btn_explore])
        if exp_on and not self._prev_explore_btn and (now - self._last_btn_t["explore"]) >= deb:
            self._last_btn_t["explore"] = now
            self._toggle_exploration()
        self._prev_explore_btn = exp_on

        # B butonu → keşfi sıfırla (kara liste temizle, yeniden başlat)
        rst_on = self._btn_reset < len(msg.buttons) and bool(msg.buttons[self._btn_reset])
        if rst_on and not self._prev_reset_btn and (now - self._last_btn_t["reset"]) >= deb:
            self._last_btn_t["reset"] = now
            self._reset_exploration()
        self._prev_reset_btn = rst_on

        # L3 butonu → AMCL poz sıfırla (/initialpose)
        pose_on = self._btn_pose_reset < len(msg.buttons) and bool(msg.buttons[self._btn_pose_reset])
        if pose_on and not self._prev_pose_btn and (now - self._last_btn_t["pose"]) >= deb:
            self._last_btn_t["pose"] = now
            self._reset_pose()
        self._prev_pose_btn = pose_on

    # ── Abonelik geri çağrıları ───────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _mode_cb(self, msg: Bool) -> None:
        self._autonomous = msg.data
        if not self._autonomous and self._goal_handle is not None:
            self._cancel_goal()
        if not self._autonomous:
            self._direct_explore_active = False
            self._invalidate_frontier_target()
            self._escape_turn_sign = 0.0
            self._publish_zero()

    def _scan_cb(self, msg: LaserScan) -> None:
        front = math.inf
        left = math.inf
        right = math.inf
        side_left = math.inf   # doğrudan sol taraf (50°–90°) — dar boşluk tespiti
        side_right = math.inf  # doğrudan sağ taraf (50°–90°) — dar boşluk tespiti
        back = math.inf        # arka (150°–180°) — geri gitme güvenliği
        half_angle = max(0.0, self._direct_front_angle_rad / 2.0)

        # P-kontrolcü: sektörlere bölerek en açık yön açısını bul.
        # Her sektörde minimum mesafeyi tut. Sensör verisi olmayan sektör
        # (açık alan) react_distance kadar açık sayılır — koridorda düz gidiş.
        sector_deg = 5
        n_sectors = int(2 * math.degrees(self._direct_lookahead_rad) / sector_deg)
        sector_min = [math.inf] * n_sectors   # her sektörün en yakın engeli
        lookahead = self._direct_lookahead_rad

        # 360° kaçış sektör analizi (10° çözünürlük, ön 180° öncelikli)
        _ESC_DEG = 10
        _N_ESC   = 36
        esc_min  = [math.inf] * _N_ESC

        # Dar koni (±15°) sayaçları — ince engel (sandalye ayağı vb.) tespiti.
        # Geçersiz (inf) ışınları da sayabilmek için açıyı erken hesapla.
        _NARROW_RAD = math.radians(15.0)
        narrow_total = 0
        narrow_valid = 0

        for i, distance in enumerate(msg.ranges):
            # Açıyı ve shifted'ı erken hesapla — inf ışınlar dahil her şeyi sayabilmek için.
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            shifted = math.atan2(
                math.sin(angle - self._lidar_angle_offset_rad),
                math.cos(angle - self._lidar_angle_offset_rad),
            )

            # Dar koni sayacı: geçerli ve geçersiz (inf) ışınları say
            if abs(shifted) <= _NARROW_RAD:
                narrow_total += 1
                if (math.isfinite(distance)
                        and distance >= max(0.0, msg.range_min)
                        and distance <= msg.range_max):
                    narrow_valid += 1

            if not math.isfinite(distance):
                continue
            if distance < max(0.0, msg.range_min) or distance > msg.range_max:
                continue

            if abs(shifted) <= half_angle:
                front = min(front, float(distance))

            # Left/right clearance sectors use shifted angle so they are in robot frame.
            # positive shifted angle = robot left (+Y), negative = robot right (-Y).
            if 0.0 <= shifted <= math.radians(120.0):
                left = min(left, float(distance))
            elif -math.radians(120.0) <= shifted < 0.0:
                right = min(right, float(distance))

            # Narrow side sectors (50°–90°) detect walls/poles directly beside the robot.
            # This catches thin pillars and corridor walls before the robot enters.
            side_lo = math.radians(50.0)
            side_hi = math.radians(90.0)
            if side_lo <= shifted <= side_hi:
                side_left = min(side_left, float(distance))
            elif -side_hi <= shifted <= -side_lo:
                side_right = min(side_right, float(distance))

            # Back sector (±150°–180°): used to decide if reversing is safe.
            if abs(shifted) >= math.radians(150.0):
                back = min(back, float(distance))

            # P-kontrolcü sektör analizi: her sektörde en yakın engeli kaydet.
            if abs(shifted) <= lookahead and n_sectors > 0:
                sector_width = 2.0 * lookahead / n_sectors
                idx = int((shifted + lookahead) / sector_width)
                idx = max(0, min(n_sectors - 1, idx))
                sector_min[idx] = min(sector_min[idx], float(distance))

            # 360° kaçış analizi: shifted ∈ (-π, π] → 0-360° indeksine çevir
            shifted_360 = math.degrees(shifted) % 360.0
            esc_idx = int(shifted_360 / _ESC_DEG) % _N_ESC
            esc_min[esc_idx] = min(esc_min[esc_idx], float(distance))

        # İnce engel (sandalye ayağı, metal çubuk) tespiti:
        # Dar konide (±15°) ışınların çoğu inf dönüyorsa yansıyan/geçen ince cisim var.
        # 0 < narrow_valid < %40: bazı ışınlar çarptı ama çoğu cisimden geçti.
        # narrow_total >= 20: yeterli örnek (RPLidar A2M12 bu konide ~65 ışın üretir).
        self._front_sparse = (
            narrow_total >= 20
            and 0 < narrow_valid < narrow_total * 0.40
        )

        # En açık sektör: en büyük minimum mesafeye sahip sektör.
        # Sensörü gelmeyen sektör (açık alan) react_distance ile puanlanır.
        # İleri yön önyargısı: eşit puanlı sektörlerde orta (düz) sektörü tercih et.
        # Bu açık alanda tüm sektörler eşit olduğunda sol/sağ sallanmayı önler.
        if n_sectors > 0:
            best_idx = 0
            best_score = -1.0
            sector_width = 2.0 * lookahead / n_sectors
            center_idx = n_sectors // 2
            # Frontier sektörü: hangi sector_idx frontier yönüne bakıyor?
            ft_idx: Optional[int] = None
            with self._frontier_lock:
                frontier_target_angle = self._frontier_target_angle
            if (frontier_target_angle is not None
                    and abs(frontier_target_angle) <= lookahead):
                ft_idx = int((frontier_target_angle + lookahead) / sector_width)
                ft_idx = max(0, min(n_sectors - 1, ft_idx))
            for i in range(n_sectors):
                score = sector_min[i] if sector_min[i] != math.inf else self._direct_react_distance
                fwd_bias = 0.10 * max(0.0, 1.0 - abs(i - center_idx) / max(1, center_idx))
                # Keşfedilmemiş alana (frontier) doğru yön bonusu — aynı yerden geçmeyi azalt
                frontier_bias = 0.0
                if ft_idx is not None:
                    dist_ft = abs(i - ft_idx)
                    frontier_bias = 0.40 * max(0.0, 1.0 - dist_ft / max(1, n_sectors // 4))
                if score + fwd_bias + frontier_bias > best_score:
                    best_score = score + fwd_bias + frontier_bias
                    best_idx = i
            self._best_open_angle = -lookahead + (best_idx + 0.5) * sector_width
        else:
            self._best_open_angle = 0.0

        # En açık kaçış yönü (360°, ön 180° öncelikli)
        best_esc_score = -1.0
        best_esc_angle = 0.0
        for ei in range(_N_ESC):
            center_deg = (ei + 0.5) * _ESC_DEG          # 0..360
            # -180..+180 normalize: 0=ileri, +90=sol, -90=sağ, ±180=arka
            centered = center_deg if center_deg <= 180.0 else center_deg - 360.0
            clearance = esc_min[ei] if esc_min[ei] != math.inf else self._direct_react_distance
            # Ön 180°'ye bonus: 0°=+0.5, 90°=+0.25, 180°=0
            front_bonus = 0.5 * max(0.0, 1.0 - abs(centered) / 180.0)
            score = clearance + front_bonus
            if score > best_esc_score:
                best_esc_score = score
                best_esc_angle = math.radians(centered)
        self._best_escape_angle = best_esc_angle

        self._front_obstacle_distance = front
        self._left_clearance = left
        self._right_clearance = right
        self._side_left_clearance = side_left
        self._side_right_clearance = side_right
        self._back_obstacle_distance = back

    # ── Poz sıfırlama (AMCL /initialpose) ───────────────────────────────────

    def _reset_pose(self) -> None:
        """L3: mevcut TF konumunu /initialpose olarak yayınla (AMCL parçacık filtresi sıfırla)."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
            pose_msg.pose.pose.position.x = tf.transform.translation.x
            pose_msg.pose.pose.position.y = tf.transform.translation.y
            pose_msg.pose.pose.orientation = tf.transform.rotation
            info = f"({tf.transform.translation.x:.2f}, {tf.transform.translation.y:.2f})"
        except Exception:
            pose_msg.pose.pose.orientation.w = 1.0
            info = "orijin (TF yok)"
        # Orta büyüklükte kovaryans: AMCL bu nokta etrafında parçacık yayar
        pose_msg.pose.covariance[0]  = 0.25   # x
        pose_msg.pose.covariance[7]  = 0.25   # y
        pose_msg.pose.covariance[35] = 0.07   # yaw
        self._initialpose_pub.publish(pose_msg)
        self.get_logger().info(f"Poz sıfırlandı → {info}")
        self._pub_status(f"Poz sıfırlandı {info}")

    # ── Harita kaydetme ───────────────────────────────────────────────────────

    def _save_map(self) -> None:
        ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{self._map_dir}/slam_{ts}"
        self.get_logger().info(f"Harita kaydediliyor: {name}")
        self._pub_status(f"Kaydediliyor: {name}")

        try:
            os.makedirs(self._map_dir, exist_ok=True)
        except OSError as exc:
            msg = f"Harita dizini oluşturulamadı: {self._map_dir} ({exc})"
            self.get_logger().error(msg)
            self._pub_status(msg)
            return

        if self._slam_save_cli.wait_for_service(timeout_sec=1.0):
            req = SlamToolboxSaveMap.Request()
            req.name.data = name
            future = self._slam_save_cli.call_async(req)
            future.add_done_callback(lambda f: self._on_slam_saved(f, name))
            return

        if self._nav2_save_cli.wait_for_service(timeout_sec=1.0):
            req = Nav2SaveMap.Request()
            req.map_topic = "/map"
            req.map_url = name
            req.image_format = "pgm"
            req.map_mode = "trinary"
            req.free_thresh = 0.25
            req.occupied_thresh = 0.65
            future = self._nav2_save_cli.call_async(req)
            future.add_done_callback(lambda f: self._on_nav2_saved(f, name))
            return

        msg = "Harita kayıt servisi bulunamadı (/slam_toolbox/save_map veya /map_saver/save_map)"
        self.get_logger().warn(msg)
        self._pub_status(msg)

    def _on_slam_saved(self, future, name: str) -> None:
        try:
            resp = future.result()
            if getattr(resp, "result", None) == 0:
                self.get_logger().info(f"Harita kaydedildi: {name}.yaml")
                self._pub_status(f"Kaydedildi: {name}.yaml")
            else:
                self.get_logger().warn(f"Kayıt hatası, kod: {getattr(resp, 'result', 'bilinmiyor')}")
                self._pub_status("Harita kayıt hatası")
        except Exception as exc:
            self.get_logger().error(f"Harita kayıt hatası: {exc}")
            self._pub_status(f"Harita kayıt hatası: {exc}")

    def _on_nav2_saved(self, future, name: str) -> None:
        try:
            resp = future.result()
            if getattr(resp, "result", False):
                self.get_logger().info(f"Harita kaydedildi: {name}.yaml")
                self._pub_status(f"Kaydedildi: {name}.yaml")
            else:
                self.get_logger().warn("Nav2 map_saver kayıt hatası")
                self._pub_status("Harita kayıt hatası")
        except Exception as exc:
            self.get_logger().error(f"Harita kayıt hatası: {exc}")
            self._pub_status(f"Harita kayıt hatası: {exc}")

    # ── Keşif modu ────────────────────────────────────────────────────────────

    def _reset_exploration(self) -> None:
        """B butonu: kara listeyi temizle, durumu sıfırla, keşfi yeniden başlat."""
        self._failed_goals.clear()
        self._visited_positions.clear()
        self._last_visited_t = 0.0
        self._cancel_goal()
        self._direct_explore_active = False
        self._pending_frontier = None
        self._escape_turn_sign = 0.0
        self._smooth_angle = 0.0
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_wz = 0.0
        self._invalidate_frontier_target()
        self._publish_zero()
        self.get_logger().info("Keşif sıfırlandı (B butonu)")
        self._pub_status("Keşif sıfırlandı — B butonu")
        if self._exploring and self._autonomous:
            self._enable_direct_explore("sıfırdan başlat")

    def _toggle_exploration(self) -> None:
        self._exploring = not self._exploring
        state = "AÇIK" if self._exploring else "KAPALI"
        self.get_logger().info(f"Otonom keşif: {state}")
        self._pub_status(f"Keşif {state}")
        m = Bool()
        m.data = self._exploring
        self._exploring_pub.publish(m)

        if not self._exploring:
            self._cancel_goal()
            self._direct_explore_active = False
            self._pending_frontier = None
            self._invalidate_frontier_target()
            self._escape_turn_sign = 0.0
            self._publish_zero()

    def _explore_tick(self) -> None:
        """Keşif zamanlayıcısı: Nav2 hedefli frontier exploration.

        Akış:
          1. Arka plan thread'den hazır frontier varsa → Nav2 hedefi gönder.
          2. Nav2 aktif geziyorsa    → sadece timeout kontrolü yap, karışma.
          3. Boşta (hedef yok)       → background thread başlat; o sırada direct_explore.
        """
        if not self._exploring:
            return

        if not self._autonomous:
            self.get_logger().warn(
                "Keşif için AUTO mod gerekli (Start butonuna basın)",
                throttle_duration_sec=5.0,
            )
            self._direct_explore_active = False
            return

        # Ziyaret edilen konumu periyodik kaydet (oda hafızası için)
        self._record_visited_position()

        # ── 1. Arka plandan frontier hazırsa Nav2 hedefi gönder ─────────────
        if self._pending_frontier is not None:
            frontier = self._pending_frontier
            self._pending_frontier = None
            self._direct_explore_active = False   # Nav2 devralıyor
            self._send_goal(frontier[0], frontier[1])
            return

        # ── 2. Nav2 aktif geziyorsa — sadece timeout izle ────────────────────
        if self._goal_handle is not None:
            elapsed = time.monotonic() - self._goal_sent_at
            if elapsed > self._goal_timeout:
                self.get_logger().warn(
                    f"Hedef zaman aşımı ({elapsed:.0f}s) — iptal ediliyor"
                )
                self._cancel_goal()
                self._blacklist_active_goal()
                self._enable_direct_explore("hedef zaman aşımı")
            return  # Nav2 sürüyor, karışma

        # ── 3. Boşta: arka planda frontier hesapla, o sırada direct_explore ─
        if self._frontier_thread is None or not self._frontier_thread.is_alive():
            snap = self._map
            if snap is not None:
                with self._frontier_lock:
                    generation = self._frontier_generation
                visited_snap = list(self._visited_positions)
                self._frontier_thread = threading.Thread(
                    target=self._compute_frontier_bg, args=(snap, generation, visited_snap), daemon=True
                )
                self._frontier_thread.start()

        self._enable_direct_explore("frontier hesaplanıyor")

    def _invalidate_frontier_target(self) -> None:
        """Eski background sonuçlarını geçersiz kıl ve frontier açısını temizle."""
        with self._frontier_lock:
            self._frontier_generation += 1
            self._frontier_target_angle = None

    def _compute_frontier_bg(self, map_snap: OccupancyGrid, generation: int,
                             visited_snap: List[Tuple[float, float]]) -> None:
        """Arka plan thread: frontier bul, yön açısı hesapla, main thread'e bildir."""
        frontier = self._pick_frontier_from(map_snap, visited_snap)

        with self._frontier_lock:
            if generation != self._frontier_generation:
                return   # keşif iptal edildi, sonucu çöp

        if frontier is None:
            self._frontier_target_angle = None
            self._pending_frontier = None
            self.get_logger().info(
                "Tüm frontierlar keşfedildi veya kara listede",
                throttle_duration_sec=10.0,
            )
            return

        # Direktif sürüş için yön tavsiyesi (Nav2 beklenirken direct_explore'u yönlendirir)
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            rx  = tf.transform.translation.x
            ry  = tf.transform.translation.y
            q   = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            dx = frontier[0] - rx
            dy = frontier[1] - ry
            world_angle = math.atan2(dy, dx)
            with self._frontier_lock:
                if generation == self._frontier_generation:
                    self._frontier_target_angle = math.atan2(
                        math.sin(world_angle - yaw),
                        math.cos(world_angle - yaw),
                    )
        except Exception:
            pass

        # Main thread'e sinyal: bir sonraki _explore_tick'te Nav2 hedefi gönder
        self._pending_frontier = frontier

    # ── Frontier bulma ────────────────────────────────────────────────────────

    def _record_visited_position(self) -> None:
        """Keşif sırasında robot konumunu 15 saniyede bir ziyaret listesine ekle."""
        if not (self._exploring and self._autonomous):
            return
        now = time.monotonic()
        if now - self._last_visited_t < 15.0:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            pos = (tf.transform.translation.x, tf.transform.translation.y)
            self._visited_positions.append(pos)
            if len(self._visited_positions) > 200:
                self._visited_positions = self._visited_positions[-200:]
            self._last_visited_t = now
        except Exception:
            pass

    def _pick_frontier_from(self, m: OccupancyGrid,
                            visited: List[Tuple[float, float]] = None) -> Optional[Tuple[float, float]]:
        """
        Verilen harita üzerinde frontier hücrelerini bul, en büyük kümenin
        dünya koordinatlarındaki merkezini döndür.
        """
        w, h   = m.info.width, m.info.height
        res    = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data   = m.data            # int8 tuple: 0=serbest, -1/255=bilinmeyen, 100=engel

        if w == 0 or h == 0:
            return None

        obstacle_padding_cells = max(1, int(math.ceil(self._frontier_padding_m / res)))

        def near_obstacle(x: int, y: int) -> bool:
            x0 = max(0, x - obstacle_padding_cells)
            x1 = min(w, x + obstacle_padding_cells + 1)
            y0 = max(0, y - obstacle_padding_cells)
            y1 = min(h, y + obstacle_padding_cells + 1)
            for oy_i in range(y0, y1):
                row_i = oy_i * w
                for ox_i in range(x0, x1):
                    if data[row_i + ox_i] >= 50:
                        return True
            return False

        # Frontier hücreleri bul: serbest (0), bilinmeyene komşu ve engelden uzak
        frontier_set: set = set()
        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                if data[row + x] != 0 or near_obstacle(x, y):
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nb = data[(y + dy) * w + (x + dx)]
                    if nb == -1 or nb == 255:
                        frontier_set.add((x, y))
                        break

        if not frontier_set:
            return None

        # BFS ile frontier hücrelerini kümele
        clusters: List[List[Tuple[int, int]]] = []
        remaining = set(frontier_set)
        while remaining:
            seed    = next(iter(remaining))
            cluster = []
            queue   = [seed]
            while queue:
                cell = queue.pop()
                if cell not in remaining:
                    continue
                remaining.discard(cell)
                cluster.append(cell)
                cx, cy = cell
                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        nb = (cx + dx, cy + dy)
                        if nb in remaining:
                            queue.append(nb)
            clusters.append(cluster)

        # Yeterince büyük kümeleri filtrele
        valid = [c for c in clusters if len(c) >= self._min_frontier]
        if not valid:
            return None

        def cluster_target(cluster: List[Tuple[int, int]]) -> Tuple[float, float]:
            cx_i = sum(p[0] for p in cluster) / len(cluster)
            cy_i = sum(p[1] for p in cluster) / len(cluster)
            return (ox + cx_i * res, oy + cy_i * res)

        # Ziyaret edilen bölgeler için skoru önceden hesapla (arka plan thread'i, GIL korumalı)
        visited_sq = self._visited_penalty_radius_sq

        def is_near_visited(wx: float, wy: float) -> bool:
            if not visited:
                return False
            for vx, vy in visited:
                if (wx - vx) ** 2 + (wy - vy) ** 2 <= visited_sq:
                    return True
            return False

        candidates = []
        for cluster in valid:
            target = cluster_target(cluster)
            if self._goal_is_blacklisted(target):
                continue
            score = float(len(cluster))
            if is_near_visited(target[0], target[1]):
                score *= 0.4   # ziyaret edilmiş bölgeye 60% ceza → yeni alanları tercih et
            candidates.append((score, target))

        if not candidates:
            return None

        # En yüksek skorlu frontier (büyük + ziyaret edilmemiş) tercih edilir.
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    # ── Nav2 aksiyonu ─────────────────────────────────────────────────────────

    def _send_goal(self, wx: float, wy: float) -> None:
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose sunucusu hazır değil")
            self._enable_direct_explore("Nav2 hazır değil")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Keşif hedefi: ({wx:.2f}, {wy:.2f})")
        self._pub_status(f"Keşif → ({wx:.2f}, {wy:.2f})")
        self._goal_sent_at = time.monotonic()
        self._goal_handle  = None
        self._active_goal = (wx, wy)

        send_fut = self._nav.send_goal_async(goal)
        send_fut.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Keşif hedefi reddedildi — fallback")
            self._blacklist_active_goal()
            self._active_goal = None
            self._enable_direct_explore("hedef reddedildi")
            return
        self._goal_handle = handle
        self._direct_explore_active = False   # Nav2 navige ediyor, direct_explore durur
        self._pub_status(f"Nav2 → ({self._active_goal[0]:.1f}, {self._active_goal[1]:.1f})")
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
            status = getattr(result, "status", 0)
            if status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info("Frontier başarıyla tamamlandı — sonraki aranıyor")
                self._pub_status("Frontier ✓")
                # direct_explore açma: _explore_tick bir sonraki çevrimde yeni frontier bulur.
                # Ara boşlukta direct_explore aktif olsun ki robot beklemeden devam etsin.
                self._enable_direct_explore("frontier tamamlandı")
            else:
                self.get_logger().warn(f"Nav2 hedefi başarısız (status={status})")
                self._blacklist_active_goal()
                self._enable_direct_explore("Nav2 hedefi başarısız")
        except Exception as exc:
            self.get_logger().error(f"Nav2 sonuç hatası: {exc}")
            self._blacklist_active_goal()
            self._enable_direct_explore("Nav2 sonuç hatası")
        self._goal_handle = None
        self._active_goal = None
        self._goal_sent_at = 0.0

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._active_goal = None

    def _enable_direct_explore(self, reason: str) -> None:
        if not self._direct_fallback_enabled:
            return
        if not self._direct_explore_active:
            self.get_logger().warn(
                f"Doğrudan keşif fallback aktif: {reason}",
                throttle_duration_sec=5.0,
            )
            self._pub_status(f"Fallback keşif: {reason}")
        self._direct_explore_active = True

    def _choose_turn_sign(self) -> float:
        """Kaçış yönünü belirle: sol/sağ açıklık karşılaştırması (0°–120° sektörler)."""
        left  = self._left_clearance  if math.isfinite(self._left_clearance)  else 10.0
        right = self._right_clearance if math.isfinite(self._right_clearance) else 10.0
        if left > right + 0.15:
            return 1.0    # sol açık → sola dön (CCW, +angular.z)
        if right > left + 0.15:
            return -1.0   # sağ açık → sağa dön (CW, -angular.z)
        # Fark az: P-kontrolcünün sektör analizine bak (±70°)
        if abs(self._best_open_angle) > 0.15:
            return 1.0 if self._best_open_angle > 0.0 else -1.0
        return -1.0

    def _apply_velocity_smoothing(self, cmd: Twist, emergency: bool) -> Twist:
        a = 0.85 if emergency else self._SMOOTH_ALPHA
        self._smooth_vx = a * cmd.linear.x + (1.0 - a) * self._smooth_vx
        self._smooth_vy = a * cmd.linear.y + (1.0 - a) * self._smooth_vy
        self._smooth_wz = a * cmd.angular.z + (1.0 - a) * self._smooth_wz
        cmd.linear.x = self._smooth_vx
        cmd.linear.y = self._smooth_vy
        cmd.angular.z = self._smooth_wz
        return cmd

    def _direct_explore_tick(self) -> None:
        if not (self._direct_fallback_enabled and self._direct_explore_active):
            return
        if not (self._exploring and self._autonomous):
            self._direct_explore_active = False
            self._escape_turn_sign = 0.0
            self._publish_zero()
            return

        cmd = Twist()
        front_dist = self._front_obstacle_distance

        # İnce engel (seyrek tarama) tespiti: mesafeye yakınsa durma eşiklerini sıkılaştır.
        # Sandalye ayağı gibi cisimler lidar ışınlarını sıkça yansıtır/geçirir.
        _sparse = (self._front_sparse
                   and front_dist < 1.5 * self._direct_stop_distance)
        eff_stop   = self._direct_stop_distance   * (1.30 if _sparse else 1.0)
        eff_escape = self._direct_escape_distance * (1.30 if _sparse else 1.0)

        front_escape = front_dist <= eff_escape
        narrow_left  = self._side_left_clearance < self._min_side_clearance
        narrow_right = self._side_right_clearance < self._min_side_clearance
        too_narrow   = narrow_left and narrow_right
        back_blocked = self._back_obstacle_distance < 0.30

        in_trouble = front_escape or too_narrow

        # ── Strafe yönü (mecanum yan hareket) ────────────────────────────────
        # 50°–90° sektör açıklığı: robot gövdesinin yan duvar mesafesi.
        # Minimum 0.32 m yoksa o yana kayma — daha az çarpma riski.
        # ROS convention (+y=sol) kullan: teleop auto_strafe_invert=True bunu
        # donanım convention'ına (+y=sağ) çevirir — Nav2/DWB ile aynı.
        _STRAFE_MIN = 0.32
        side_r = self._side_right_clearance if math.isfinite(self._side_right_clearance) else 10.0
        side_l = self._side_left_clearance  if math.isfinite(self._side_left_clearance)  else 10.0
        can_strafe_right = side_r > _STRAFE_MIN
        can_strafe_left  = side_l > _STRAFE_MIN

        if can_strafe_right and side_r >= side_l:
            strafe_y = -self._direct_strafe_speed      # ROS: -y = sağ
        elif can_strafe_left:
            strafe_y = +self._direct_strafe_speed      # ROS: +y = sol
        else:
            strafe_y = 0.0                             # yan yol yok

        if in_trouble:
            # ── Sensor-driven escape ───────────────────────────────────────────
            # Yön kilidi: trouble başladığında seç, tamamen çıkana kadar tut.
            # Her tick yeniden seçmek → sallanma yaratır.
            if self._escape_turn_sign == 0.0:
                self._escape_turn_sign = self._choose_turn_sign()
            sign = self._escape_turn_sign

            if back_blocked:
                # Arkası da kapalı — yerinde dön + yana kayma ile çıkış dene
                cmd.angular.z = sign * self._direct_turn_speed
                cmd.linear.y  = strafe_y
                self.get_logger().warn(
                    f"Ön+arka kapalı (ön={front_dist:.2f}m); yerinde dön+kayma",
                    throttle_duration_sec=1.0)
            else:
                # Geri git düz — rotasyon yok, dairesel hareketi önle.
                # Ön engel çekilince stop-zone P-kontrolcüsü yön düzeltmesini devralır.
                cmd.linear.x  = -self._direct_backup_speed
                cmd.linear.y  = strafe_y * 0.5
                self.get_logger().warn(
                    f"Engel kaçışı (ön={front_dist:.2f}m); düz geri",
                    throttle_duration_sec=1.0)
        else:
            # ── Serbest sürüş ─────────────────────────────────────────────────
            # Kaçış kilidi sıfırla — yön tekrar P-kontrolcüye bırakılır.
            self._escape_turn_sign = 0.0

            self._smooth_angle = 0.70 * self._best_open_angle + 0.30 * self._smooth_angle
            error = self._smooth_angle

            if front_dist >= eff_stop:
                # Açık yol: ileri git + hafif yön düzeltmesi.
                # Duvara yakın (< 0.40m yan) → hafif düzeltme kayması.
                cmd.linear.x  = self._direct_speed
                cmd.angular.z = max(-self._direct_turn_speed,
                                    min(self._direct_turn_speed,
                                        self._direct_kp * 0.50 * error))
                if min(side_l, side_r) < 0.40:
                    cmd.linear.y = strafe_y * 0.35
            else:
                # Stop zone (eff_escape < front < eff_stop): ileri gitme, dön + tam yana kayma.
                # Mecanum strafing ile köşe etrafında kaymak dönmeden çok daha hızlı.
                cmd.linear.x  = 0.0
                cmd.angular.z = max(-self._direct_turn_speed,
                                    min(self._direct_turn_speed,
                                        self._direct_kp * 1.0 * error))
                cmd.linear.y  = strafe_y

        cmd = self._apply_velocity_smoothing(cmd, emergency=in_trouble)
        self._cmd_pub.publish(cmd)

    def _publish_zero(self) -> None:
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_wz = 0.0
        self._cmd_pub.publish(Twist())

    def _goal_is_blacklisted(self, target: Tuple[float, float]) -> bool:
        radius_sq = self._blacklist_radius_m * self._blacklist_radius_m
        for wx, wy in self._failed_goals:
            dx = target[0] - wx
            dy = target[1] - wy
            if (dx * dx + dy * dy) <= radius_sq:
                return True
        return False

    def _blacklist_active_goal(self) -> None:
        if self._active_goal is None:
            return
        self._failed_goals.append(self._active_goal)
        if len(self._failed_goals) > self._max_failed_goals:
            self._failed_goals = self._failed_goals[-self._max_failed_goals:]

    # ── Yardımcı ─────────────────────────────────────────────────────────────

    def _pub_status(self, text: str) -> None:
        m      = String()
        m.data = text
        self._status_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
