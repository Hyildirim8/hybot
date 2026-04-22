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
import time
import datetime
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
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
        self.declare_parameter("btn_debounce_ms", 400)
        self.declare_parameter("map_save_dir", "/maps")
        self.declare_parameter("frontier_min_cells", 8)    # küçük frontierları atla
        self.declare_parameter("frontier_obstacle_padding_m", 0.35)
        self.declare_parameter("failed_goal_blacklist_radius_m", 0.45)
        self.declare_parameter("max_failed_goals", 20)
        self.declare_parameter("goal_timeout_s", 45.0)     # hedefe max bekleme
        self.declare_parameter("explore_interval_s", 3.0)  # frontier güncelleme periyodu
        self.declare_parameter("direct_explore_fallback", True)
        self.declare_parameter("direct_explore_speed", 0.18)
        self.declare_parameter("direct_explore_turn_speed", 0.45)
        self.declare_parameter("direct_explore_backup_speed", 0.12)
        self.declare_parameter("direct_explore_stop_distance", 0.55)
        self.declare_parameter("direct_explore_escape_distance", 0.35)
        self.declare_parameter("direct_explore_front_angle_deg", 45.0)

        self._btn_save     = self.get_parameter("btn_save_map").value
        self._btn_explore  = self.get_parameter("btn_explore_toggle").value
        self._debounce_ms  = self.get_parameter("btn_debounce_ms").value
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
        self._direct_stop_distance = float(
            self.get_parameter("direct_explore_stop_distance").value
        )
        self._direct_escape_distance = float(
            self.get_parameter("direct_explore_escape_distance").value
        )
        self._direct_front_angle_rad = math.radians(
            float(self.get_parameter("direct_explore_front_angle_deg").value)
        )

        # ── Durum ─────────────────────────────────────────────────────────────
        self._exploring: bool              = False
        self._map: Optional[OccupancyGrid] = None
        self._autonomous: bool             = False
        self._goal_handle                  = None
        self._active_goal: Optional[Tuple[float, float]] = None
        self._goal_sent_at: float          = 0.0
        self._failed_goals: List[Tuple[float, float]] = []
        self._direct_explore_active: bool   = False
        self._front_obstacle_distance: float = math.inf
        self._left_clearance: float         = math.inf
        self._right_clearance: float        = math.inf
        self._prev_save_btn: bool          = False
        self._prev_explore_btn: bool       = False
        self._last_btn_t: dict             = {"save": 0.0, "explore": 0.0}

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
        self._cmd_pub       = self.create_publisher(Twist,  "/cmd_vel_nav", best_effort)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(Joy,          "joy",             self._joy_cb,  10)
        self.create_subscription(OccupancyGrid, "/map",           self._map_cb,  latched)
        self.create_subscription(Bool,          "autonomous_mode", self._mode_cb, latched)
        self.create_subscription(LaserScan,     "/scan",          self._scan_cb, best_effort)

        # ── Servis istemcisi (harita kaydet) ──────────────────────────────────
        self._slam_save_cli = self.create_client(SlamToolboxSaveMap, "/slam_toolbox/save_map")
        self._nav2_save_cli = self.create_client(Nav2SaveMap, "/map_saver/save_map")

        # ── Nav2 aksiyon istemcisi (keşif hedefleri) ─────────────────────────
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Keşif zamanlayıcısı ───────────────────────────────────────────────
        self._explore_timer = self.create_timer(explore_interval, self._explore_tick)
        self._direct_timer = self.create_timer(0.2, self._direct_explore_tick)

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

    # ── Abonelik geri çağrıları ───────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _mode_cb(self, msg: Bool) -> None:
        self._autonomous = msg.data
        if not self._autonomous and self._goal_handle is not None:
            self._cancel_goal()
        if not self._autonomous:
            self._direct_explore_active = False
            self._publish_zero()

    def _scan_cb(self, msg: LaserScan) -> None:
        front = math.inf
        left = math.inf
        right = math.inf
        half_angle = max(0.0, self._direct_front_angle_rad / 2.0)

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < max(0.0, msg.range_min) or distance > msg.range_max:
                continue
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle:
                front = min(front, float(distance))

            # Turn away from the closest front-side return. This intentionally
            # overlaps the front cone so angled walls influence the escape side.
            if 0.0 <= angle <= math.radians(120.0):
                left = min(left, float(distance))
            elif -math.radians(120.0) <= angle < 0.0:
                right = min(right, float(distance))

        self._front_obstacle_distance = front
        self._left_clearance = left
        self._right_clearance = right

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
            self._publish_zero()

    def _explore_tick(self) -> None:
        """Keşif zamanlayıcısı: yeni frontier varsa Nav2 hedefi gönder."""
        if not self._exploring:
            return

        if not self._autonomous:
            self.get_logger().warn(
                "Keşif için AUTO mod gerekli (Start butonuna basın)",
                throttle_duration_sec=5.0,
            )
            self._direct_explore_active = False
            return

        if self._map is None:
            self._enable_direct_explore("map bekleniyor")
            return

        # Aktif bir hedef varsa ve zaman aşımına uğramamışsa bekle
        if self._goal_handle is not None:
            elapsed = time.monotonic() - self._goal_sent_at
            if elapsed < self._goal_timeout:
                return
            # Zaman aşımı — iptal et
            self._blacklist_active_goal()
            self._cancel_goal()
            self._enable_direct_explore("Nav2 hedef zaman aşımı")

        target = self._pick_frontier()
        if target is None:
            self.get_logger().info(
                "Frontier bulunamadı — haritalama tamamlanmış olabilir",
                throttle_duration_sec=10.0,
            )
            self._pub_status("Frontier yok — haritalama tamamlandı?")
            self._enable_direct_explore("frontier yok")
            return

        self._send_goal(target[0], target[1])

    # ── Frontier bulma ────────────────────────────────────────────────────────

    def _pick_frontier(self) -> Optional[Tuple[float, float]]:
        """
        /map üzerinde frontier hücrelerini bul, en büyük kümenin
        dünya koordinatlarındaki merkezini döndür.
        """
        m      = self._map
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

        candidates = []
        for cluster in valid:
            target = cluster_target(cluster)
            if self._goal_is_blacklisted(target):
                continue
            candidates.append((len(cluster), target))

        if not candidates:
            return None

        # Büyük frontier tercih edilir; başarısız hedefler kara listeye alınır.
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
            self.get_logger().warn("Keşif hedefi reddedildi")
            self._blacklist_active_goal()
            self._enable_direct_explore("Nav2 hedefi reddetti")
            return
        self._direct_explore_active = False
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
            status = getattr(result, "status", 0)
            if status != 4:  # 4 = STATUS_SUCCEEDED
                self._blacklist_active_goal()
                self._enable_direct_explore("Nav2 hedefi tamamlanmadı")
        except Exception:
            self._blacklist_active_goal()
            self._enable_direct_explore("Nav2 hedef sonucu alınamadı")
        self._goal_handle = None
        self._active_goal = None
        self._goal_sent_at = 0.0   # hemen bir sonraki frontier'ı ara

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

    def _direct_explore_tick(self) -> None:
        if not (self._direct_fallback_enabled and self._direct_explore_active):
            return
        if not (self._exploring and self._autonomous):
            self._direct_explore_active = False
            self._publish_zero()
            return

        cmd = Twist()
        turn_left = self._left_clearance >= self._right_clearance
        turn = self._direct_turn_speed if turn_left else -self._direct_turn_speed

        if self._front_obstacle_distance <= self._direct_escape_distance:
            cmd.linear.x = -self._direct_backup_speed
            cmd.angular.z = turn
            self.get_logger().warn(
                f"Duvar çok yakın ({self._front_obstacle_distance:.2f}m); geri kaçış manevrası",
                throttle_duration_sec=1.0,
            )
        elif self._front_obstacle_distance <= self._direct_stop_distance:
            cmd.linear.x = -0.5 * self._direct_backup_speed
            cmd.angular.z = turn
            self.get_logger().warn(
                f"Ön engel ({self._front_obstacle_distance:.2f}m); geri dönerek kaçış",
                throttle_duration_sec=1.0,
            )
        else:
            cmd.linear.x = self._direct_speed
        self._cmd_pub.publish(cmd)

    def _publish_zero(self) -> None:
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
