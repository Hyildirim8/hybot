"""
slam_manager_node.py — SLAM haritalama ve otonom frontier keşfi yöneticisi.

Joystick tuşları (Logitech F710, D-modu):
  A (btn 1) — otonom frontier keşfini aç/kapat
  Y (btn 3) — mevcut SLAM haritasını /maps/ dizinine kaydet

Keşif modunda robot:
  1. /map yayınından bilinmeyen alanlara sınır (frontier) hücreleri bulur
  2. En büyük frontier kümesinin merkezine Nav2 hedefi gönderir
  3. Hedefe ulaşıldığında / başarısız olduğunda bir sonraki frontier'a geçer
  4. Tüm yüzey kaplanınca veya B'ye tekrar basılınca durur

Gereksinimler:
  - slam_toolbox çalışıyor olmalı  (/slam_toolbox/save_map servisi)
  - Nav2 çalışıyor olmalı           (navigate_to_pose action server)
  - Keşif için robot AUTO modunda olmalı  (Start butonu ile geçiş)
"""

import math
import time
import datetime
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Bool, String

from nav2_msgs.action import NavigateToPose


class SlamManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_manager")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter("btn_save_map", 3)          # Y butonu
        self.declare_parameter("btn_explore_toggle", 1)    # B butonu
        self.declare_parameter("btn_debounce_ms", 400)
        self.declare_parameter("map_save_dir", "/maps")
        self.declare_parameter("frontier_min_cells", 8)    # küçük frontierları atla
        self.declare_parameter("goal_timeout_s", 45.0)     # hedefe max bekleme
        self.declare_parameter("explore_interval_s", 3.0)  # frontier güncelleme periyodu

        self._btn_save     = self.get_parameter("btn_save_map").value
        self._btn_explore  = self.get_parameter("btn_explore_toggle").value
        self._debounce_ms  = self.get_parameter("btn_debounce_ms").value
        self._map_dir      = self.get_parameter("map_save_dir").value
        self._min_frontier = self.get_parameter("frontier_min_cells").value
        self._goal_timeout = self.get_parameter("goal_timeout_s").value
        explore_interval   = self.get_parameter("explore_interval_s").value

        # ── Durum ─────────────────────────────────────────────────────────────
        self._exploring: bool              = False
        self._map: Optional[OccupancyGrid] = None
        self._autonomous: bool             = False
        self._goal_handle                  = None
        self._goal_sent_at: float          = 0.0
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

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(Joy,          "joy",             self._joy_cb,  10)
        self.create_subscription(OccupancyGrid, "/map",           self._map_cb,  latched)
        self.create_subscription(Bool,          "autonomous_mode", self._mode_cb, latched)

        # ── Servis istemcisi (harita kaydet) ──────────────────────────────────
        self._save_cli = self.create_client(SaveMap, "/slam_toolbox/save_map")

        # ── Nav2 aksiyon istemcisi (keşif hedefleri) ─────────────────────────
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Keşif zamanlayıcısı ───────────────────────────────────────────────
        self._explore_timer = self.create_timer(explore_interval, self._explore_tick)

        self._pub_status("Hazır: B=keşif aç/kapat  Y=harita kaydet")
        self.get_logger().info("slam_manager_node başlatıldı")

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

        # B butonu → keşif modunu aç/kapat
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

    # ── Harita kaydetme ───────────────────────────────────────────────────────

    def _save_map(self) -> None:
        ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{self._map_dir}/slam_{ts}"
        self.get_logger().info(f"Harita kaydediliyor: {name}")
        self._pub_status(f"Kaydediliyor: {name}")

        if not self._save_cli.wait_for_service(timeout_sec=2.0):
            msg = "Harita kayıt servisi bulunamadı"
            self.get_logger().warn(msg)
            self._pub_status(msg)
            return

        req = SaveMap.Request()
        req.name.data = name
        future = self._save_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_saved(f, name))

    def _on_saved(self, future, name: str) -> None:
        try:
            resp = future.result()
            if resp.result == 0:
                self.get_logger().info(f"Harita kaydedildi: {name}.yaml")
                self._pub_status(f"Kaydedildi: {name}.yaml")
            else:
                self.get_logger().warn(f"Kayıt hatası, kod: {resp.result}")
        except Exception as exc:
            self.get_logger().error(f"Harita kayıt hatası: {exc}")

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

    def _explore_tick(self) -> None:
        """Keşif zamanlayıcısı: yeni frontier varsa Nav2 hedefi gönder."""
        if not self._exploring:
            return

        if not self._autonomous:
            self.get_logger().warn(
                "Keşif için AUTO mod gerekli (Start butonuna basın)",
                throttle_duration_sec=5.0,
            )
            return

        if self._map is None:
            return

        # Aktif bir hedef varsa ve zaman aşımına uğramamışsa bekle
        if self._goal_handle is not None:
            elapsed = time.monotonic() - self._goal_sent_at
            if elapsed < self._goal_timeout:
                return
            # Zaman aşımı — iptal et
            self._cancel_goal()

        target = self._pick_frontier()
        if target is None:
            self.get_logger().info(
                "Frontier bulunamadı — haritalama tamamlanmış olabilir",
                throttle_duration_sec=10.0,
            )
            self._pub_status("Frontier yok — haritalama tamamlandı?")
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

        # Frontier hücreleri bul: serbest (0) ve bilinmeyene komşu
        frontier_set: set = set()
        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                if data[row + x] != 0:
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

        # En büyük kümeyi seç
        best = max(valid, key=len)

        # Centroid → dünya koordinatları
        cx = sum(p[0] for p in best) / len(best)
        cy = sum(p[1] for p in best) / len(best)
        return (ox + cx * res, oy + cy * res)

    # ── Nav2 aksiyonu ─────────────────────────────────────────────────────────

    def _send_goal(self, wx: float, wy: float) -> None:
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose sunucusu hazır değil")
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

        send_fut = self._nav.send_goal_async(goal)
        send_fut.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Keşif hedefi reddedildi")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_handle = None
        self._goal_sent_at = 0.0   # hemen bir sonraki frontier'ı ara

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

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
