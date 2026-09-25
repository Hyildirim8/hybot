"""ROS 2 ölçüm çekirdeği — SALT DİNLEME.

Bu düğüm hiçbir hareket komutu yayınlamaz. Tek yayın yapan yer
experiments/mecanum.py'dir ve o da yalnızca --allow-motion ile açık operatör
onayından sonra çalışır.

Zaman kuralı: ölçülen SÜRELER için ROS saati (node.get_clock()) kullanılır;
use_sim_time açıkken de doğru olması gerekir. Mesaj varış aralıkları (kamera
FPS gibi) için de ROS saati kullanılır, böylece bag replay tutarlı ölçülür.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CompressedImage, Joy, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from . import ros_names as N
from .stats import yaw_from_quaternion

try:  # nav2_msgs her ortamda olmayabilir (mock modu)
    from nav2_msgs.msg import BehaviorTreeLog
    HAVE_BT_LOG = True
except ImportError:  # pragma: no cover
    BehaviorTreeLog = None
    HAVE_BT_LOG = False

try:
    from tf2_ros import Buffer, TransformListener
    HAVE_TF = True
except ImportError:  # pragma: no cover
    HAVE_TF = False


SENSOR_QOS = qos_profile_sensor_data

# teleop_node /autonomous_mode'u latched yayınlar; geç abone olan son değeri
# almalı, yoksa mod hiç okunmaz.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float           # rad
    frame: str
    stamp_s: float       # ROS saati, saniye

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


class BenchmarkMonitor(Node):
    """İhtiyaca göre abone olan tek ölçüm düğümü.

    `want` kümesiyle yalnızca gereken abonelikler açılır; Pi'de gereksiz
    CPU harcamamak için (yük hâlihazırda 4 çekirdekte 4-5 seviyesinde).
    """

    ALL = frozenset({
        "pose", "wheels", "mode", "plan", "scan", "map", "camera", "bt", "cmd",
        "joy",
    })

    def __init__(self, want: set[str] | None = None, node_name: str = "robot_benchmark"):
        super().__init__(node_name)
        want = set(want or self.ALL)
        self.want = want

        # ── poz ───────────────────────────────────────────────────────────
        self.odom: Odometry | None = None
        self.odom_pose: Pose2D | None = None
        self.tf_buffer = None
        self._tf_listener = None
        if "pose" in want:
            self.create_subscription(Odometry, N.TOPIC_ODOM_FILTERED,
                                     self._odom_cb, SENSOR_QOS)
            if HAVE_TF:
                self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30))
                self._tf_listener = TransformListener(self.tf_buffer, self)

        # ── tekerlek (duruş tespiti) ──────────────────────────────────────
        self.wheel_speeds: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.wheel_stamp_s: float | None = None
        if "wheels" in want:
            self.create_subscription(Float32MultiArray, N.TOPIC_WHEEL_VELOCITIES,
                                     self._wheel_cb, SENSOR_QOS)

        # ── mod ───────────────────────────────────────────────────────────
        self.autonomous: bool | None = None
        self.autonomous_changed_at_s: float | None = None
        self.mode_history: list[tuple[float, bool]] = []
        self.slam_status: str | None = None
        self.exploring: bool | None = None
        if "mode" in want:
            self.create_subscription(Bool, N.TOPIC_AUTONOMOUS_MODE,
                                     self._mode_cb, LATCHED_QOS)
            self.create_subscription(String, N.TOPIC_SLAM_MANAGER_STATUS,
                                     self._slam_status_cb, RELIABLE_QOS)
            self.create_subscription(Bool, N.TOPIC_SLAM_MANAGER_EXPLORING,
                                     self._exploring_cb, LATCHED_QOS)

        # ── plan (yeniden planlama tespiti) ───────────────────────────────
        self.global_plan: Path | None = None
        self.plan_count = 0                  # alınan /plan mesajı sayısı
        self.replan_count = 0                # içeriği DEĞİŞEN plan sayısı
        self.first_plan_s: float | None = None
        self.first_replan_s: float | None = None
        self.planned_length_m: float | None = None      # SON planin uzunlugu
        self.first_plan_length_m: float | None = None   # ILK planin uzunlugu
        self._plan_fingerprint: tuple | None = None
        if "plan" in want:
            self.create_subscription(Path, N.TOPIC_GLOBAL_PLAN,
                                     self._plan_cb, RELIABLE_QOS)

        # ── lidar ─────────────────────────────────────────────────────────
        self.scan: LaserScan | None = None
        self.scan_count = 0
        self.min_range_m: float | None = None
        self.obstacle_events: list[dict[str, Any]] = []
        self._obstacle_active = False
        self.obstacle_threshold_m = 0.45
        if "scan" in want:
            self.create_subscription(LaserScan, N.TOPIC_SCAN,
                                     self._scan_cb, SENSOR_QOS)

        # ── harita ────────────────────────────────────────────────────────
        self.map_msg: OccupancyGrid | None = None
        self.map_count = 0
        if "map" in want:
            self.create_subscription(OccupancyGrid, N.TOPIC_MAP,
                                     self._map_cb, LATCHED_QOS)

        # ── kamera ────────────────────────────────────────────────────────
        self.cam_count = 0
        self.cam_stamps_s: list[float] = []      # ROS saati varış anları
        self.cam_header_stamps_s: list[float] = []  # mesaj header damgaları
        self.cam_sizes: list[int] = []
        self.cam_resolution: str | None = None
        self._cam_res_probed = False
        if "camera" in want:
            self.create_subscription(CompressedImage, N.TOPIC_CAMERA_COMPRESSED,
                                     self._cam_cb, SENSOR_QOS)

        # ── davranış ağacı (recovery / yeniden planlama) ───────────────────
        self.bt_events: list[dict[str, Any]] = []
        self.bt_available = HAVE_BT_LOG
        if "bt" in want and HAVE_BT_LOG:
            self.create_subscription(BehaviorTreeLog, N.TOPIC_BEHAVIOR_TREE_LOG,
                                     self._bt_cb, RELIABLE_QOS)

        # ── komut aynası (yalnızca dinleme) ───────────────────────────────
        self.last_cmd: Twist | None = None
        if "cmd" in want:
            self.create_subscription(Twist, N.TOPIC_CMD_VEL_NAV_SMOOTHED,
                                     self._cmd_cb, SENSOR_QOS)

        # ── joystick (mod gecis referansi) ────────────────────────────────
        # Mod gecisinin "istek" anini operatorun Enter'i yerine BUTONUN
        # kendisinden almak, olcumden insan reaksiyon suresini cikarir ve
        # gercek sistem gecikmesini verir. btn_auto_mode=9 (Start).
        self.joy_buttons: list[int] = []
        self.auto_button_index = 9
        self.auto_button_presses: list[float] = []   # yukselen kenar zamanlari
        self._prev_auto_button = 0
        if "joy" in want:
            self.create_subscription(Joy, "/joy", self._joy_cb, SENSOR_QOS)

        self.on_sample: Callable[[str, dict], None] | None = None

    # ── yardımcılar ───────────────────────────────────────────────────────
    def now_s(self) -> float:
        """ROS saati, saniye (use_sim_time'a saygılı)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wheels_still(self, max_age_s: float = 1.0) -> bool:
        """Dört tekerlek de duruyor mu?

        Bu oturumun en pahalı dersi: /autonomous_mode false okusa bile robot
        hareket edebiliyor. Poz/gyro ölçümü almadan önce MUTLAKA bunu sor.
        Geri besleme bayatsa "duruyor" DEME — bilinmiyor demektir.
        """
        if self.wheel_stamp_s is None:
            return False
        if self.now_s() - self.wheel_stamp_s > max_age_s:
            return False
        return sum(abs(v) for v in self.wheel_speeds) < N.WHEEL_STILL_SUM_RAD_S

    def map_pose(self, timeout_s: float = 0.0) -> Pose2D | None:
        """Robotun HARİTA çerçevesindeki pozu (map -> base_link).

        Konumlandırma doğruluğu bunun üzerinden ölçülür: Nav2 hedefi map
        çerçevesindedir, dolayısıyla hata da map çerçevesinde hesaplanmalı.
        /odometry/filtered odom çerçevesindedir ve SLAM düzeltmesini içermez.
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                N.FRAME_MAP, N.FRAME_BASE_LINK, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_s),
            )
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return Pose2D(t.x, t.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w),
                      N.FRAME_MAP,
                      tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)

    def reset_plan_stats(self) -> None:
        """Yeni hedef icin plan sayaclarini sifirla (kosular birbirine karismasin)."""
        self.plan_count = 0
        self.replan_count = 0
        self.first_plan_s = None
        self.first_replan_s = None
        self.first_plan_length_m = None
        self.planned_length_m = None
        self._plan_fingerprint = None

    def spin_for(self, seconds: float, hz: float = 50.0) -> None:
        """Belirtilen süre boyunca ROS saatine göre döner.

        Dışarıdan kapatma (SIGINT/SIGTERM -> ExternalShutdownException)
        KeyboardInterrupt'a çevrilir; böylece RunStore bağlam yöneticisi
        devreye girip o ana kadarki veriyi partial olarak kaydeder ve
        kullanıcıya yığın izi yerine temiz bir mesaj gösterilir.
        """
        end = self.now_s() + seconds
        period = 1.0 / hz
        while self.now_s() < end:
            try:
                rclpy.spin_once(self, timeout_sec=period)
            except ExternalShutdownException as exc:
                raise KeyboardInterrupt("ROS bağlamı dışarıdan kapatıldı") from exc

    def wait_until(self, predicate: Callable[[], bool], timeout_s: float,
                   hz: float = 50.0) -> bool:
        """predicate True olana kadar döner. Zaman aşımında False."""
        end = self.now_s() + timeout_s
        period = 1.0 / hz
        while self.now_s() < end:
            if predicate():
                return True
            try:
                rclpy.spin_once(self, timeout_sec=period)
            except ExternalShutdownException as exc:
                raise KeyboardInterrupt("ROS bağlamı dışarıdan kapatıldı") from exc
        return predicate()

    # ── geri çağrımlar ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        q = msg.pose.pose.orientation
        self.odom_pose = Pose2D(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
            msg.header.frame_id or N.FRAME_ODOM,
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _wheel_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)[:4]
        if len(data) == 4 and all(math.isfinite(v) for v in data):
            self.wheel_speeds = [float(v) for v in data]
            self.wheel_stamp_s = self.now_s()

    def _mode_cb(self, msg: Bool) -> None:
        value = bool(msg.data)
        if self.autonomous is None or value != self.autonomous:
            self.autonomous_changed_at_s = self.now_s()
            self.mode_history.append((self.autonomous_changed_at_s, value))
        self.autonomous = value

    def _slam_status_cb(self, msg: String) -> None:
        self.slam_status = msg.data

    def _exploring_cb(self, msg: Bool) -> None:
        self.exploring = bool(msg.data)

    def _plan_cb(self, msg: Path) -> None:
        self.global_plan = msg
        self.plan_count += 1
        now = self.now_s()
        if self.first_plan_s is None:
            self.first_plan_s = now
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        total = 0.0
        for a, b in zip(pts, pts[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        self.planned_length_m = total if pts else None
        if self.first_plan_length_m is None and pts:
            # Hedefe ulasma yaklastikca /plan KISALIR; "planlanan yol uzunlugu"
            # olarak son plani almak metrigi anlamsiz kilar (olcum: gidilen
            # 18.9 m iken son plan 0.17 m). Bu yuzden hedef kabul edildikten
            # sonraki ILK tam plan saklanir.
            self.first_plan_length_m = total
        # Parmak izi: uç noktalar + nokta sayısı + uzunluk. Aynı planın
        # tekrar yayınlanmasını "yeniden planlama" saymamak için.
        fp = (len(pts), round(total, 3),
              tuple(round(v, 3) for v in (pts[0] if pts else (0.0, 0.0))),
              tuple(round(v, 3) for v in (pts[-1] if pts else (0.0, 0.0))))
        if self._plan_fingerprint is not None and fp != self._plan_fingerprint:
            self.replan_count += 1
            if self.first_replan_s is None:
                self.first_replan_s = now
        self._plan_fingerprint = fp

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_count += 1
        lo = max(0.0, msg.range_min)
        valid = [d for d in msg.ranges
                 if math.isfinite(d) and lo <= d < msg.range_max]
        self.min_range_m = min(valid) if valid else None
        # Engel olayı: eşiğin altına iniş/çıkış kenarları.
        if self.min_range_m is not None:
            close = self.min_range_m <= self.obstacle_threshold_m
            if close and not self._obstacle_active:
                self._obstacle_active = True
                self.obstacle_events.append({
                    "t_s": self.now_s(), "edge": "detected",
                    "min_range_m": round(self.min_range_m, 3),
                    "valid_rays": len(valid), "total_rays": len(msg.ranges),
                })
            elif not close and self._obstacle_active:
                self._obstacle_active = False
                self.obstacle_events.append({
                    "t_s": self.now_s(), "edge": "cleared",
                    "min_range_m": round(self.min_range_m, 3),
                    "valid_rays": len(valid), "total_rays": len(msg.ranges),
                })

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_count += 1

    def _cam_cb(self, msg: CompressedImage) -> None:
        self.cam_count += 1
        self.cam_stamps_s.append(self.now_s())
        hs = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cam_header_stamps_s.append(hs)
        self.cam_sizes.append(len(msg.data))
        if not self._cam_res_probed:
            self._cam_res_probed = True
            self.cam_resolution = _jpeg_resolution(bytes(msg.data))

    def _bt_cb(self, msg) -> None:
        for ev in msg.event_log:
            self.bt_events.append({
                "t_s": self.now_s(),
                "node_name": ev.node_name,
                "previous_status": ev.previous_status,
                "current_status": ev.current_status,
            })

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def _joy_cb(self, msg: Joy) -> None:
        self.joy_buttons = list(msg.buttons)
        idx = self.auto_button_index
        cur = int(msg.buttons[idx]) if idx < len(msg.buttons) else 0
        if cur and not self._prev_auto_button:      # yukselen kenar
            self.auto_button_presses.append(self.now_s())
        self._prev_auto_button = cur


def _jpeg_resolution(data: bytes) -> str | None:
    """JPEG başlığından genişlik×yükseklik. Çözülemezse None (uydurma yok)."""
    if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    i = 2
    n = len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            height = (data[i + 5] << 8) | data[i + 6]
            width = (data[i + 7] << 8) | data[i + 8]
            return f"{width}x{height}"
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        seg = (data[i + 2] << 8) | data[i + 3]
        if seg <= 0:
            return None
        i += 2 + seg
    return None
