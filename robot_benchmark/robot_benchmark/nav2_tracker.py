"""Nav2 NavigateToPose hedef yaşam döngüsü izleyicisi — SALT DİNLEME.

Benchmark hedef GÖNDERMEZ. Operatör RViz'den "2D Nav Goal" verir; buradaki
izleyici yalnızca sonucu okur. Böylece robot kendiliğinden hareket etmez.

Kullanılan topic'ler (hepsi çalışan sistemde doğrulandı):
    /goal_pose                        geometry_msgs/PoseStamped     (RViz hedefi)
    /navigate_to_pose/_action/status   action_msgs/GoalStatusArray   (sonuç)
    /navigate_to_pose/_action/feedback nav2_msgs/.../FeedbackMessage (ilerleme)

Durum ayrımı action_msgs/GoalStatus sabitlerinden gelir; tahmin yoktur:
    4 SUCCEEDED, 5 CANCELED, 6 ABORTED, 2 EXECUTING, 1 ACCEPTED, 3 CANCELING
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from . import ros_names as N
from .stats import yaw_from_quaternion

try:
    from nav2_msgs.action import NavigateToPose
    FEEDBACK_TYPE = NavigateToPose.Impl.FeedbackMessage
    HAVE_NAV2 = True
except Exception:  # pragma: no cover
    FEEDBACK_TYPE = None
    HAVE_NAV2 = False


STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

TERMINAL = {"succeeded", "canceled", "aborted"}

ACTION_STATUS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


@dataclass
class GoalRecord:
    """Tek bir Nav2 hedefinin izlenen yaşam döngüsü."""

    uuid: str
    first_seen_s: float
    status: str = "accepted"
    accepted_s: float | None = None
    executing_s: float | None = None
    terminal_s: float | None = None
    # feedback'ten:
    nav_time_s: float | None = None
    recoveries: int | None = None
    distance_remaining_m: float | None = None
    feedback_count: int = 0
    # Ilk goruldugunde ZATEN sonuclanmis hedef: /navigate_to_pose/_action/status
    # TRANSIENT_LOCAL oldugu icin gec abone olan dugum, onceki kosularin bitmis
    # hedeflerini de alir. Bunlar "yeni hedef" sayilmamalidir.
    stale: bool = False

    @property
    def duration_s(self) -> float | None:
        """Kabul -> sonuç süresi (ROS saati)."""
        start = self.executing_s or self.accepted_s or self.first_seen_s
        if self.terminal_s is None or start is None:
            return None
        return self.terminal_s - start

    def as_dict(self) -> dict[str, Any]:
        d = {
            "uuid": self.uuid, "status": self.status,
            "accepted_s": self.accepted_s, "executing_s": self.executing_s,
            "terminal_s": self.terminal_s, "duration_s": self.duration_s,
            "nav_time_s": self.nav_time_s, "recoveries": self.recoveries,
            "distance_remaining_m": self.distance_remaining_m,
            "feedback_count": self.feedback_count,
        }
        return d


class Nav2GoalTracker:
    """BenchmarkMonitor'a takılan hedef izleyici (ayrı düğüm açmaz)."""

    def __init__(self, node):
        self.node = node
        self.goals: dict[str, GoalRecord] = {}
        self.order: list[str] = []
        self.goal_poses: list[dict[str, Any]] = []   # /goal_pose'tan gelen hedefler
        self.last_goal_pose: dict[str, Any] | None = None
        self.available = HAVE_NAV2

        node.create_subscription(PoseStamped, N.TOPIC_GOAL_POSE,
                                 self._goal_pose_cb, 10)
        node.create_subscription(GoalStatusArray,
                                 f"{N.ACTION_NAVIGATE_TO_POSE}/_action/status",
                                 self._status_cb, ACTION_STATUS_QOS)
        if HAVE_NAV2:
            node.create_subscription(FEEDBACK_TYPE,
                                     f"{N.ACTION_NAVIGATE_TO_POSE}/_action/feedback",
                                     self._feedback_cb, 10)

    # ── geri çağrımlar ────────────────────────────────────────────────────
    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        rec = {
            "t_s": self.node.now_s(),
            "frame": msg.header.frame_id or N.FRAME_MAP,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw_rad": yaw_from_quaternion(q.x, q.y, q.z, q.w),
        }
        self.goal_poses.append(rec)
        self.last_goal_pose = rec

    def _status_cb(self, msg: GoalStatusArray) -> None:
        now = self.node.now_s()
        for st in msg.status_list:
            uid = bytes(st.goal_info.goal_id.uuid).hex()
            name = STATUS_NAME.get(st.status, f"code_{st.status}")
            rec = self.goals.get(uid)
            if rec is None:
                rec = GoalRecord(uuid=uid, first_seen_s=now, status=name,
                                 stale=(name in TERMINAL))
                self.goals[uid] = rec
                self.order.append(uid)
            if name == "accepted" and rec.accepted_s is None:
                rec.accepted_s = now
            if name == "executing" and rec.executing_s is None:
                rec.executing_s = now
            if name in TERMINAL and rec.terminal_s is None:
                rec.terminal_s = now
            rec.status = name

    def _feedback_cb(self, msg) -> None:
        uid = bytes(msg.goal_id.uuid).hex()
        rec = self.goals.get(uid)
        if rec is None:
            rec = GoalRecord(uuid=uid, first_seen_s=self.node.now_s(),
                             status="executing")
            self.goals[uid] = rec
            self.order.append(uid)
        fb = msg.feedback
        rec.feedback_count += 1
        rec.nav_time_s = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        rec.recoveries = int(fb.number_of_recoveries)
        rec.distance_remaining_m = float(fb.distance_remaining)

    # ── sorgular ──────────────────────────────────────────────────────────
    def active_goal(self) -> GoalRecord | None:
        """Halen yürüyen hedef (executing/accepted)."""
        for uid in reversed(self.order):
            if self.goals[uid].status in ("executing", "accepted", "canceling"):
                return self.goals[uid]
        return None

    def latest_goal(self) -> GoalRecord | None:
        return self.goals[self.order[-1]] if self.order else None

    def latest_terminal(self) -> GoalRecord | None:
        for uid in reversed(self.order):
            if self.goals[uid].status in TERMINAL:
                return self.goals[uid]
        return None

    def wait_for_new_goal(self, timeout_s: float) -> GoalRecord | None:
        """Yeni bir hedef kabul edilene kadar bekler (operatör RViz'den verir).

        Latched durum topic'inden gelen ESKI hedefler atlanir:
        /navigate_to_pose/_action/status TRANSIENT_LOCAL oldugu icin gec abone
        olan dugum onceki kosularin bitmis hedeflerini de alir. Ilk
        goruldugunde zaten sonuclanmis (stale) olanlar "yeni" sayilmaz; yoksa
        dugum acilir acilmaz 0 saniyelik sahte bir 'aborted' kosu kaydediliyor.
        """
        # Latched durumu sindir: baseline bundan sonra olussun.
        self.node.spin_for(0.8)
        baseline = set(self.order)

        def fresh_uid() -> str | None:
            for uid in self.order:
                if uid in baseline or self.goals[uid].stale:
                    continue
                return uid
            return None

        if not self.node.wait_until(lambda: fresh_uid() is not None, timeout_s):
            return None
        uid = fresh_uid()
        return self.goals[uid] if uid else None

    def wait_for_terminal(self, rec: GoalRecord, timeout_s: float) -> bool:
        """Verilen hedef sonuçlanana kadar bekler."""
        return self.node.wait_until(lambda: rec.status in TERMINAL, timeout_s)
