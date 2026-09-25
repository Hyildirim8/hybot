"""Hedef tabanlı deneyler: navigation, obstacle, repeatability.

Üçü de aynı çekirdeği kullanır: operatör RViz'den hedef verir, biz ölçeriz.
BENCHMARK HEDEF GÖNDERMEZ — robot kendiliğinden hareket etmez.

Aralarındaki fark yalnızca hangi metriklerin öne çıkarıldığı ve operatörden
ne istendiğidir:
  navigation    : süre, mesafe, hız, başarı durumu
  obstacle      : yeniden planlama, engel algılama, recovery, çarpışma işareti
  repeatability : aynı hedefe tekrar; doğrusal/açısal hata dağılımı
"""

from __future__ import annotations

from typing import Any, Callable

from ..monitors import BenchmarkMonitor
from ..nav2_tracker import Nav2GoalTracker
from ..stats import path_length
from .common import (PoseTrack, avg_speed, bt_replan_stats, goal_error,
                     map_metrics)


def run_goal_run(monitor: BenchmarkMonitor, tracker: Nav2GoalTracker, store,
                 wait_goal_s: float = 300.0, goal_timeout_s: float = 300.0,
                 sample_hz: float = 5.0,
                 on_status: Callable[[str], None] | None = None) -> dict[str, Any]:
    """Tek hedef koşusunu ölçer ve metrik sözlüğü döner.

    Akış:
      1. Operatörün RViz'den hedef vermesini bekle.
      2. Hedef yürürken pozu örnekle, mesafeyi biriktir.
      3. Hedef sonuçlanınca (succeeded/aborted/canceled) metrikleri yaz.
    """
    def say(msg: str) -> None:
        if on_status:
            on_status(msg)
        store.event("status", message=msg)

    say("RViz'den 2D Nav Goal bekleniyor…")
    goal = tracker.wait_for_new_goal(wait_goal_s)
    if goal is None:
        say("Hedef gelmedi (zaman aşımı).")
        store.event("no_goal", waited_s=wait_goal_s)
        return {
            "goal_status": "no_goal", "goal_received": False,
            "duration_s": None, "planned_path_len_m": None,
            "actual_path_len_m": None, "avg_speed_mps": None,
            "linear_error_m": None, "angular_error_deg": None,
        }

    # Başlangıç durumu
    track = PoseTrack()
    track.update(monitor, store)
    start_ros_s = monitor.now_s()
    plan_at_start = monitor.plan_count
    replan_at_start = monitor.replan_count
    obstacles_at_start = len(monitor.obstacle_events)
    bt_at_start = len(monitor.bt_events)
    goal_pose = tracker.last_goal_pose
    store.event("goal_accepted", uuid=goal.uuid, goal_pose=goal_pose)
    say(f"Hedef kabul edildi. Sonuç bekleniyor (en fazla {goal_timeout_s:.0f} s)…")

    # Hedef yürürken örnekle
    period = 1.0 / max(sample_hz, 0.5)
    deadline = monitor.now_s() + goal_timeout_s
    while goal.status not in ("succeeded", "canceled", "aborted"):
        if monitor.now_s() > deadline:
            store.event("goal_timeout", uuid=goal.uuid)
            say("Hedef zaman aşımına uğradı (benchmark beklemeyi kesti).")
            break
        monitor.spin_for(period)
        track.update(monitor, store)

    end_ros_s = monitor.now_s()
    track.update(monitor, store)

    # ── metrikler ─────────────────────────────────────────────────────────
    planned = monitor.planned_length_m
    if monitor.global_plan is not None and monitor.global_plan.poses:
        planned = path_length([(p.pose.position.x, p.pose.position.y)
                               for p in monitor.global_plan.poses])
    duration = goal.duration_s
    if duration is None:
        duration = end_ros_s - start_ros_s if end_ros_s > start_ros_s else None
    lin_err, ang_err = goal_error(goal_pose, track.last_map)
    bt = bt_replan_stats(monitor.bt_events[bt_at_start:])

    metrics: dict[str, Any] = {
        "goal_received": True,
        "goal_uuid": goal.uuid,
        "goal_status": goal.status,
        "goal_succeeded": goal.status == "succeeded",
        "goal_pose_x": goal_pose["x"] if goal_pose else None,
        "goal_pose_y": goal_pose["y"] if goal_pose else None,
        "goal_pose_yaw_rad": goal_pose["yaw_rad"] if goal_pose else None,
        "goal_frame": goal_pose["frame"] if goal_pose else None,
        "start_ros_s": start_ros_s,
        "end_ros_s": end_ros_s,
        "duration_s": duration,
        "nav2_navigation_time_s": goal.nav_time_s,
        "nav2_recoveries": goal.recoveries,
        "nav2_distance_remaining_m": goal.distance_remaining_m,
        "planned_path_len_m": planned,
        "actual_path_len_m": track.travelled_m if track.samples > 1 else None,
        "avg_speed_mps": avg_speed(
            track.travelled_m if track.samples > 1 else None, duration),
        "linear_error_m": lin_err,
        "angular_error_deg": ang_err,
        "plan_msgs": monitor.plan_count - plan_at_start,
        "replan_count": monitor.replan_count - replan_at_start,
        "first_replan_s": (monitor.first_replan_s - start_ros_s
                           if monitor.first_replan_s
                           and monitor.first_replan_s >= start_ros_s else None),
        "obstacle_events": len(monitor.obstacle_events) - obstacles_at_start,
        "min_scan_range_m": monitor.min_range_m,
        "pose_samples": track.samples,
        "map_jumps_ignored": track.jumps_ignored,
        "start_map_x": track.first_map.x if track.first_map else None,
        "start_map_y": track.first_map.y if track.first_map else None,
        "final_map_x": track.last_map.x if track.last_map else None,
        "final_map_y": track.last_map.y if track.last_map else None,
        "final_map_yaw_rad": track.last_map.yaw if track.last_map else None,
    }
    metrics.update(bt)
    metrics.update(map_metrics(monitor.map_msg))

    for ev in monitor.obstacle_events[obstacles_at_start:]:
        store.event("obstacle", **ev)

    say(f"Sonuç: {goal.status}. "
        f"süre={_f(duration)} s  gidilen={_f(track.travelled_m)} m  "
        f"hata={_f(lin_err)} m / {_f(ang_err)}°")
    return metrics


def _f(v: Any, digits: int = 2) -> str:
    return "N/A" if v is None else f"{v:.{digits}f}"


def needed_topics(kind: str) -> set[str]:
    """Deney türüne göre açılacak abonelikler (gereksiz CPU harcamamak için)."""
    base = {"pose", "wheels", "mode", "plan", "bt"}
    if kind == "obstacle":
        return base | {"scan", "map", "cmd"}
    if kind == "repeatability":
        return base | {"map"}
    return base | {"scan", "map"}
