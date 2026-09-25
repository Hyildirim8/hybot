"""Deneyler arası paylaşılan yardımcılar."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .. import ros_names as N
from ..monitors import BenchmarkMonitor, Pose2D
from ..stats import angle_diff_deg


@dataclass
class PoseTrack:
    """Gidilen mesafeyi biriktirir ve poz örneklerini akıtır.

    ÇERÇEVE SEÇİMİ — önemli: mesafe ODOM çerçevesinden entegre edilir, harita
    çerçevesinden DEĞİL. slam_toolbox `map->odom` düzeltmesini süreksiz
    uygular (bu oturumda 60 s'de 19 sıçrama, en büyüğü 494 cm ölçüldü); harita
    çerçevesinde entegre etmek bu sıçramaları "gidilen yol" sayar ve mesafeyi
    şişirir. Odom çerçevesi 2026-09-24'ten beri saf ölü hesaplamadır, yani
    pürüzsüzdür ve mesafe entegrasyonu için doğru kaynaktır.
    Hedefe olan HATA ise map çerçevesinde ölçülür (hedef map'te verilir).
    """

    travelled_m: float = 0.0
    samples: int = 0
    _last: tuple[float, float] | None = None
    first_odom: Pose2D | None = None
    last_odom: Pose2D | None = None
    first_map: Pose2D | None = None
    last_map: Pose2D | None = None
    max_step_m: float = 0.5   # bundan büyük tek adım sıçrama sayılır, atılır
    jumps_ignored: int = 0

    def update(self, monitor: BenchmarkMonitor, store=None) -> None:
        odom = monitor.odom_pose
        if odom is None:
            return
        if self.first_odom is None:
            self.first_odom = odom
        self.last_odom = odom
        mp = monitor.map_pose()
        if mp is not None:
            if self.first_map is None:
                self.first_map = mp
            self.last_map = mp
        if self._last is not None:
            step = math.hypot(odom.x - self._last[0], odom.y - self._last[1])
            if step > self.max_step_m:
                self.jumps_ignored += 1
            else:
                self.travelled_m += step
        self._last = (odom.x, odom.y)
        self.samples += 1
        if store is not None:
            store.sample("pose", {
                "t_ros_s": round(monitor.now_s(), 4),
                "odom_x": round(odom.x, 4), "odom_y": round(odom.y, 4),
                "odom_yaw_rad": round(odom.yaw, 5),
                "map_x": round(mp.x, 4) if mp else "",
                "map_y": round(mp.y, 4) if mp else "",
                "map_yaw_rad": round(mp.yaw, 5) if mp else "",
                "travelled_m": round(self.travelled_m, 4),
                "wheels_still": int(monitor.wheels_still()),
            })


def goal_error(goal: dict[str, Any] | None,
               final_map_pose: Pose2D | None) -> tuple[float | None, float | None]:
    """(doğrusal hata [m], açısal hata [derece]).

    Ölçülemezse (None, None) — 0 DÖNDÜRÜLMEZ. Hedef map çerçevesinde değilse
    de ölçüm yapılmaz, çünkü karşılaştırma anlamsız olur.
    """
    if goal is None or final_map_pose is None:
        return None, None
    if goal.get("frame") not in (N.FRAME_MAP, "", None):
        return None, None
    lin = math.hypot(final_map_pose.x - goal["x"], final_map_pose.y - goal["y"])
    ang = abs(angle_diff_deg(final_map_pose.yaw, goal["yaw_rad"]))
    return lin, ang


def map_metrics(msg) -> dict[str, Any]:
    """OccupancyGrid'den ölçülebilen harita metrikleri.

    Harita kalitesi TEK BİR UYDURMA PUANA indirgenmez; her metrik ayrı
    raporlanır. -1 bilinmeyen, 0..occupied_threshold boş, üstü dolu kabul
    edilir (ROS nav_msgs/OccupancyGrid sözleşmesi).
    """
    if msg is None:
        return {
            "map_resolution_m": None, "map_width_cells": None,
            "map_height_cells": None, "map_area_m2": None,
            "map_known_area_m2": None, "map_unknown_area_m2": None,
            "map_occupied_cells": None, "map_free_cells": None,
            "map_unknown_cells": None, "map_coverage_ratio": None,
        }
    res = float(msg.info.resolution)
    w, h = int(msg.info.width), int(msg.info.height)
    cell_area = res * res
    occupied = free = unknown = 0
    for v in msg.data:
        if v < 0:
            unknown += 1
        elif v >= 65:      # slam_toolbox occupied_threshold ~0.65
            occupied += 1
        else:
            free += 1
    total_cells = w * h
    known = occupied + free
    return {
        "map_resolution_m": res,
        "map_width_cells": w,
        "map_height_cells": h,
        "map_area_m2": total_cells * cell_area,
        "map_known_area_m2": known * cell_area,
        "map_unknown_area_m2": unknown * cell_area,
        "map_occupied_cells": occupied,
        "map_free_cells": free,
        "map_unknown_cells": unknown,
        "map_coverage_ratio": (known / total_cells) if total_cells else None,
    }


def bt_replan_stats(bt_events: list[dict]) -> dict[str, Any]:
    """BehaviorTreeLog'dan yeniden planlama / recovery sayıları.

    ComputePathToPose her çalıştığında planlama olur; IDLE->RUNNING geçişi
    sayılır. ClearEntireCostmap ve BackUp recovery göstergesidir.
    BT log yoksa None döner (0 değil — ölçülemedi ile hiç olmadı farklı).
    """
    if not bt_events:
        return {"bt_compute_path_runs": None, "bt_clear_costmap_runs": None,
                "bt_backup_runs": None, "bt_first_recovery_s": None}
    compute = clear = backup = 0
    first_recovery = None
    for ev in bt_events:
        name = ev.get("node_name", "")
        started = (ev.get("previous_status") == "IDLE"
                   and ev.get("current_status") == "RUNNING")
        if not started:
            continue
        if name == "ComputePathToPose":
            compute += 1
        elif "ClearEntireCostmap" in name or "ClearCostmap" in name:
            clear += 1
            first_recovery = first_recovery or ev.get("t_s")
        elif name == "BackUp":
            backup += 1
            first_recovery = first_recovery or ev.get("t_s")
    return {"bt_compute_path_runs": compute, "bt_clear_costmap_runs": clear,
            "bt_backup_runs": backup, "bt_first_recovery_s": first_recovery}


def avg_speed(distance_m: float | None, duration_s: float | None) -> float | None:
    if distance_m is None or duration_s is None or duration_s <= 0:
        return None
    return distance_m / duration_s
