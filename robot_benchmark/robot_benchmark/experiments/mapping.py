"""Haritalama testi.

Harita kalitesi TEK BİR UYDURMA PUANA indirgenmez. OccupancyGrid'den
ölçülebilen her metrik ayrı raporlanır (çözünürlük, genişlik, yükseklik,
toplam/bilinen/bilinmeyen alan, dolu/boş/bilinmeyen hücre, kapsama oranı).

Aynı ortam için birden fazla koşu `environment` alanıyla eşleşir; rapor
tarafı bunları karşılaştırır.
"""

from __future__ import annotations

from typing import Any, Callable

from ..monitors import BenchmarkMonitor
from .common import map_metrics


def run_mapping(monitor: BenchmarkMonitor, store, duration_s: float = 120.0,
                sample_period_s: float = 5.0,
                on_status: Callable[[str], None] | None = None,
                stop_when: Callable[[], bool] | None = None) -> dict[str, Any]:
    """Haritalama süresince harita büyümesini örnekler."""
    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", message=m)

    say("Harita bekleniyor…")
    if not monitor.wait_until(lambda: monitor.map_msg is not None, 30.0):
        say("/map gelmedi — slam_toolbox çalışıyor mu?")
        out = {"mapping_started_ros_s": None, "mapping_ended_ros_s": None,
               "mapping_duration_s": None, "map_updates": 0}
        out.update(map_metrics(None))
        return out

    start = monitor.now_s()
    store.event("mapping_start", t_ros_s=start)
    first = map_metrics(monitor.map_msg)
    say(f"Haritalama başladı. {duration_s:.0f} s boyunca ölçülecek "
        f"(Ctrl+C ile erken bitirilebilir, veri korunur).")

    updates_at_start = monitor.map_count
    end_t = start + duration_s
    while monitor.now_s() < end_t:
        if stop_when is not None and stop_when():
            say("Operatör haritalamayı bitirdi.")
            break
        monitor.spin_for(sample_period_s)
        m = map_metrics(monitor.map_msg)
        store.sample("map_growth", {
            "t_rel_s": round(monitor.now_s() - start, 2),
            "known_area_m2": _r(m["map_known_area_m2"]),
            "unknown_area_m2": _r(m["map_unknown_area_m2"]),
            "occupied_cells": m["map_occupied_cells"],
            "free_cells": m["map_free_cells"],
            "coverage_ratio": _r(m["map_coverage_ratio"], 5),
            "width_cells": m["map_width_cells"],
            "height_cells": m["map_height_cells"],
        })
        say(f"  kapsama={_p(m['map_coverage_ratio'])}  "
            f"bilinen={_r(m['map_known_area_m2'])} m²  "
            f"dolu={m['map_occupied_cells']} hücre")

    end = monitor.now_s()
    store.event("mapping_end", t_ros_s=end)
    final = map_metrics(monitor.map_msg)
    out: dict[str, Any] = {
        "mapping_started_ros_s": start,
        "mapping_ended_ros_s": end,
        "mapping_duration_s": end - start,
        "map_updates": monitor.map_count - updates_at_start,
        "map_known_area_start_m2": first["map_known_area_m2"],
        "map_known_area_gain_m2": (
            None if final["map_known_area_m2"] is None
            or first["map_known_area_m2"] is None
            else final["map_known_area_m2"] - first["map_known_area_m2"]),
    }
    out.update(final)
    say(f"Haritalama bitti. kapsama={_p(final['map_coverage_ratio'])} "
        f"bilinen alan={_r(final['map_known_area_m2'])} m²")
    return out


def _r(v, digits: int = 3):
    return "" if v is None else round(v, digits)


def _p(v) -> str:
    return "N/A" if v is None else f"%{v*100:.1f}"
