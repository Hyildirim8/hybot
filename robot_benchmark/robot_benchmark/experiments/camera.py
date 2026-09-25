"""Kamera akış testi.

Bu projede kamera YALNIZCA sıkıştırılmış yayınlanır:
    /camera_csi/image_raw/compressed   sensor_msgs/CompressedImage

Uçtan uca gecikme, mesaj header damgası GÜVENİLİR değilse ÖLÇÜLMEZ ve
"N/A" raporlanır. Güvenilirlik ölçütü: damganın sıfır olmaması ve alım anına
göre makul aralıkta (-1 s .. +5 s) kalması. rpicam_node damgayı yayın anında
koyuyorsa bu "kamera->ROS" gecikmesini ölçmez; o yüzden iddia edilmez.
"""

from __future__ import annotations

import statistics
from typing import Any, Callable

from ..monitors import BenchmarkMonitor


def run_camera(monitor: BenchmarkMonitor, store, duration_s: float = 30.0,
               gap_threshold_s: float = 0.5,
               on_status: Callable[[str], None] | None = None) -> dict[str, Any]:
    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", message=m)

    say(f"Kamera akışı {duration_s:.0f} s dinleniyor…")
    if not monitor.wait_until(lambda: monitor.cam_count > 0, 15.0):
        say("Kamera karesi gelmedi — csi_camera servisi çalışıyor mu?")
        return {
            "camera_topic": monitor.__class__ and "/camera_csi/image_raw/compressed",
            "camera_frames": 0, "camera_fps_mean": None, "camera_fps_nominal": None,
            "camera_resolution": None, "camera_gap_count": None,
            "camera_gap_total_s": None, "camera_interval_mean_s": None,
            "camera_interval_median_s": None, "camera_interval_std_s": None,
            "camera_interval_min_s": None, "camera_interval_max_s": None,
            "camera_latency_s": None,
            "camera_latency_note": "kare alınamadı",
            "camera_bytes_mean": None,
        }

    base_count = monitor.cam_count
    base_idx = len(monitor.cam_stamps_s)
    t0 = monitor.now_s()
    monitor.spin_for(duration_s)
    t1 = monitor.now_s()

    arrivals = monitor.cam_stamps_s[base_idx:]
    headers = monitor.cam_header_stamps_s[base_idx:]
    sizes = monitor.cam_sizes[base_idx:]
    frames = monitor.cam_count - base_count
    elapsed = t1 - t0

    intervals = [b - a for a, b in zip(arrivals, arrivals[1:])]
    gaps = [d for d in intervals if d > gap_threshold_s]

    # Gecikme yalnızca damga güvenilirse
    latency_vals = []
    for arr, hdr in zip(arrivals, headers):
        if hdr <= 0.0:
            continue
        d = arr - hdr
        if -1.0 <= d <= 5.0:
            latency_vals.append(d)
    if len(latency_vals) >= max(3, int(0.5 * len(arrivals))):
        latency = statistics.median(latency_vals)
        note = ("header damgası kullanıldı; bu ROS yayın->alım gecikmesidir, "
                "sensörden-ekrana uçtan uca gecikme DEĞİLDİR")
    else:
        latency = None
        note = "header damgası güvenilir değil (sıfır veya tutarsız) — ölçülemedi"

    for a, h, s in zip(arrivals, headers, sizes):
        store.sample("camera_frames", {
            "arrival_ros_s": round(a, 5),
            "header_ros_s": round(h, 5),
            "bytes": s,
        })

    def st(fn, vals):
        try:
            return fn(vals)
        except statistics.StatisticsError:
            return None

    out = {
        "camera_topic": "/camera_csi/image_raw/compressed",
        "camera_observe_duration_s": elapsed,
        "camera_frames": frames,
        "camera_fps_mean": (frames / elapsed) if elapsed > 0 else None,
        "camera_resolution": monitor.cam_resolution,
        "camera_gap_count": len(gaps),
        "camera_gap_total_s": sum(gaps) if gaps else 0.0,
        "camera_gap_max_s": max(gaps) if gaps else None,
        "camera_interval_mean_s": st(statistics.fmean, intervals) if intervals else None,
        "camera_interval_median_s": st(statistics.median, intervals) if intervals else None,
        "camera_interval_std_s": (st(statistics.stdev, intervals)
                                 if len(intervals) >= 2 else None),
        "camera_interval_min_s": min(intervals) if intervals else None,
        "camera_interval_max_s": max(intervals) if intervals else None,
        "camera_latency_s": latency,
        "camera_latency_note": note,
        "camera_bytes_mean": st(statistics.fmean, sizes) if sizes else None,
    }
    say(f"Kamera: {frames} kare, {out['camera_fps_mean']:.2f} FPS, "
        f"çözünürlük={out['camera_resolution'] or 'N/A'}, "
        f"kesinti={len(gaps)} adet")
    return out
