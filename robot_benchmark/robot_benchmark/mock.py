"""Sahte (mock) veri üretimi ve çevrimdışı doğrulama.

İki amaç:
  1. Gerçek robot bağlı DEĞİLKEN rapor üretim zincirini uçtan uca doğrulamak.
  2. Düğümlerin ROS olmadan da başlatılabildiğini göstermek.

Üretilen koşular `meta.mock = True` ile işaretlenir; rapor bunları ayrı sayar
ve "gerçek robot ölçümü değildir" diye belirtir. Böylece sahte veri akademik
sonuçlara sessizce karışamaz.
"""

from __future__ import annotations

import math
import random
from datetime import datetime, timedelta
from pathlib import Path


from .store import RunMeta, RunStore, new_experiment_id


def _meta(kind: str, env: str, goal: str, rep: int, when: datetime) -> RunMeta:
    return RunMeta(
        experiment_id=new_experiment_id(kind),
        kind=kind,
        started_at=when.isoformat(timespec="seconds"),
        environment=env,
        goal_name=goal,
        repetition=rep,
        operator_note="mock veri — gerçek ölçüm değil",
        mock=True,
    )


def generate(base: Path, seed: int = 7, goals: int = 3, repeats: int = 4
             ) -> list[Path]:
    """Sahte koşular üretir: navigation, obstacle, repeatability, mapping,
    camera, battery, mode-switch, mecanum."""
    rng = random.Random(seed)
    base = Path(base)
    base.mkdir(parents=True, exist_ok=True)
    made: list[Path] = []
    t = datetime(2026, 9, 25, 10, 0, 0)
    env = "3. kat koridor"

    goal_defs = [(f"H{i}", 3.0 + 2.4 * i, 1.2 * i) for i in range(1, goals + 1)]

    # ── hedefe ulaşma + tekrarlanabilirlik ────────────────────────────────
    for kind in ("navigation", "repeatability"):
        for gname, gx, gy in goal_defs:
            for rep in range(1, repeats + 1):
                t += timedelta(minutes=3)
                # gerçekçi dağılım: çoğu başarılı, bazıları abort/cancel
                roll = rng.random()
                status = ("succeeded" if roll < 0.75
                          else ("aborted" if roll < 0.9 else "canceled"))
                planned = math.hypot(gx, gy) * rng.uniform(1.02, 1.15)
                actual = planned * rng.uniform(1.03, 1.30)
                dur = actual / rng.uniform(0.18, 0.34)
                ok = status == "succeeded"
                st = RunStore(_meta(kind, env, gname, rep, t), base=base)
                st.event("goal_accepted", uuid=f"mock{rep}")
                for i in range(10):
                    st.sample("pose", {"t_ros_s": i * 0.5, "odom_x": i * 0.1,
                                       "odom_y": 0.0, "odom_yaw_rad": 0.0,
                                       "map_x": i * 0.1, "map_y": 0.0,
                                       "map_yaw_rad": 0.0,
                                       "travelled_m": i * 0.1,
                                       "wheels_still": 0})
                st.set_metrics(
                    goal_received=True, goal_status=status,
                    goal_succeeded=ok,
                    goal_pose_x=gx, goal_pose_y=gy, goal_pose_yaw_rad=0.0,
                    goal_frame="map",
                    duration_s=dur if ok else dur * rng.uniform(0.4, 0.8),
                    planned_path_len_m=planned,
                    actual_path_len_m=actual if ok else actual * 0.6,
                    avg_speed_mps=(actual / dur) if ok else None,
                    linear_error_m=(rng.gauss(0.11, 0.05) if ok else None),
                    angular_error_deg=(abs(rng.gauss(6.0, 3.5)) if ok else None),
                    replan_count=rng.randint(0, 5),
                    plan_msgs=rng.randint(3, 25),
                    first_replan_s=(rng.uniform(1.0, 9.0) if rng.random() < 0.7
                                    else None),
                    obstacle_events=rng.randint(0, 4),
                    nav2_recoveries=rng.randint(0, 3),
                    bt_compute_path_runs=rng.randint(1, 8),
                    bt_clear_costmap_runs=rng.randint(0, 3),
                    bt_backup_runs=rng.randint(0, 2),
                    operator_collision_marked=False,
                    min_scan_range_m=rng.uniform(0.25, 1.8),
                )
                made.append(st.close())

    # ── engelden kaçınma (operatör çarpışma işaretiyle) ───────────────────
    for gname, gx, gy in goal_defs[:2]:
        for rep in range(1, 3):
            t += timedelta(minutes=4)
            hit = rng.random() < 0.35
            st = RunStore(_meta("obstacle", env, gname, rep, t), base=base)
            st.event("obstacle", t_s=2.0, edge="detected", min_range_m=0.31)
            st.set_metrics(
                goal_received=True,
                goal_status="succeeded" if not hit else "aborted",
                goal_succeeded=not hit,
                duration_s=rng.uniform(20, 70),
                planned_path_len_m=math.hypot(gx, gy) * 1.1,
                actual_path_len_m=math.hypot(gx, gy) * 1.35,
                linear_error_m=(rng.gauss(0.15, 0.06) if not hit else None),
                angular_error_deg=(abs(rng.gauss(8.0, 4.0)) if not hit else None),
                replan_count=rng.randint(2, 9),
                first_replan_s=rng.uniform(0.8, 5.0),
                obstacle_events=rng.randint(1, 7),
                nav2_recoveries=rng.randint(0, 4),
                bt_clear_costmap_runs=rng.randint(1, 6),
                bt_backup_runs=rng.randint(0, 3),
                operator_collision_marked=hit,
                reached_after_obstacle=not hit,
            )
            made.append(st.close())

    # ── haritalama (aynı ortam için 2 koşu) ───────────────────────────────
    for rep in range(1, 3):
        t += timedelta(minutes=10)
        cov = rng.uniform(0.32, 0.61)
        w = h = 320
        total = w * h
        known = int(total * cov)
        occ = int(known * rng.uniform(0.10, 0.2))
        st = RunStore(_meta("mapping", env, f"harita-{rep}", rep, t), base=base)
        for i in range(8):
            st.sample("map_growth", {"t_rel_s": i * 15.0,
                                     "known_area_m2": round(known * 0.0025 * (i + 1) / 8, 3),
                                     "unknown_area_m2": 0.0,
                                     "occupied_cells": occ,
                                     "free_cells": known - occ,
                                     "coverage_ratio": round(cov * (i + 1) / 8, 5),
                                     "width_cells": w, "height_cells": h})
        st.set_metrics(
            mapping_duration_s=rng.uniform(180, 420),
            map_updates=rng.randint(40, 160),
            map_resolution_m=0.05, map_width_cells=w, map_height_cells=h,
            map_area_m2=total * 0.0025,
            map_known_area_m2=known * 0.0025,
            map_unknown_area_m2=(total - known) * 0.0025,
            map_occupied_cells=occ, map_free_cells=known - occ,
            map_unknown_cells=total - known, map_coverage_ratio=cov,
        )
        made.append(st.close())

    # ── kamera ────────────────────────────────────────────────────────────
    for rep in range(1, 3):
        t += timedelta(minutes=2)
        fps = rng.uniform(7.5, 14.0)
        iv = 1.0 / fps
        st = RunStore(_meta("camera", env, f"kamera-{rep}", rep, t), base=base)
        for i in range(40):
            st.sample("camera_frames", {"arrival_ros_s": round(i * iv, 5),
                                       "header_ros_s": round(i * iv - 0.02, 5),
                                       "bytes": rng.randint(9000, 26000)})
        st.set_metrics(
            camera_topic="/camera_csi/image_raw/compressed",
            camera_observe_duration_s=30.0,
            camera_frames=int(fps * 30),
            camera_fps_mean=fps,
            camera_resolution="640x480",
            camera_gap_count=rng.randint(0, 3),
            camera_gap_total_s=rng.uniform(0.0, 1.6),
            camera_interval_mean_s=iv,
            camera_interval_median_s=iv * 0.98,
            camera_interval_std_s=iv * rng.uniform(0.08, 0.3),
            camera_interval_min_s=iv * 0.6, camera_interval_max_s=iv * 2.4,
            camera_latency_s=None,
            camera_latency_note="header damgası güvenilir değil — ölçülemedi",
            camera_bytes_mean=17000.0,
        )
        made.append(st.close())

    # ── batarya / çalışma süresi ──────────────────────────────────────────
    t += timedelta(minutes=30)
    st = RunStore(_meta("battery", env, "dayanim-1", 1, t), base=base)
    st.set_metrics(
        battery_source="operator_input", battery_unit="V",
        battery_start=12.6, battery_end=11.4, battery_drop=1.2,
        battery_drop_per_hour=0.72,
        runtime_duration_s=6000.0, runtime_duration_min=100.0,
        battery_remaining_runtime_s=None,
        battery_remaining_note="kapasite bilinmiyor; tahmin edilmedi",
    )
    made.append(st.close())

    # ── mod geçişi ────────────────────────────────────────────────────────
    t += timedelta(minutes=5)
    lat = [rng.uniform(0.12, 0.6) for _ in range(9)]
    st = RunStore(_meta("mode-switch", env, "mod-10tekrar", 1, t), base=base)
    for i, l in enumerate(lat, start=1):
        st.sample("mode_switch", {"index": i, "from_mode": i % 2,
                                  "to_mode": (i + 1) % 2, "confirmed": 1,
                                  "latency_s": round(l, 4)})
    st.set_metrics(
        mode_initial=False, mode_switch_attempts=10, mode_switch_success=9,
        mode_switch_failed=1, mode_switch_success_rate=0.9,
        mode_switch_latencies_s=[round(x, 4) for x in lat],
        latency_reference="operator_keypress",
    )
    made.append(st.close())

    # ── mecanum ───────────────────────────────────────────────────────────
    t += timedelta(minutes=6)
    st = RunStore(_meta("mecanum", env, "6yon", 1, t), base=base)
    dirs = {"ileri": (0.19, 0.01, 0.6), "geri": (-0.12, 0.02, -0.4),
            "sol": (0.04, 0.09, 1.1), "sag": (0.02, -0.08, -0.9),
            "ccw": (0.01, 0.0, 28.0), "cw": (0.0, 0.01, -26.0)}
    for k, (fwd, left, yaw) in dirs.items():
        st.sample("mecanum", {"direction": k, "forward_m": fwd,
                              "lateral_m": left, "yaw_deg": yaw})
    st.set_metrics(
        mecanum_directions=list(dirs),
        linear_distance_m=math.hypot(0.19, 0.01),
        angular_change_deg=28.0,
        mecanum_all_correct=True,
        motion_mode="observe_only",
    )
    made.append(st.close())
    return made
