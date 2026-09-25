"""Tüm koşuları toplayıp özet CSV/JSON üretir.

İki kural korunur:
  * BAŞARISIZ KOŞULAR ÇIKARILMAZ — ayrı durum olarak sayılır ve tabloya girer.
  * EKSİK METRİK 0 İLE DOLDURULMAZ — None kalır, çıktıda boş/N/A görünür.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

from ..stats import Summary, summarize, success_rate
from ..store import load_runs, results_dir, _atomic_json


# Toplu tabloda özetlenecek sayısal metrikler (deney türünden bağımsız).
NUMERIC_METRICS = (
    "duration_s", "planned_path_len_m", "actual_path_len_m", "avg_speed_mps",
    "linear_error_m", "angular_error_deg", "replan_count", "plan_msgs",
    "first_replan_s", "obstacle_events", "nav2_recoveries",
    "bt_compute_path_runs", "bt_clear_costmap_runs", "bt_backup_runs",
    "map_coverage_ratio", "map_known_area_m2", "map_occupied_cells",
    "mapping_duration_s", "camera_fps_mean", "camera_interval_mean_s",
    "camera_gap_total_s", "camera_frames", "runtime_duration_min",
    "battery_drop", "battery_drop_per_hour",
    "mode_switch_success_rate", "linear_distance_m", "angular_change_deg",
    "mode_switch_latency_mean_s", "mode_switch_latency_median_s",
    "mode_switch_latency_max_s",
)

UNITS = {
    "duration_s": "s", "planned_path_len_m": "m", "actual_path_len_m": "m",
    "avg_speed_mps": "m/s", "linear_error_m": "m", "angular_error_deg": "°",
    "first_replan_s": "s", "map_coverage_ratio": "oran",
    "map_known_area_m2": "m²", "mapping_duration_s": "s",
    "camera_fps_mean": "FPS", "camera_interval_mean_s": "s",
    "camera_gap_total_s": "s", "runtime_duration_min": "dk",
    "battery_drop_per_hour": "birim/saat", "linear_distance_m": "m",
    "mode_switch_latency_mean_s": "s", "mode_switch_latency_median_s": "s",
    "mode_switch_latency_max_s": "s",
    "angular_change_deg": "°",
}

LABELS_TR = {
    "duration_s": "Hedefe ulaşma süresi",
    "planned_path_len_m": "Planlanan yol uzunluğu",
    "actual_path_len_m": "Gerçek gidilen mesafe",
    "avg_speed_mps": "Ortalama hız",
    "linear_error_m": "Doğrusal konum hatası",
    "angular_error_deg": "Açısal konum hatası",
    "replan_count": "Yeniden planlama sayısı",
    "plan_msgs": "Alınan plan mesajı",
    "first_replan_s": "İlk yeniden planlama süresi",
    "obstacle_events": "Engel algılama olayı",
    "nav2_recoveries": "Nav2 recovery sayısı",
    "bt_compute_path_runs": "BT plan hesaplama",
    "bt_clear_costmap_runs": "BT costmap temizleme",
    "bt_backup_runs": "BT geri çekilme",
    "map_coverage_ratio": "Harita kapsama oranı",
    "map_known_area_m2": "Bilinen harita alanı",
    "map_occupied_cells": "Dolu hücre sayısı",
    "mapping_duration_s": "Haritalama süresi",
    "camera_fps_mean": "Kamera FPS",
    "camera_interval_mean_s": "Kareler arası süre",
    "camera_gap_total_s": "Toplam kesinti",
    "camera_frames": "Alınan kare sayısı",
    "runtime_duration_min": "Çalışma süresi",
    "battery_drop": "Batarya düşüşü",
    "battery_drop_per_hour": "Saatlik batarya tüketimi",
    "mode_switch_success_rate": "Mod geçiş başarı oranı",
    "mode_switch_latency_mean_s": "Mod geçiş gecikmesi (ort)",
    "mode_switch_latency_median_s": "Mod geçiş gecikmesi (medyan)",
    "mode_switch_latency_max_s": "Mod geçiş gecikmesi (maks)",
    "linear_distance_m": "Doğrusal mesafe",
    "angular_change_deg": "Açısal değişim",
}


def collect(base: Path | None = None) -> list[dict[str, Any]]:
    return load_runs(base or results_dir())


def group_by(runs: list[dict], *keys: str) -> dict[tuple, list[dict]]:
    out: dict[tuple, list[dict]] = defaultdict(list)
    for r in runs:
        out[tuple(r["meta"].get(k, "") for k in keys)].append(r)
    return dict(out)


def metric_values(runs: list[dict], name: str) -> list[float | None]:
    vals: list[float | None] = []
    for r in runs:
        v = r["metrics"].get(name)
        if isinstance(v, bool):
            v = float(v)
        if isinstance(v, (int, float)):
            vals.append(float(v))
        else:
            vals.append(None)
    return vals


def summarize_group(runs: list[dict]) -> dict[str, Summary]:
    return {m: summarize(metric_values(runs, m)) for m in NUMERIC_METRICS}


# Konumlandirma DOGRULUGU yalnizca hedefe ULASILAN kosularda anlamlidir:
# iptal edilen bir hedefte "hata" doğruluk degil, robotun nereye kadar
# gidebildigidir (olcum: bir aborted kosuda 11.8 m). Bu yuzden bu metrikler
# ayrica succeeded-only olarak da ozetlenir. Basarisiz kosular SILINMEZ;
# basari oraninda ve tum-kosu ozetinde aynen sayilir.
ACCURACY_METRICS = ("linear_error_m", "angular_error_deg")


def succeeded_runs(runs: list[dict]) -> list[dict]:
    return [r for r in runs if r["metrics"].get("goal_status") == "succeeded"]


def summarize_accuracy(runs: list[dict]) -> dict[str, Summary]:
    ok = succeeded_runs(runs)
    return {m: summarize(metric_values(ok, m)) for m in ACCURACY_METRICS}


def status_breakdown(runs: list[dict]) -> tuple[float | None, dict[str, int]]:
    """Hedef durumlarına göre başarı oranı. Hedef almayan koşular dışarıda."""
    statuses = [r["metrics"].get("goal_status") for r in runs
                if r["metrics"].get("goal_status")]
    return success_rate([s for s in statuses if s])


def build_summary(base: Path | None = None) -> dict[str, Any]:
    """Tüm koşuların toplu özeti."""
    runs = collect(base)
    by_kind = group_by(runs, "kind")
    payload: dict[str, Any] = {
        "run_count": len(runs),
        "partial_run_count": sum(1 for r in runs if r["meta"].get("partial")),
        "mock_run_count": sum(1 for r in runs if r["meta"].get("mock")),
        "kinds": {},
        "by_goal": {},
    }
    for (kind,), group in sorted(by_kind.items()):
        rate, counts = status_breakdown(group)
        payload["kinds"][kind] = {
            "run_count": len(group),
            "goal_success_rate": rate,
            "goal_status_counts": counts,
            "metrics": {m: s.as_dict() for m, s in summarize_group(group).items()
                        if s.n > 0},
            "accuracy_succeeded_only": {
                m: s.as_dict() for m, s in summarize_accuracy(group).items()
                if s.n > 0},
        }
    # Hedef bazlı (tekrarlanabilirlik için asıl tablo)
    for (kind, env, goal), group in sorted(
            group_by(runs, "kind", "environment", "goal_name").items()):
        if not goal:
            continue
        rate, counts = status_breakdown(group)
        payload["by_goal"][f"{kind}|{env}|{goal}"] = {
            "kind": kind, "environment": env, "goal_name": goal,
            "run_count": len(group),
            "goal_success_rate": rate,
            "goal_status_counts": counts,
            "metrics": {m: s.as_dict() for m, s in summarize_group(group).items()
                        if s.n > 0},
            "accuracy_succeeded_only": {
                m: s.as_dict() for m, s in summarize_accuracy(group).items()
                if s.n > 0},
        }
    return payload


def write_summary(out_dir: Path, base: Path | None = None) -> dict[str, Path]:
    """summary.json + summary_runs.csv + summary_metrics.csv yazar."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    runs = collect(base)
    payload = build_summary(base)

    paths: dict[str, Path] = {}
    p = out_dir / "summary.json"
    _atomic_json(p, payload)
    paths["summary_json"] = p

    # Her koşu bir satır (ham, hiçbir koşu atılmaz)
    meta_keys = ["experiment_id", "kind", "environment", "goal_name",
                 "repetition", "started_at", "finished_at", "duration_s",
                 "partial", "mock", "operator_note"]
    metric_keys: list[str] = []
    for r in runs:
        for k in r["metrics"]:
            if k not in metric_keys:
                metric_keys.append(k)
    p = out_dir / "summary_runs.csv"
    with open(p, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(meta_keys + metric_keys)
        for r in runs:
            row = [r["meta"].get(k, "") for k in meta_keys]
            for k in metric_keys:
                v = r["metrics"].get(k)
                if v is None:
                    row.append("")            # 0 DEĞİL
                elif isinstance(v, (dict, list, tuple)):
                    row.append(json.dumps(v, ensure_ascii=False))
                else:
                    row.append(v)
            w.writerow(row)
    paths["summary_runs_csv"] = p

    # Metrik özet tablosu
    p = out_dir / "summary_metrics.csv"
    with open(p, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["deney", "ortam", "hedef", "metrik", "birim", "n",
                    "ortalama", "medyan", "std", "min", "max",
                    "ci95_alt", "ci95_ust"])
        for blk in payload["by_goal"].values():
            for m, s in blk.get("accuracy_succeeded_only", {}).items():
                w.writerow([blk["kind"], blk["environment"], blk["goal_name"],
                            LABELS_TR.get(m, m) + " (yalnizca basarili)",
                            UNITS.get(m, ""), s["n"],
                            _c(s["mean"]), _c(s["median"]), _c(s["std"]),
                            _c(s["minimum"]), _c(s["maximum"]),
                            _c(s["ci95_low"]), _c(s["ci95_high"])])
            for m, s in blk["metrics"].items():
                w.writerow([blk["kind"], blk["environment"], blk["goal_name"],
                            LABELS_TR.get(m, m), UNITS.get(m, ""), s["n"],
                            _c(s["mean"]), _c(s["median"]), _c(s["std"]),
                            _c(s["minimum"]), _c(s["maximum"]),
                            _c(s["ci95_low"]), _c(s["ci95_high"])])
        for kind, blk in payload["kinds"].items():
            for m, s in blk["metrics"].items():
                w.writerow([kind, "(tümü)", "(tümü)",
                            LABELS_TR.get(m, m), UNITS.get(m, ""), s["n"],
                            _c(s["mean"]), _c(s["median"]), _c(s["std"]),
                            _c(s["minimum"]), _c(s["maximum"]),
                            _c(s["ci95_low"]), _c(s["ci95_high"])])
    paths["summary_metrics_csv"] = p
    return paths


def _c(v: Any) -> Any:
    """CSV hücresi: None -> boş (0 değil)."""
    if v is None:
        return ""
    if isinstance(v, float):
        return round(v, 6)
    return v
