"""PNG grafikleri (akademik rapor için).

matplotlib kurulu değilse grafikler ATLANIR ve bu durum açıkça bildirilir;
CSV/JSON/Markdown/LaTeX çıktıları yine üretilir. seaborn varsa stil için
kullanılır, yok ise matplotlib'in kendi stiliyle benzer görünüm kurulur
(seaborn zorunlu bağımlılık yapılmadı: Pi'de ağır).

Kurallar:
  * Tüm başlık/eksen/gösterge Türkçe, birim parantez içinde.
  * Örnek sayısı (n) grafikte görünür — akademik raporda şart.
  * Eksik ölçüm çizilmez; 0 olarak gösterilmez.
  * dpi=200 (yüksek çözünürlük).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .aggregate import (collect, group_by, metric_values, status_breakdown)

DPI = 200
FIGSIZE = (9.0, 5.2)

try:
    import matplotlib
    matplotlib.use("Agg")           # başsız ortam (Pi, container)
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:                   # pragma: no cover
    plt = None
    HAVE_MPL = False

try:
    import seaborn as sns
    HAVE_SNS = True
except Exception:
    sns = None
    HAVE_SNS = False


def _style() -> None:
    if not HAVE_MPL:
        return
    if HAVE_SNS:
        sns.set_theme(style="whitegrid", context="paper")
    else:
        # seaborn yoksa benzer görünüm
        plt.rcParams.update({
            "axes.grid": True, "grid.alpha": 0.3, "grid.linestyle": "--",
            "axes.spines.top": False, "axes.spines.right": False,
            "figure.autolayout": True,
        })
    plt.rcParams.update({
        "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold",
        "axes.labelsize": 11, "savefig.dpi": DPI, "figure.dpi": DPI,
    })


def _finish(fig, ax, out: Path, title: str, xlabel: str, ylabel: str,
            note: str | None = None) -> Path:
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if note:
        fig.text(0.99, 0.01, note, ha="right", va="bottom", fontsize=8,
                 alpha=0.75)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return out


def _labelled(values: list[float | None], labels: list[str]
              ) -> tuple[list[float], list[str]]:
    """None olanları ATAR (0 yapmaz) ve etiketleri hizalar."""
    xs, ls = [], []
    for v, l in zip(values, labels):
        if v is not None:
            xs.append(float(v))
            ls.append(l)
    return xs, ls


def generate_all(out_dir: Path, base: Path | None = None) -> dict[str, Any]:
    """Tüm grafikleri üretir. Dönen sözlükte üretilenler ve atlananlar var."""
    out_dir = Path(out_dir)
    result: dict[str, Any] = {"created": [], "skipped": [], "matplotlib": HAVE_MPL,
                              "seaborn": HAVE_SNS}
    if not HAVE_MPL:
        result["skipped"].append(
            "TÜM GRAFİKLER: matplotlib kurulu değil. Kurulum: "
            "pip3 install --user matplotlib  (veya requirements.txt)")
        return result
    _style()
    runs = collect(base)
    if not runs:
        result["skipped"].append("TÜM GRAFİKLER: hiç koşu yok")
        return result

    for fn in (_plot_success_rate, _plot_goal_duration, _plot_planned_vs_actual,
               _plot_error_box, _plot_replan, _plot_map_coverage, _plot_camera):
        try:
            p = fn(runs, out_dir)
            if p is None:
                result["skipped"].append(f"{fn.__name__}: yeterli ölçüm yok")
            else:
                result["created"].append(str(p))
        except Exception as exc:   # bir grafik patlarsa diğerleri üretilsin
            result["skipped"].append(f"{fn.__name__}: hata {exc}")
    return result


# ── 1. başarı oranı ───────────────────────────────────────────────────────────
def _plot_success_rate(runs, out_dir: Path) -> Path | None:
    groups = group_by(runs, "kind")
    names, rates, ns = [], [], []
    for (kind,), g in sorted(groups.items()):
        rate, counts = status_breakdown(g)
        if rate is None:
            continue
        names.append(kind)
        rates.append(rate * 100.0)
        ns.append(sum(counts.values()))
    if not names:
        return None
    fig, ax = plt.subplots(figsize=FIGSIZE)
    bars = ax.bar(names, rates, color="#3b7dd8", edgecolor="black", linewidth=0.6)
    for b, r, n in zip(bars, rates, ns):
        ax.text(b.get_x() + b.get_width() / 2, r + 1.5,
                f"%{r:.1f}\n(n={n})", ha="center", va="bottom", fontsize=9)
    ax.set_ylim(0, 108)
    return _finish(fig, ax, out_dir / "01_basari_orani.png",
                   "Hedefe Ulaşma Başarı Oranı (deney türüne göre)",
                   "Deney türü", "Başarı oranı (%)",
                   "Başarısız koşular çıkarılmadı; oran tüm hedefli koşular üzerinden.")


# ── 2. hedefe ulaşma süresi ───────────────────────────────────────────────────
def _plot_goal_duration(runs, out_dir: Path) -> Path | None:
    groups = group_by(runs, "goal_name")
    data, labels = [], []
    for (goal,), g in sorted(groups.items()):
        if not goal:
            continue
        vals = [v for v in metric_values(g, "duration_s") if v is not None]
        if vals:
            data.append(vals)
            labels.append(f"{goal}\n(n={len(vals)})")
    if not data:
        return None
    fig, ax = plt.subplots(figsize=FIGSIZE)
    bp = ax.boxplot(data, labels=labels, patch_artist=True, showmeans=True)
    for patch in bp["boxes"]:
        patch.set_facecolor("#8fc7ff")
    for i, vals in enumerate(data, start=1):
        ax.scatter([i] * len(vals), vals, s=18, color="#14477d", zorder=3, alpha=0.8)
    return _finish(fig, ax, out_dir / "02_hedefe_ulasma_suresi.png",
                   "Hedefe Ulaşma Süresi (hedef bazlı)",
                   "Hedef", "Süre (s)",
                   "Kutu: çeyrekler, üçgen: ortalama, noktalar: tek tek koşular.")


# ── 3. planlanan vs gerçek mesafe ─────────────────────────────────────────────
def _plot_planned_vs_actual(runs, out_dir: Path) -> Path | None:
    pairs = []
    for r in runs:
        p = r["metrics"].get("planned_path_len_m")
        a = r["metrics"].get("actual_path_len_m")
        if isinstance(p, (int, float)) and isinstance(a, (int, float)):
            pairs.append((r["meta"].get("goal_name") or r["meta"]["experiment_id"][:8],
                          float(p), float(a)))
    if not pairs:
        return None
    labels = [p[0] for p in pairs]
    planned = [p[1] for p in pairs]
    actual = [p[2] for p in pairs]
    idx = range(len(pairs))
    width = 0.38
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.bar([i - width / 2 for i in idx], planned, width,
           label="Planlanan yol", color="#3b7dd8", edgecolor="black", linewidth=0.5)
    ax.bar([i + width / 2 for i in idx], actual, width,
           label="Gerçek gidilen", color="#e0902b", edgecolor="black", linewidth=0.5)
    ax.set_xticks(list(idx))
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.legend()
    return _finish(fig, ax, out_dir / "03_planlanan_vs_gercek_mesafe.png",
                   f"Planlanan ve Gerçek Gidilen Mesafe (n={len(pairs)} koşu)",
                   "Koşu / hedef", "Mesafe (m)",
                   "Gerçek mesafe odom çerçevesinden entegre edildi "
                   "(map sıçramaları hariç tutuldu).")


# ── 4. konum hatası kutu grafiği ──────────────────────────────────────────────
def _plot_error_box(runs, out_dir: Path) -> Path | None:
    groups = group_by(runs, "goal_name")
    lin_data, ang_data, labels = [], [], []
    for (goal,), g in sorted(groups.items()):
        if not goal:
            continue
        lin = [v for v in metric_values(g, "linear_error_m") if v is not None]
        ang = [v for v in metric_values(g, "angular_error_deg") if v is not None]
        if lin or ang:
            lin_data.append(lin)
            ang_data.append(ang)
            labels.append(f"{goal}\n(n={max(len(lin), len(ang))})")
    if not labels:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.2))
    for ax, data, title, ylab, color in (
            (axes[0], lin_data, "Doğrusal Konum Hatası", "Hata (m)", "#8fc7ff"),
            (axes[1], ang_data, "Açısal Konum Hatası", "Hata (derece)", "#ffc78f")):
        usable = [d if d else [] for d in data]
        if not any(usable):
            ax.text(0.5, 0.5, "Ölçüm yok (N/A)", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_title(title)
            continue
        bp = ax.boxplot(usable, labels=labels, patch_artist=True, showmeans=True)
        for patch in bp["boxes"]:
            patch.set_facecolor(color)
        ax.set_title(title)
        ax.set_xlabel("Hedef")
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3, linestyle="--")
    fig.suptitle("Konumlandırma Doğruluğu ve Tekrarlanabilirliği",
                 fontsize=13, fontweight="bold")
    fig.text(0.99, 0.01, "Hata map çerçevesinde: Nav2 hedefi ile son map pozu arası.",
             ha="right", va="bottom", fontsize=8, alpha=0.75)
    out = out_dir / "04_konum_hatasi_kutu.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return out


# ── 5. yeniden planlama sayısı ────────────────────────────────────────────────
def _plot_replan(runs, out_dir: Path) -> Path | None:
    labels, vals = [], []
    for r in runs:
        v = r["metrics"].get("replan_count")
        if isinstance(v, (int, float)):
            labels.append(r["meta"].get("goal_name")
                          or r["meta"]["experiment_id"][:8])
            vals.append(float(v))
    if not vals:
        return None
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.bar(range(len(vals)), vals, color="#7d3bd8", edgecolor="black", linewidth=0.5)
    ax.set_xticks(range(len(vals)))
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylim(0, max(vals) * 1.25 + 1)
    return _finish(fig, ax, out_dir / "05_yeniden_planlama.png",
                   f"Yeniden Planlama Sayısı (n={len(vals)} koşu)",
                   "Koşu / hedef", "Yeniden planlama (adet)",
                   "İçeriği değişen /plan mesajları sayıldı; aynı planın "
                   "tekrar yayını sayılmadı.")


# ── 6. harita kapsama ─────────────────────────────────────────────────────────
def _plot_map_coverage(runs, out_dir: Path) -> Path | None:
    labels, cov, known = [], [], []
    for r in runs:
        c = r["metrics"].get("map_coverage_ratio")
        k = r["metrics"].get("map_known_area_m2")
        if isinstance(c, (int, float)):
            labels.append(f"{r['meta'].get('environment') or '?'}\n"
                          f"{r['meta']['experiment_id'][-6:]}")
            cov.append(float(c) * 100.0)
            known.append(float(k) if isinstance(k, (int, float)) else 0.0)
    if not cov:
        return None
    fig, ax = plt.subplots(figsize=FIGSIZE)
    bars = ax.bar(range(len(cov)), cov, color="#2f9e63",
                  edgecolor="black", linewidth=0.5)
    for b, c, k in zip(bars, cov, known):
        ax.text(b.get_x() + b.get_width() / 2, c + 0.8,
                f"%{c:.1f}\n{k:.1f} m²", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(range(len(cov)))
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylim(0, max(cov) * 1.3 + 2)
    return _finish(fig, ax, out_dir / "06_harita_kapsama.png",
                   f"Harita Kapsama Oranı ve Bilinen Alan (n={len(cov)} koşu)",
                   "Ortam / koşu", "Kapsama oranı (%)",
                   "Kapsama = (dolu + boş hücre) / toplam hücre.")


# ── 7. kamera FPS ve kare aralığı ─────────────────────────────────────────────
def _plot_camera(runs, out_dir: Path) -> Path | None:
    labels, fps, iv_mean, iv_std = [], [], [], []
    for r in runs:
        f = r["metrics"].get("camera_fps_mean")
        if not isinstance(f, (int, float)):
            continue
        labels.append(r["meta"]["experiment_id"][-6:])
        fps.append(float(f))
        m = r["metrics"].get("camera_interval_mean_s")
        s = r["metrics"].get("camera_interval_std_s")
        iv_mean.append(float(m) * 1000.0 if isinstance(m, (int, float)) else 0.0)
        iv_std.append(float(s) * 1000.0 if isinstance(s, (int, float)) else 0.0)
    if not fps:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.2))
    axes[0].bar(range(len(fps)), fps, color="#d83b7d",
                edgecolor="black", linewidth=0.5)
    axes[0].set_xticks(range(len(fps)))
    axes[0].set_xticklabels(labels, rotation=30, ha="right")
    axes[0].set_title("Kamera Kare Hızı")
    axes[0].set_xlabel("Koşu")
    axes[0].set_ylabel("FPS (kare/s)")
    axes[0].grid(True, alpha=0.3, linestyle="--")
    axes[1].bar(range(len(iv_mean)), iv_mean, yerr=iv_std, capsize=4,
                color="#3bd8c7", edgecolor="black", linewidth=0.5)
    axes[1].set_xticks(range(len(iv_mean)))
    axes[1].set_xticklabels(labels, rotation=30, ha="right")
    axes[1].set_title("Kareler Arası Süre (ortalama ± std)")
    axes[1].set_xlabel("Koşu")
    axes[1].set_ylabel("Süre (ms)")
    axes[1].grid(True, alpha=0.3, linestyle="--")
    fig.suptitle(f"Kamera Akış Performansı (n={len(fps)} koşu)",
                 fontsize=13, fontweight="bold")
    fig.text(0.99, 0.01,
             "Varış anları ROS saatiyle ölçüldü; uçtan uca gecikme ayrı "
             "raporlanır (güvenilir damga yoksa N/A).",
             ha="right", va="bottom", fontsize=8, alpha=0.75)
    out = out_dir / "07_kamera_fps_aralik.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return out
