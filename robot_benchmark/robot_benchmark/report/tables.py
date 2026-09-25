"""Markdown ve LaTeX tabloları.

Eksik değerler N/A olarak yazılır (0 değil). Başarısız koşular ayrı satırda
sayılır, tablodan çıkarılmaz.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .aggregate import LABELS_TR, UNITS, build_summary

NA = "N/A"

STATUS_TR = {
    "succeeded": "Başarılı", "aborted": "İptal (abort)",
    "canceled": "Vazgeçildi (cancel)", "no_goal": "Hedef gelmedi",
    "executing": "Yürüyor", "accepted": "Kabul edildi",
    "canceling": "Vazgeçiliyor", "unknown": "Bilinmiyor",
}


def _n(v: Any, digits: int = 3) -> str:
    if v is None:
        return NA
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def _tex_escape(s: str) -> str:
    for a, b in (("\\", r"\textbackslash{}"), ("&", r"\&"), ("%", r"\%"),
                 ("$", r"\$"), ("#", r"\#"), ("_", r"\_"), ("{", r"\{"),
                 ("}", r"\}"), ("~", r"\textasciitilde{}"),
                 ("^", r"\textasciicircum{}")):
        s = s.replace(a, b)
    return s


def _metric_rows(metrics: dict[str, dict]) -> list[list[str]]:
    rows = []
    for m, s in metrics.items():
        rows.append([
            LABELS_TR.get(m, m), UNITS.get(m, "-"), str(s["n"]),
            _n(s["mean"]), _n(s["median"]), _n(s["std"]),
            _n(s["minimum"]), _n(s["maximum"]),
            NA if s["ci95_low"] is None
            else f"[{s['ci95_low']:.3f}; {s['ci95_high']:.3f}]",
        ])
    return rows


HEADER = ["Metrik", "Birim", "n", "Ortalama", "Medyan", "Std",
          "Min", "Maks", "%95 GA"]


def markdown_report(base: Path | None = None) -> str:
    """Akademik rapora yapıştırılabilir Markdown."""
    s = build_summary(base)
    out: list[str] = []
    out.append("# Robot Performans Testi Sonuçları\n")
    out.append(f"- Toplam koşu sayısı: **{s['run_count']}**")
    out.append(f"- Yarıda kesilen koşu: **{s['partial_run_count']}** "
               f"(veriler korundu, sonuçlara dahil)")
    out.append(f"- Sahte/mock koşu: **{s['mock_run_count']}** "
               f"(gerçek robot ölçümü değildir)\n")
    out.append("> Eksik ölçümler `N/A` ile gösterilir; 0 ile doldurulmaz. "
               "Başarısız koşular sonuçlardan çıkarılmaz.\n")

    for kind, blk in s["kinds"].items():
        out.append(f"\n## Deney: {kind}\n")
        out.append(f"- Koşu sayısı: **{blk['run_count']}**")
        rate = blk["goal_success_rate"]
        out.append(f"- Hedefe ulaşma oranı: "
                   f"**{NA if rate is None else f'%{rate*100:.1f}'}**")
        if blk["goal_status_counts"]:
            parts = ", ".join(f"{STATUS_TR.get(k, k)}: {v}"
                              for k, v in sorted(blk["goal_status_counts"].items()))
            out.append(f"- Durum dağılımı: {parts}")
        out.append("")
        rows = _metric_rows(blk["metrics"])
        if rows:
            out.append("| " + " | ".join(HEADER) + " |")
            out.append("|" + "|".join(["---"] * len(HEADER)) + "|")
            for r in rows:
                out.append("| " + " | ".join(r) + " |")
        else:
            out.append("_Bu deney için sayısal metrik ölçülemedi._")
        out.append("")

    if s["by_goal"]:
        out.append("\n## Hedef bazlı tekrarlanabilirlik\n")
        for blk in s["by_goal"].values():
            out.append(f"\n### {blk['kind']} — ortam: "
                       f"{blk['environment'] or '(belirtilmedi)'} — hedef: "
                       f"{blk['goal_name']}\n")
            out.append(f"- Tekrar sayısı: **{blk['run_count']}**")
            rate = blk["goal_success_rate"]
            out.append(f"- Başarı oranı: "
                       f"**{NA if rate is None else f'%{rate*100:.1f}'}**\n")
            rows = _metric_rows(blk["metrics"])
            if rows:
                out.append("| " + " | ".join(HEADER) + " |")
                out.append("|" + "|".join(["---"] * len(HEADER)) + "|")
                for r in rows:
                    out.append("| " + " | ".join(r) + " |")
            out.append("")
    return "\n".join(out) + "\n"


def latex_report(base: Path | None = None) -> str:
    """LaTeX tabloları (booktabs). Ana dokümana \\input ile alınabilir."""
    s = build_summary(base)
    out: list[str] = []
    out.append("% robot_benchmark tarafindan uretildi.")
    out.append("% Gerekli paketler: \\usepackage{booktabs}")
    out.append("% Eksik olcumler N/A; sifir ile doldurulmamistir.")
    out.append("")
    for kind, blk in s["kinds"].items():
        rows = _metric_rows(blk["metrics"])
        if not rows:
            continue
        rate = blk["goal_success_rate"]
        rate_s = NA if rate is None else f"\\%{rate*100:.1f}"
        out.append("\\begin{table}[htbp]")
        out.append("\\centering")
        out.append(f"\\caption{{{_tex_escape(kind)} deneyi sonuclari "
                   f"(n={blk['run_count']} kosu, hedefe ulasma orani {rate_s}).}}")
        out.append(f"\\label{{tab:benchmark-{_tex_escape(kind)}}}")
        out.append("\\begin{tabular}{lrrrrrrrl}")
        out.append("\\toprule")
        out.append(" & ".join(_tex_escape(h) for h in HEADER) + " \\\\")
        out.append("\\midrule")
        for r in rows:
            out.append(" & ".join(_tex_escape(c) for c in r) + " \\\\")
        out.append("\\bottomrule")
        out.append("\\end{tabular}")
        out.append("\\end{table}")
        out.append("")
    return "\n".join(out) + "\n"


def write_tables(out_dir: Path, base: Path | None = None) -> dict[str, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    md = out_dir / "sonuclar.md"
    tex = out_dir / "sonuclar.tex"
    md.write_text(markdown_report(base), encoding="utf-8")
    tex.write_text(latex_report(base), encoding="utf-8")
    return {"markdown": md, "latex": tex}
