"""İstatistik yardımcıları.

İki kural:
  1. EKSİK METRİK 0 İLE DOLDURULMAZ. Ölçülemeyen her şey None'dur ve raporda
     "N/A" görünür. 0 geçerli bir ölçüm değeridir; ölçüm yokluğuyla
     karıştırılmamalıdır.
  2. BAŞARISIZ KOŞULAR SİLİNMEZ. Ayrı durum olarak sayılır ve rapora girer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Iterable, Sequence

NA = "N/A"

# Student-t iki yanlı %95 kritik değerleri, serbestlik derecesi 1..30.
# Küçük örneklem (n<30) için normal dağılım 1.96 kullanmak güven aralığını
# olduğundan dar gösterir; bitirme projesinde n genelde 3-10 olacağı için
# t dağılımı şart.
_T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
    8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145,
    15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
    21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060, 26: 2.056,
    27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
}


def t_critical_95(dof: int) -> float:
    """dof serbestlik derecesi için iki yanlı %95 t kritik değeri."""
    if dof <= 0:
        return float("nan")
    if dof in _T95:
        return _T95[dof]
    return 1.96  # dof > 30 -> normale yakınsar


@dataclass
class Summary:
    """Bir metriğin özet istatistiği. Ölçüm yoksa alanlar None kalır."""

    n: int = 0
    mean: float | None = None
    median: float | None = None
    std: float | None = None          # örneklem standart sapması (ddof=1)
    minimum: float | None = None
    maximum: float | None = None
    ci95_low: float | None = None
    ci95_high: float | None = None
    ci95_halfwidth: float | None = None

    def as_dict(self) -> dict:
        return asdict(self)

    def fmt(self, digits: int = 3, unit: str = "") -> str:
        """İnsan okunur tek satır. Ölçüm yoksa N/A."""
        if self.n == 0 or self.mean is None:
            return NA
        suffix = f" {unit}" if unit else ""
        out = f"{self.mean:.{digits}f}{suffix}"
        if self.std is not None:
            out += f" ± {self.std:.{digits}f}"
        out += f" (n={self.n})"
        return out


def summarize(values: Iterable[float | None]) -> Summary:
    """None ve NaN'ları atarak özet üretir.

    None'lar ATILIR ama n yalnızca geçerli ölçümleri sayar — böylece raporda
    "kaç koşudan ölçülebildi" görünür.
    """
    clean = [
        float(v) for v in values
        if v is not None and not (isinstance(v, float) and math.isnan(v))
    ]
    n = len(clean)
    if n == 0:
        return Summary(n=0)
    clean.sort()
    mean = sum(clean) / n
    if n % 2:
        median = clean[n // 2]
    else:
        median = (clean[n // 2 - 1] + clean[n // 2]) / 2.0
    if n >= 2:
        var = sum((x - mean) ** 2 for x in clean) / (n - 1)
        std = math.sqrt(var)
        half = t_critical_95(n - 1) * std / math.sqrt(n)
        lo, hi = mean - half, mean + half
    else:
        # Tek ölçümden standart sapma veya güven aralığı ÜRETİLEMEZ.
        std = None
        half = lo = hi = None
    return Summary(
        n=n, mean=mean, median=median, std=std,
        minimum=clean[0], maximum=clean[-1],
        ci95_low=lo, ci95_high=hi, ci95_halfwidth=half,
    )


def success_rate(statuses: Sequence[str]) -> tuple[float | None, dict[str, int]]:
    """Başarı oranı (0..1) ve durum dağılımı.

    Başarısız koşular silinmez; hepsi dağılımda sayılır. Hiç koşu yoksa
    oran None döner (0.0 değil — 0 başarı ile hiç veri aynı şey değildir).
    """
    counts: dict[str, int] = {}
    for s in statuses:
        counts[s] = counts.get(s, 0) + 1
    total = len(statuses)
    if total == 0:
        return None, counts
    return counts.get("succeeded", 0) / total, counts


def fmt(value: float | None, digits: int = 3, unit: str = "") -> str:
    """Tek değeri biçimlendirir; None ise N/A."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return NA
    suffix = f" {unit}" if unit else ""
    return f"{value:.{digits}f}{suffix}"


def path_length(points: Sequence[tuple[float, float]]) -> float | None:
    """Ardışık (x, y) noktaları arasındaki toplam yol uzunluğu (m)."""
    if points is None or len(points) < 2:
        return None
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def angle_diff_deg(a_rad: float, b_rad: float) -> float:
    """İki yaw arasındaki en kısa açı farkı, derece, [-180, 180]."""
    d = a_rad - b_rad
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return math.degrees(d)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Kuaterniyondan yaw (rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
