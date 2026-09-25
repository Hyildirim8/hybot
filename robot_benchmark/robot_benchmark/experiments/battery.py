"""Çalışma süresi ve batarya testi.

BU ROBOTTA BATARYA ÖLÇÜM DONANIMI VE TOPIC'İ YOKTUR. 2026-09-25'te tüm kod
tabanı (src/, firmware/main/, config/) battery|voltage|vbat için arandı;
hiçbir yayıncı bulunmadı. ros_names.TOPIC_BATTERY = None.

Bu yüzden değerler OPERATÖRDEN alınır. Batarya kapasitesi (Ah/Wh) bilinmediği
için KALAN ÇALIŞMA SÜRESİ TAHMİN EDİLMEZ — uydurmak yerine N/A raporlanır.
Saatlik tüketim yalnızca birim başına değişim olarak verilir (ör. V/saat).
"""

from __future__ import annotations

from typing import Any, Callable

from .. import ros_names as N
from ..monitors import BenchmarkMonitor


def run_battery(monitor: BenchmarkMonitor, store, duration_s: float | None = None,
                start_value: float | None = None, end_value: float | None = None,
                unit: str = "V",
                prompt: Callable[[str], str] | None = None,
                on_status: Callable[[str], None] | None = None) -> dict[str, Any]:
    """Çalışma süresi ölçer; batarya değerlerini operatörden alır."""
    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", message=m)

    def ask(m: str) -> str:
        return prompt(m) if prompt else input(m)

    if N.TOPIC_BATTERY is not None:  # pragma: no cover - ileride donanım eklenirse
        say("Batarya topic'i tanımlı; otomatik ölçüm yapılacak.")

    say("Bu robotta batarya topic'i YOK — değerler operatörden alınacak.")
    if start_value is None:
        start_value = _ask_float(ask, f"  Başlangıç batarya değeri ({unit}, "
                                      f"bilinmiyorsa boş bırakın): ")
    t0 = monitor.now_s()
    store.event("battery_start", value=start_value, unit=unit, t_ros_s=t0)

    if duration_s is not None:
        say(f"{duration_s:.0f} s çalışma süresi ölçülüyor "
            f"(Ctrl+C ile erken bitirilebilir, veri korunur)…")
        end_t = t0 + duration_s
        while monitor.now_s() < end_t:
            monitor.spin_for(5.0)
            store.sample("runtime", {
                "t_rel_s": round(monitor.now_s() - t0, 1),
                "autonomous": "" if monitor.autonomous is None else int(monitor.autonomous),
                "wheels_still": int(monitor.wheels_still()),
            })
    else:
        ask("  Test bitince Enter'a basın: ")

    t1 = monitor.now_s()
    if end_value is None:
        end_value = _ask_float(ask, f"  Bitiş batarya değeri ({unit}, "
                                   f"bilinmiyorsa boş bırakın): ")
    store.event("battery_end", value=end_value, unit=unit, t_ros_s=t1)

    elapsed = t1 - t0
    drop = None
    per_hour = None
    if start_value is not None and end_value is not None:
        drop = start_value - end_value
        if elapsed > 0:
            per_hour = drop / (elapsed / 3600.0)

    out = {
        "battery_source": "operator_input",   # otomatik DEĞİL
        "battery_unit": unit,
        "battery_start": start_value,
        "battery_end": end_value,
        "battery_drop": drop,
        "battery_drop_per_hour": per_hour,
        "runtime_duration_s": elapsed,
        "runtime_duration_min": elapsed / 60.0 if elapsed else None,
        # Kapasite bilinmiyor -> kalan süre TAHMİN EDİLMEZ.
        "battery_remaining_runtime_s": None,
        "battery_remaining_note": (
            "kapasite (Ah/Wh) bilinmiyor; kalan çalışma süresi tahmin edilmedi"),
    }
    say(f"Süre={elapsed/60:.1f} dk  düşüş="
        f"{'N/A' if drop is None else f'{drop:.2f} {unit}'}  "
        f"saatlik={'N/A' if per_hour is None else f'{per_hour:.2f} {unit}/saat'}")
    return out


def _ask_float(ask: Callable[[str], str], msg: str) -> float | None:
    """Boş girdi -> None (N/A). Uydurma değer üretilmez."""
    raw = (ask(msg) or "").strip().replace(",", ".")
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        return None
