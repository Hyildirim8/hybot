"""TELEOP <-> AUTONOMOUS mod geçiş testi.

Bu projede mod geçişini TETİKLEYEN bir servis veya komut topic'i YOKTUR
(çalışan sistemde arandı). Geçiş iki yolla olur:
  * operatör joystick'te Start'a basar (teleop_node modu çevirir),
  * /goal_pose'a hedef gelir (teleop_node bunu AUTO'ya geçiş sayar).

"Geçiş isteği" anı OPERATÖRÜN BUTONUNDAN okunur: /joy üzerindeki Start
butonunun (rover_params btn_auto_mode=9) yükselen kenarı. Doğrulama ise
/autonomous_mode (latched std_msgs/Bool) değişimidir.

Ölçülen gecikme = (mod topic'i değişti) - (Start butonuna basıldı).
Bu, insan reaksiyon süresini İÇERMEZ; gerçek sistem gecikmesidir. Operatörün
ayrıca bir tuşa basmasına gerek yoktur, dolayısıyla test TTY istemez.
"""

from __future__ import annotations

import statistics
from typing import Any, Callable

from ..monitors import BenchmarkMonitor


def run_mode_switch(monitor: BenchmarkMonitor, store, repetitions: int = 10,
                    confirm_timeout_s: float = 10.0,
                    press_timeout_s: float = 120.0,
                    prompt: Callable[[str], str] | None = None,
                    on_status: Callable[[str], None] | None = None,
                    ) -> dict[str, Any]:
    """`repetitions` kez mod geçişi ölçer. Referans: /joy Start butonu."""
    del prompt  # artık operatör tuşu gerekmiyor

    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", message=m)

    if not monitor.wait_until(lambda: monitor.autonomous is not None, 15.0):
        say("/autonomous_mode okunamadı — teleop_node çalışıyor mu?")
        return {"mode_switch_attempts": 0, "mode_switch_success": None,
                "mode_switch_failed": None, "mode_switch_latencies_s": None,
                "mode_initial": None}
    if not monitor.wait_until(lambda: bool(monitor.joy_buttons), 15.0):
        say("/joy okunamadı — joystick bağlı ve joy_node çalışıyor mu?")
        return {"mode_switch_attempts": 0, "mode_switch_success": None,
                "mode_switch_failed": None, "mode_switch_latencies_s": None,
                "mode_initial": bool(monitor.autonomous)}

    initial = monitor.autonomous
    say(f"Başlangıç modu: {'AUTONOMOUS' if initial else 'TELEOP'}")
    say(f"{repetitions} kez Start'a basın. Her basış arasında modun "
        f"değiştiğini görün; ekrana bakmanıza gerek yok.")

    latencies: list[float] = []
    success = failed = 0
    records: list[dict[str, Any]] = []

    for i in range(1, repetitions + 1):
        before = monitor.autonomous
        target = not before
        seen = len(monitor.auto_button_presses)
        if not monitor.wait_until(
                lambda n=seen: len(monitor.auto_button_presses) > n,
                press_timeout_s):
            say(f"  [{i}/{repetitions}] Start'a basılmadı "
                f"({press_timeout_s:.0f} s) — test bitiriliyor.")
            break
        t_press = monitor.auto_button_presses[-1]
        store.event("mode_request", index=i, from_mode=bool(before),
                    to_mode=bool(target), t_press_s=t_press,
                    reference="joy_button")

        ok = monitor.wait_until(
            lambda t=target: (monitor.autonomous is not None
                              and monitor.autonomous == t),
            confirm_timeout_s)
        if ok:
            latency = monitor.now_s() - t_press
            latencies.append(latency)
            success += 1
            verdict = f"onaylandı, {latency*1000:.0f} ms"
        else:
            failed += 1
            latency = None
            verdict = f"ONAYLANMADI ({confirm_timeout_s:.0f} s içinde)"
        records.append({"index": i, "from_mode": bool(before),
                        "to_mode": bool(target), "confirmed": ok,
                        "latency_s": latency})
        store.event("mode_result", index=i, confirmed=ok, latency_s=latency)
        store.sample("mode_switch", {
            "index": i, "from_mode": int(bool(before)),
            "to_mode": int(bool(target)), "confirmed": int(ok),
            "latency_s": "" if latency is None else round(latency, 4)})
        say(f"  [{i}/{repetitions}] "
            f"{'TELEOP->AUTO' if target else 'AUTO->TELEOP'}: {verdict}")
        # Ayni basisi iki kez saymamak icin butonun birakilmasini bekle
        monitor.wait_until(
            lambda: not (monitor.joy_buttons
                         and len(monitor.joy_buttons) > monitor.auto_button_index
                         and monitor.joy_buttons[monitor.auto_button_index]),
            5.0)

    attempts = len(records)
    # Liste halindeki gecikmeler ozet tabloya giremiyor; skaler karsiliklarini
    # da yaz (toplu rapor bunlari ortalama/medyan/GA ile ozetliyor).
    lat_mean = statistics.fmean(latencies) if latencies else None
    lat_med = statistics.median(latencies) if latencies else None
    lat_max = max(latencies) if latencies else None
    return {
        "mode_switch_latency_mean_s": lat_mean,
        "mode_switch_latency_median_s": lat_med,
        "mode_switch_latency_max_s": lat_max,
        "mode_initial": bool(initial),
        "mode_switch_attempts": attempts,
        "mode_switch_success": success,
        "mode_switch_failed": failed,
        "mode_switch_success_rate": (success / attempts) if attempts else None,
        "mode_switch_latencies_s": latencies or None,
        "mode_switch_records": records,
        "latency_reference": "joy_start_button",  # insan reaksiyonu HARIC
    }
