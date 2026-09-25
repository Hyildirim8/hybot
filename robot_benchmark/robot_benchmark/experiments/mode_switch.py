"""TELEOP <-> AUTONOMOUS mod geçiş testi.

Bu projede mod geçişini TETİKLEYEN bir servis veya komut topic'i YOKTUR
(çalışan sistemde arandı). Geçiş iki yolla olur:
  * operatör joystick'te Start'a basar (teleop_node modu çevirir),
  * /goal_pose'a hedef gelir (teleop_node bunu AUTO'ya geçiş sayar).

Dolayısıyla "geçiş isteği" zamanı ancak OPERATÖRÜN işaretiyle bilinebilir:
operatör Start'a bastığı anda Enter'a basar, biz o anı referans alırız.
Doğrulama ise objektiftir: /autonomous_mode (latched std_msgs/Bool) değişimi.

Ölçülen gecikme = (mod topic'i değişti) - (operatörün Enter'ı).
Bu yüzden ölçüme insan reaksiyon süresi karışır; raporda böyle belirtilir.
"""

from __future__ import annotations

from typing import Any, Callable

from ..monitors import BenchmarkMonitor


def run_mode_switch(monitor: BenchmarkMonitor, store, repetitions: int = 10,
                    confirm_timeout_s: float = 10.0,
                    prompt: Callable[[str], str] | None = None,
                    on_status: Callable[[str], None] | None = None,
                    ) -> dict[str, Any]:
    """`repetitions` kez mod geçişi ölçer."""
    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", message=m)

    def ask(m: str) -> str:
        return prompt(m) if prompt else input(m)

    # Başlangıç modunu öğren (latched olduğu için hemen gelmeli)
    if not monitor.wait_until(lambda: monitor.autonomous is not None, 10.0):
        say("/autonomous_mode okunamadı — teleop_node çalışıyor mu?")
        return {"mode_switch_attempts": 0, "mode_switch_success": None,
                "mode_switch_failed": None, "mode_switch_latencies_s": None,
                "mode_initial": None}

    initial = monitor.autonomous
    say(f"Başlangıç modu: {'AUTONOMOUS' if initial else 'TELEOP'}")
    latencies: list[float] = []
    success = 0
    failed = 0
    records: list[dict[str, Any]] = []

    for i in range(1, repetitions + 1):
        before = monitor.autonomous
        target = not before
        ask(f"  [{i}/{repetitions}] Şimdi Start'a basın "
            f"({'TELEOP' if before else 'AUTONOMOUS'} -> "
            f"{'AUTONOMOUS' if target else 'TELEOP'}), sonra Enter: ")
        t_request = monitor.now_s()
        store.event("mode_request", index=i, from_mode=bool(before),
                    to_mode=bool(target), t_request_s=t_request)
        # target'i varsayilan argumanla bagla: dongu degiskenini gec
        # baglamak (late binding) hatali karsilastirmaya yol acar.
        ok = monitor.wait_until(
            lambda t=target: (monitor.autonomous is not None
                              and monitor.autonomous == t),
            confirm_timeout_s)
        if ok:
            latency = monitor.now_s() - t_request
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
            "index": i, "from_mode": int(bool(before)), "to_mode": int(bool(target)),
            "confirmed": int(ok), "latency_s": "" if latency is None else round(latency, 4),
        })
        say(f"  [{i}/{repetitions}] {verdict}")

    return {
        "mode_initial": bool(initial),
        "mode_switch_attempts": repetitions,
        "mode_switch_success": success,
        "mode_switch_failed": failed,
        "mode_switch_success_rate": success / repetitions if repetitions else None,
        "mode_switch_latencies_s": latencies or None,
        "mode_switch_records": records,
        "latency_reference": "operator_keypress",  # insan reaksiyonu dahil
    }
