"""Terminal menüsü ve canlı gösterge.

curses kullanılmaz: SSH üzerinden ve dar terminallerde daha dayanıklı olsun
diye düz metin + arka planda stdin okuyan bir iş parçacığı tercih edildi.

Canlı gösterge: süre, gidilen mesafe, mevcut poz (map ve odom), hedefe kalan
hata, Nav2 durumu, mod, tekerlek duruşu.

Operatör kontrolleri (test sırasında yazıp Enter):
    d  -> durdur (koşuyu bitir, veriyi kaydet)
    b  -> başarısız işaretle
    c  -> çarpışma işaretle
    n  -> not ekle
    ?  -> yardım
"""

from __future__ import annotations

import queue
import sys
import threading
from typing import Any



HELP = """
  Kontroller (yazıp Enter):
    d  koşuyu durdur ve kaydet
    b  bu koşuyu BAŞARISIZ işaretle
    c  ÇARPIŞMA işaretle
    n  not ekle
    ?  bu yardım
"""


class KeyReader:
    """stdin'i arka planda okur; canlı gösterge bloklanmasın."""

    def __init__(self) -> None:
        self.q: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line:
                return
            self.q.put(line.strip())

    def get(self) -> str | None:
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None

    def stop(self) -> None:
        self._stop.set()


def _ask(msg: str, default: str = "") -> str:
    try:
        v = input(msg).strip()
    except EOFError:
        return default
    return v or default


def _ask_int(msg: str, default: int) -> int:
    raw = _ask(f"{msg} [{default}]: ")
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default


def live_line(monitor, tracker, track, start_s: float) -> str:
    """Tek satırlık canlı durum."""
    now = monitor.now_s()
    dur = now - start_s
    mp = monitor.map_pose()
    op = monitor.odom_pose
    goal = tracker.last_goal_pose if tracker else None
    err = "N/A"
    if goal and mp:
        import math
        err = f"{math.hypot(mp.x - goal['x'], mp.y - goal['y']):.2f} m"
    active = tracker.active_goal() if tracker else None
    latest = tracker.latest_goal() if tracker else None
    status = (active.status if active else (latest.status if latest else "—"))
    mode = ("?" if monitor.autonomous is None
            else ("AUTO" if monitor.autonomous else "TELEOP"))
    still = "durdu" if monitor.wheels_still() else "hareket"
    pos_map = f"({mp.x:+.2f},{mp.y:+.2f})" if mp else "N/A"
    pos_odom = f"({op.x:+.2f},{op.y:+.2f})" if op else "N/A"
    return (f"  süre {dur:6.1f}s | gidilen {track.travelled_m:6.2f}m | "
            f"map {pos_map} | odom {pos_odom} | hedefe {err} | "
            f"Nav2 {status:<9} | {mode:<6} | {still}")


def run_menu(args) -> int:
    """Etkileşimli menü: test seç, çalıştır, canlı izle."""
    # _Args argparse.Namespace gibi dinamik bir taşıyıcıdır; alanları seçime
    # göre burada atanır. Bu pylint uyarısı tasarım gereğidir.
    # pylint: disable=attribute-defined-outside-init
    from .cli import (cmd_battery, cmd_camera, cmd_goal_based, cmd_mapping,
                      cmd_mecanum, cmd_mode_switch, cmd_report)

    items = [
        ("1", "Hedefe ulaşma testi", "navigation"),
        ("2", "Engelden kaçınma / yeniden planlama", "obstacle"),
        ("3", "Konumlandırma tekrarlanabilirliği", "repeatability"),
        ("4", "Mecanum hareket testi", "mecanum"),
        ("5", "TELEOP/AUTONOMOUS mod geçişi", "mode-switch"),
        ("6", "Haritalama testi", "mapping"),
        ("7", "Kamera akış testi", "camera"),
        ("8", "Çalışma süresi / batarya", "battery"),
        ("9", "Rapor üret (CSV/JSON/PNG/MD/TeX)", "report"),
        ("c", "Canlı izleme (ölçüm kaydetmeden)", "live"),
        ("q", "Çıkış", None),
    ]

    while True:
        print("\n" + "=" * 62)
        print("  robot_benchmark — ecza-robotu performans testi")
        print("  GÜVENLİK: bu araç robotu kendiliğinden HAREKET ETTİRMEZ.")
        print("=" * 62)
        for key, label, _ in items:
            print(f"   {key}) {label}")
        choice = _ask("\n  Seçim: ").lower()
        match = next((it for it in items if it[0] == choice), None)
        if match is None:
            print("  Geçersiz seçim.")
            continue
        kind = match[2]
        if kind is None:
            return 0
        if kind == "live":
            _live_only(args)
            continue
        if kind == "report":
            ns = _Args(results=None, out=None, mock=False, seed=7)
            cmd_report(ns)
            continue

        env = _ask(f"  Ortam adı [{args.environment or 'belirtilmedi'}]: ",
                   args.environment)
        note = _ask("  Operatör notu (boş geçilebilir): ")
        ns = _Args(environment=env, note=note, record_bag=False,
                   use_sim_time=bool(getattr(args, "use_sim_time", False)),
                   goal="", repetitions=1, wait_goal=300.0, goal_timeout=300.0,
                   allow_motion=False, speed=0.12, duration=30.0,
                   directions=None, start=None, end=None, unit="V")
        if kind in ("navigation", "obstacle", "repeatability"):
            ns.goal = _ask("  Hedef adı (ör. H1-kapi): ", "H1")
            ns.repetitions = _ask_int("  Tekrar sayısı", 3)
            if _ask("  rosbag2 kaydı alınsın mı? (e/h) [h]: ").lower().startswith("e"):
                ns.record_bag = True
            cmd_goal_based(ns, kind)
        elif kind == "mecanum":
            ns.goal = _ask("  Test adı: ", "6yon")
            if _ask("  Benchmark robotu HAREKET ETTİRSİN mi? (e/h) [h]: "
                    ).lower().startswith("e"):
                ns.allow_motion = True
            cmd_mecanum(ns)
        elif kind == "mode-switch":
            ns.goal = _ask("  Test adı: ", "mod-gecis")
            ns.repetitions = _ask_int("  Tekrar sayısı", 10)
            cmd_mode_switch(ns)
        elif kind == "mapping":
            ns.goal = _ask("  Harita adı: ", "harita-1")
            ns.duration = float(_ask_int("  Süre (s)", 120))
            cmd_mapping(ns)
        elif kind == "camera":
            ns.goal = _ask("  Test adı: ", "kamera-1")
            ns.duration = float(_ask_int("  Süre (s)", 30))
            cmd_camera(ns)
        elif kind == "battery":
            ns.goal = _ask("  Test adı: ", "dayanim-1")
            ns.duration = float(_ask_int("  Süre (s, 0=Enter bekle)", 0))
            ns.unit = _ask("  Birim [V]: ", "V")
            cmd_battery(ns)


def _live_only(args) -> int:
    """Hiçbir şey kaydetmeden canlı izleme (güvenli keşif)."""
    from .cli import _ros_init, _ros_shutdown
    from .experiments.common import PoseTrack
    from .nav2_tracker import Nav2GoalTracker

    rclpy, monitor = _ros_init(args, {"pose", "wheels", "mode", "plan", "scan"})
    tracker = Nav2GoalTracker(monitor)
    track = PoseTrack()
    keys = KeyReader()
    start = monitor.now_s()
    print("  Canlı izleme — çıkmak için 'd' + Enter" + HELP)
    try:
        while True:
            monitor.spin_for(0.5)
            track.update(monitor)
            print(live_line(monitor, tracker, track, start), end="\r", flush=True)
            k = keys.get()
            if k is None:
                continue
            if k.lower() == "d":
                break
            if k == "?":
                print("\n" + HELP)
    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        print()
        _ros_shutdown(rclpy, monitor)
    return 0


class _Args:  # pylint: disable=too-few-public-methods
    """cmd_* fonksiyonlarına argparse.Namespace yerine geçen basit taşıyıcı.

    argparse.Namespace gibi alanları dinamik olarak taşır; menüde seçime göre
    alan atanır. Bu yüzden pylint'in "attribute-defined-outside-init" uyarısı
    burada tasarım gereğidir ve bastırılmıştır.
    """

    # pylint: disable=attribute-defined-outside-init

    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)

    def __repr__(self) -> str:  # pragma: no cover
        return f"_Args({self.__dict__})"
