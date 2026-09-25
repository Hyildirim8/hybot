"""robot_benchmark komut satırı arayüzü.

GÜVENLİK: Varsayılan mod SALT GÖZLEM ve ÖLÇÜMDÜR. Hiçbir alt komut robotu
kendiliğinden hareket ettirmez. Tek istisna `mecanum --allow-motion`'dır ve
o da açık operatör onayı ister.

Kullanım:
    python3 -m robot_benchmark.cli <alt-komut> [seçenekler]
"""

from __future__ import annotations

import argparse
import shutil
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from . import ros_names as N
from .store import (RunMeta, RunStore, new_experiment_id, outputs_dir,
                    results_dir)

EXPERIMENTS = ("navigation", "obstacle", "repeatability", "mecanum",
               "mode-switch", "mapping", "camera", "battery")

BAG_TOPICS = [
    N.TOPIC_ODOM_FILTERED, N.TOPIC_ODOM_WHEEL, N.TOPIC_MAP,
    N.TOPIC_GLOBAL_PLAN, N.TOPIC_LOCAL_PLAN, N.TOPIC_SCAN,
    N.TOPIC_AUTONOMOUS_MODE, N.TOPIC_GOAL_POSE, N.TOPIC_WHEEL_VELOCITIES,
    N.TOPIC_IMU, N.TOPIC_CMD_VEL_NAV_SMOOTHED, N.TOPIC_BEHAVIOR_TREE_LOG,
    "/tf", "/tf_static",
]


# ── yardımcılar ───────────────────────────────────────────────────────────────
def say(msg: str) -> None:
    print(msg, flush=True)


def make_meta(kind: str, args) -> RunMeta:
    return RunMeta(
        experiment_id=new_experiment_id(kind),
        kind=kind,
        started_at=datetime.now().isoformat(timespec="seconds"),
        environment=getattr(args, "environment", "") or "",
        goal_name=getattr(args, "goal", "") or "",
        operator_note=getattr(args, "note", "") or "",
        repetition=getattr(args, "repetition", 1) or 1,
        use_sim_time=bool(getattr(args, "use_sim_time", False)),
        mock=bool(getattr(args, "mock", False)),
    )


class BagRecorder:
    """İsteğe bağlı rosbag2 kaydı (--record-bag)."""

    def __init__(self, out_dir: Path, enabled: bool):
        self.proc: subprocess.Popen | None = None
        self.path = Path(out_dir) / "bag"
        self.enabled = enabled and shutil.which("ros2") is not None
        if enabled and not self.enabled:
            say("  [bag] ros2 bulunamadı, kayıt atlandı.")

    def __enter__(self) -> "BagRecorder":
        if self.enabled:
            cmd = ["ros2", "bag", "record", "-o", str(self.path)] + BAG_TOPICS
            try:
                self.proc = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                say(f"  [bag] kayıt başladı -> {self.path}")
            except Exception as exc:
                say(f"  [bag] başlatılamadı: {exc}")
                self.proc = None
        return self

    def __exit__(self, *exc) -> bool:
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            say(f"  [bag] kayıt durdu -> {self.path}")
        return False


def _ros_init(args, want: set[str]):
    """rclpy başlat ve monitörü kur. ROS yoksa anlamlı hata verir."""
    try:
        import rclpy
    except ImportError as exc:
        say("HATA: rclpy bulunamadı. ROS ortamını kaynaklayın:")
        say("  source /opt/ros/humble/setup.bash")
        say("Gerçek robot olmadan yazılımı doğrulamak için: "
            "python3 -m robot_benchmark.cli report --mock")
        raise SystemExit(2) from exc
    from .monitors import BenchmarkMonitor
    rclpy.init()
    monitor = BenchmarkMonitor(want=want)
    if args.use_sim_time:
        from rclpy.parameter import Parameter
        monitor.set_parameters([Parameter("use_sim_time",
                                          Parameter.Type.BOOL, True)])
        say("  use_sim_time=True")
    return rclpy, monitor


def _ros_shutdown(rclpy, monitor) -> None:
    try:
        monitor.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def _ask(msg: str) -> str:
    try:
        return input(msg)
    except EOFError:
        return ""


def _confirm_motion() -> bool:
    say("")
    say("  ============================ UYARI ============================")
    say("  --allow-motion açık: benchmark ROBOTU HAREKET ETTİRECEK.")
    say("  * Robotun etrafını boşaltın, tekerleklerin önünü açın.")
    say("  * Acil durdurma: Ctrl+C (sıfır hız yayınlanır) veya")
    say("    joystick'te dead-man tuşunu bırakın / gücü kesin.")
    say("  * Komut /cmd_vel_nav_smoothed'e gider; teleop_node'un çarpışma")
    say("    kapısından geçer ama BU BİR GÜVENLİK GARANTİSİ DEĞİLDİR.")
    say("  ===============================================================")
    return _ask("  Devam etmek için 'evet' yazın: ").strip().lower() == "evet"


# ── alt komutlar ──────────────────────────────────────────────────────────────
def cmd_goal_based(args, kind: str) -> int:
    from .experiments.goal_based import needed_topics, run_goal_run
    from .nav2_tracker import Nav2GoalTracker

    rclpy, monitor = _ros_init(args, needed_topics(kind))
    tracker = Nav2GoalTracker(monitor)
    total = max(1, args.repetitions)
    try:
        for rep in range(1, total + 1):
            args.repetition = rep
            meta = make_meta(kind, args)
            say(f"\n=== {kind} — tekrar {rep}/{total} "
                f"(id: {meta.experiment_id}) ===")
            with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
                metrics = run_goal_run(
                    monitor, tracker, store,
                    wait_goal_s=args.wait_goal, goal_timeout_s=args.goal_timeout,
                    on_status=say)
                if kind == "obstacle":
                    ans = _ask("  Çarpışma oldu mu? (e/h, boş=hayır): ")
                    hit = ans.strip().lower().startswith("e")
                    metrics["operator_collision_marked"] = hit
                    metrics["reached_after_obstacle"] = (
                        metrics.get("goal_status") == "succeeded")
                    store.event("operator_collision", marked=hit)
                extra = _ask("  Bu koşu için not (boş geçilebilir): ").strip()
                if extra:
                    store.meta.operator_note = (
                        (store.meta.operator_note + " | " + extra).strip(" |"))
                store.set_metrics(**metrics)
            say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


def cmd_mecanum(args) -> int:
    from .experiments.mecanum import MotionGuard, measure_direction
    rclpy, monitor = _ros_init(args, {"pose", "wheels", "scan", "mode"})
    motion = None
    try:
        if args.allow_motion:
            if not _confirm_motion():
                say("  İptal edildi. Robot hareket ettirilmedi.")
                return 1
            motion = MotionGuard(monitor)
        else:
            say("  SALT ÖLÇÜM modu: robot hareket ettirilmeyecek, "
                "hareketi siz yapacaksınız.")
        meta = make_meta("mecanum", args)
        dirs = args.directions or [d.key for d in N.MECANUM_DIRECTIONS]
        with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
            results = []
            guard = motion if motion is not None else _NullCtx()
            with guard:
                for key in dirs:
                    r = measure_direction(
                        monitor, store, key, on_status=say, motion=motion,
                        speed=args.speed, duration_s=args.duration)
                    results.append(r)
                    store.sample("mecanum", {k: ("" if v is None else v)
                                             for k, v in r.items()})
            store.set_metrics(
                mecanum_directions=dirs,
                mecanum_results=results,
                motion_mode="auto" if motion is not None else "observe_only",
                mecanum_all_correct=all(r["direction_verdict"] == "DOGRU"
                                        for r in results) if results else None,
            )
            for r in results:
                store.set_metric(f"{r['direction']}_verdict", r["direction_verdict"])
                store.set_metric(f"{r['direction']}_primary",
                                 r["primary_component_m_or_deg"])
        say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


class _NullCtx:
    def __enter__(self): return self
    def __exit__(self, *a): return False


def cmd_mode_switch(args) -> int:
    from .experiments.mode_switch import run_mode_switch
    rclpy, monitor = _ros_init(args, {"mode", "wheels"})
    try:
        meta = make_meta("mode-switch", args)
        with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
            m = run_mode_switch(monitor, store, repetitions=args.repetitions,
                                prompt=_ask, on_status=say)
            store.set_metrics(**m)
        say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


def cmd_mapping(args) -> int:
    from .experiments.mapping import run_mapping
    rclpy, monitor = _ros_init(args, {"map", "pose", "wheels", "mode"})
    try:
        meta = make_meta("mapping", args)
        with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
            m = run_mapping(monitor, store, duration_s=args.duration,
                            on_status=say)
            store.set_metrics(**m)
        say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


def cmd_camera(args) -> int:
    from .experiments.camera import run_camera
    rclpy, monitor = _ros_init(args, {"camera"})
    try:
        meta = make_meta("camera", args)
        with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
            m = run_camera(monitor, store, duration_s=args.duration,
                           on_status=say)
            store.set_metrics(**m)
        say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


def cmd_battery(args) -> int:
    from .experiments.battery import run_battery
    rclpy, monitor = _ros_init(args, {"mode", "wheels"})
    try:
        meta = make_meta("battery", args)
        with RunStore(meta) as store, BagRecorder(store.dir, args.record_bag):
            m = run_battery(monitor, store,
                            duration_s=args.duration if args.duration > 0 else None,
                            start_value=args.start, end_value=args.end,
                            unit=args.unit, prompt=_ask, on_status=say)
            store.set_metrics(**m)
        say(f"  kaydedildi -> {results_dir() / meta.experiment_id}")
    finally:
        _ros_shutdown(rclpy, monitor)
    return 0


def cmd_mark(args) -> int:
    """Kaydedilmiş bir koşuya operatör gözlemi ekler (çarpışma, başarısız, not).

    Arka planda/TTY'siz çalıştırılan testlerde operatör sorusu sorulamaz.
    Bu komut aynı bilgiyi koşu bittikten SONRA, izlenebilir biçimde yazar:
    metrics.json güncellenir, events.jsonl'a operator_mark olayı eklenir ve
    run.csv yeniden üretilir. Değerin operatörden geldiği
    collision_source alanıyla kayda geçer.
    """
    import json
    from .store import results_dir as _rd, _atomic_json

    base = Path(args.results) if args.results else _rd()
    target = base / args.experiment_id
    if not target.is_dir():
        matches = sorted(base.glob(f"*{args.experiment_id}*"))
        if len(matches) != 1:
            say(f"HATA: koşu bulunamadı veya birden fazla eşleşti: {args.experiment_id}")
            return 2
        target = matches[0]

    mpath = target / "metrics.json"
    metrics = json.loads(mpath.read_text(encoding="utf-8")) if mpath.is_file() else {}
    changed = {}
    if args.collision is not None:
        metrics["operator_collision_marked"] = args.collision
        metrics["collision_source"] = "operator_mark"
        changed["operator_collision_marked"] = args.collision
    if args.failed:
        metrics["operator_marked_failed"] = True
        changed["operator_marked_failed"] = True
    if args.note:
        prev = metrics.get("operator_note_extra") or ""
        metrics["operator_note_extra"] = (prev + " | " + args.note).strip(" |")
        changed["operator_note_extra"] = metrics["operator_note_extra"]
    if not changed:
        say("Değişiklik verilmedi (--collision / --failed / --note kullanın).")
        return 1

    _atomic_json(mpath, metrics)
    with open(target / "events.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps({"kind": "operator_mark", **changed},
                            ensure_ascii=False) + "\n")
    # run.csv'yi metrics + meta'dan yeniden üret
    import csv as _csv
    meta = json.loads((target / "meta.json").read_text(encoding="utf-8"))
    flat = dict(meta)
    for k, v in metrics.items():
        flat[k] = (json.dumps(v, ensure_ascii=False)
                   if isinstance(v, (dict, list, tuple)) else v)
    with open(target / "run.csv", "w", newline="", encoding="utf-8") as fh:
        w = _csv.DictWriter(fh, fieldnames=list(flat.keys()))
        w.writeheader()
        w.writerow({k: ("" if v is None else v) for k, v in flat.items()})
    say(f"  işaretlendi -> {target.name}: {changed}")
    return 0


def cmd_report(args) -> int:
    from .report import aggregate, plots, tables
    base = Path(args.results) if args.results else results_dir()
    if args.mock:
        from .mock import generate
        say(f"  Sahte veri üretiliyor -> {base}")
        made = generate(base, seed=args.seed)
        say(f"  {len(made)} sahte koşu üretildi (meta.mock=True).")
    out = Path(args.out) if args.out else outputs_dir()
    out.mkdir(parents=True, exist_ok=True)
    say(f"  Koşular okunuyor: {base}")
    p1 = aggregate.write_summary(out, base=base)
    p2 = tables.write_tables(out, base=base)
    p3 = plots.generate_all(out, base=base)
    say("\n=== ÜRETİLEN DOSYALAR ===")
    for _name, v in list(p1.items()) + list(p2.items()):
        say(f"  {Path(v)}")
    for c in p3["created"]:
        say(f"  {c}")
    for s in p3["skipped"]:
        say(f"  ATLANDI: {s}")
    say(f"\n  Toplu çıktı dizini: {out}")
    return 0


def cmd_tui(args) -> int:
    from .tui import run_menu
    return run_menu(args)


# ── argüman ayrıştırma ────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="robot_benchmark",
        description="ecza-robotu gerçek ortam performans testi ve rapor üretimi. "
                    "Varsayılan davranış SALT GÖZLEMDİR; robot kendiliğinden "
                    "hareket etmez.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp, with_goal: bool = True) -> None:
        sp.add_argument("--environment", "-e", default="",
                        help="ortam adı (ör. '3. kat koridor')")
        if with_goal:
            sp.add_argument("--goal", "-g", default="",
                            help="hedef adı (ör. 'H1-kapi')")
        sp.add_argument("--note", "-n", default="", help="operatör notu")
        sp.add_argument("--record-bag", action="store_true",
                        help="koşu boyunca rosbag2 kaydı al")
        sp.add_argument("--use-sim-time", action="store_true",
                        help="simülasyon saatini kullan")

    for kind in ("navigation", "obstacle", "repeatability"):
        sp = sub.add_parser(kind, help=f"{kind} testi (RViz'den hedef verin)")
        common(sp)
        sp.add_argument("--repetitions", "-r", type=int, default=1,
                        help="tekrar sayısı")
        sp.add_argument("--wait-goal", type=float, default=300.0,
                        help="hedef bekleme zaman aşımı (s)")
        sp.add_argument("--goal-timeout", type=float, default=300.0,
                        help="hedefin sonuçlanma zaman aşımı (s)")

    sp = sub.add_parser("mecanum", help="mecanum yön testi (varsayılan: salt ölçüm)")
    common(sp)
    sp.add_argument("--allow-motion", action="store_true",
                    help="GÜVENLİK BAYRAĞI: benchmark'ın robotu hareket "
                         "ettirmesine izin ver (onay istenir)")
    sp.add_argument("--speed", type=float, default=0.12, help="m/s")
    sp.add_argument("--duration", type=float, default=1.5, help="yön başına s")
    sp.add_argument("--directions", nargs="*",
                    choices=[d.key for d in N.MECANUM_DIRECTIONS],
                    help="test edilecek yönler (varsayılan: hepsi)")

    sp = sub.add_parser("mode-switch", help="TELEOP/AUTONOMOUS geçiş testi")
    common(sp)
    sp.add_argument("--repetitions", "-r", type=int, default=10)

    sp = sub.add_parser("mapping", help="haritalama testi")
    common(sp)
    sp.add_argument("--duration", type=float, default=120.0, help="s")

    sp = sub.add_parser("camera", help="kamera akış testi")
    common(sp)
    sp.add_argument("--duration", type=float, default=30.0, help="s")

    sp = sub.add_parser("battery", help="çalışma süresi / batarya testi")
    common(sp)
    sp.add_argument("--duration", type=float, default=0.0,
                    help="s (0 = Enter'a basana kadar bekle)")
    sp.add_argument("--start", type=float, default=None, help="başlangıç değeri")
    sp.add_argument("--end", type=float, default=None, help="bitiş değeri")
    sp.add_argument("--unit", default="V", help="birim (V, %%, ...)")

    sp = sub.add_parser("report", help="toplu rapor üret (CSV/JSON/PNG/MD/TeX)")
    sp.add_argument("--results", default=None, help="koşu dizini")
    sp.add_argument("--out", default=None, help="çıktı dizini")
    sp.add_argument("--mock", action="store_true",
                    help="önce sahte veri üret (gerçek robot gerekmez)")
    sp.add_argument("--seed", type=int, default=7)

    sp = sub.add_parser("mark", help="kaydedilmiş koşuya operatör gözlemi ekle")
    sp.add_argument("experiment_id", help="koşu kimliği (kısmi eşleşme olur)")
    sp.add_argument("--collision", dest="collision", action="store_true",
                    default=None, help="çarpışma OLDU olarak işaretle")
    sp.add_argument("--no-collision", dest="collision", action="store_false",
                    help="çarpışma OLMADI olarak işaretle")
    sp.add_argument("--failed", action="store_true",
                    help="operatör bu koşuyu başarısız sayıyor")
    sp.add_argument("--note", default="", help="ek not")
    sp.add_argument("--results", default=None, help="koşu dizini")

    sp = sub.add_parser("menu", help="terminal menüsü (canlı gösterge)")
    sp.add_argument("--environment", "-e", default="")
    sp.add_argument("--use-sim-time", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    try:
        return _dispatch(build_parser().parse_args(argv))
    except KeyboardInterrupt:
        # Veri RunStore tarafından zaten partial olarak kaydedildi.
        say("\n  Kesildi. O ana kadarki veri kaydedildi (partial=true).")
        return 130


def _dispatch(args) -> int:
    cmd = args.command
    if cmd in ("navigation", "obstacle", "repeatability"):
        return cmd_goal_based(args, cmd)
    if cmd == "mecanum":
        return cmd_mecanum(args)
    if cmd == "mode-switch":
        return cmd_mode_switch(args)
    if cmd == "mapping":
        return cmd_mapping(args)
    if cmd == "camera":
        return cmd_camera(args)
    if cmd == "battery":
        return cmd_battery(args)
    if cmd == "mark":
        return cmd_mark(args)
    if cmd == "report":
        return cmd_report(args)
    if cmd == "menu":
        return cmd_tui(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
