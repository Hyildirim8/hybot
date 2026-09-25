"""Mecanum hareket testi.

GÜVENLİK — varsayılan davranış SALT ÖLÇÜMDÜR. Benchmark robotu kendiliğinden
hareket ETTİRMEZ: operatör joystick ile hareketi yapar, biz başlangıç/bitiş
pozundan mesafe ve sapmayı ölçeriz.

--allow-motion verilirse benchmark komut yayınlayabilir. O zaman bile:
  * açık operatör onayı (evet yazılmalı) istenir,
  * komut /cmd_vel_nav_smoothed'e gönderilir — teleop_node'un AUTO çarpışma
    kapısından (_auto_scan_safe) GEÇER, doğrudan kontrolcüye yazılmaz,
  * her yön öncesi lidar ile o yönde boşluk kontrol edilir,
  * firmware teker limiti (18.75 rad/s) aşılırsa komut reddedilir,
  * Ctrl+C ve her çıkış yolunda sıfır hız yayınlanır (acil durdurma).
"""

from __future__ import annotations

import math
from typing import Any, Callable

from geometry_msgs.msg import Twist

from .. import ros_names as N
from ..monitors import BenchmarkMonitor
from ..stats import angle_diff_deg

# Lidar gövdeye 180° ters monte; /scan açıları gövde çerçevesine bu kadar
# döndürülerek çevrilir (config/rover_params.yaml lidar_angle_offset_deg).
LIDAR_OFFSET_RAD = math.pi

# Yön -> (vx, vy, wz işaretleri) ve güvenlik için bakılacak gövde açısı (derece)
DIRECTION_CMD = {
    "ileri": ((1.0, 0.0, 0.0), 0.0),
    "geri": ((-1.0, 0.0, 0.0), 180.0),
    "sol": ((0.0, 1.0, 0.0), 90.0),
    "sag": ((0.0, -1.0, 0.0), -90.0),
    "ccw": ((0.0, 0.0, 1.0), None),   # dönüş: tek yön yok, tüm çevre bakılır
    "cw": ((0.0, 0.0, -1.0), None),
}


def sector_clearance(monitor: BenchmarkMonitor, center_deg: float | None,
                     half_deg: float = 30.0) -> float | None:
    """Gövde çerçevesinde verilen koninin en yakın geçerli lidar mesafesi.

    center_deg None ise tüm çevrenin en yakını döner (dönüş testi için).
    Hiç geçerli ışın yoksa None (ölçülemedi) — 'sonsuz boşluk' varsayılmaz.
    """
    msg = monitor.scan
    if msg is None:
        return None
    lo = max(0.0, msg.range_min)
    best = None
    half = math.radians(half_deg)
    ctr = None if center_deg is None else math.radians(center_deg)
    for i, d in enumerate(msg.ranges):
        if not math.isfinite(d) or not (lo <= d < msg.range_max):
            continue
        ang = msg.angle_min + i * msg.angle_increment
        body = math.atan2(math.sin(ang - LIDAR_OFFSET_RAD),
                          math.cos(ang - LIDAR_OFFSET_RAD))
        if ctr is not None:
            delta = abs(math.atan2(math.sin(body - ctr), math.cos(body - ctr)))
            if delta > half:
                continue
        if best is None or d < best:
            best = float(d)
    return best


class MotionGuard:
    """--allow-motion açıkken hız yayınlar; her yolda sıfırlar."""

    def __init__(self, monitor: BenchmarkMonitor):
        self.monitor = monitor
        self.pub = monitor.create_publisher(
            Twist, N.TOPIC_CMD_VEL_NAV_SMOOTHED, 10)
        self.published_any = False

    def stop(self, seconds: float = 1.5) -> None:
        """Acil durdurma: sıfır hız akıt."""
        zero = Twist()
        end = self.monitor.now_s() + seconds
        while self.monitor.now_s() < end:
            self.pub.publish(zero)
            self.monitor.spin_for(0.05)

    def drive(self, vx: float, vy: float, wz: float, seconds: float,
              abort: Callable[[], bool] | None = None) -> bool:
        """Belirtilen hızı `seconds` boyunca yayınlar. abort() True olursa keser."""
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        end = self.monitor.now_s() + seconds
        self.published_any = True
        while self.monitor.now_s() < end:
            if abort is not None and abort():
                self.stop()
                return False
            self.pub.publish(cmd)
            self.monitor.spin_for(0.05)
        self.stop()
        return True

    def __enter__(self) -> "MotionGuard":
        return self

    def __exit__(self, *exc) -> bool:
        # İstisna/kesinti olsa da MUTLAKA durdur.
        if self.published_any:
            try:
                self.stop()
            except Exception:
                pass
        return False


def wheels_still_for(monitor: BenchmarkMonitor, seconds: float,
                     timeout_s: float) -> bool:
    """Tekerlekler KESINTISIZ `seconds` boyunca durdu mu?

    Tek bir anlik sifir okumasina guvenmek olcumu bozuyordu: tekerlek geri
    beslemesi hareket ortasinda bir an sifir okuyunca "hareket bitti" saniliyor,
    bir sonraki yon dilimi hala suren hareketi yakaliyor ve butun sira bir dilim
    kayiyordu (olcum: 0.4 s'lik pencerede 56.8 cm, yani 1.4 m/s — fiziksel
    tavan ~0.75 m/s). Bu yuzden duruş DEBOUNCE ediliyor.
    """
    end = monitor.now_s() + timeout_s
    still_since: float | None = None
    while monitor.now_s() < end:
        if monitor.wheels_still():
            if still_since is None:
                still_since = monitor.now_s()
            elif monitor.now_s() - still_since >= seconds:
                return True
        else:
            still_since = None
        monitor.spin_for(0.05)
    return False


def measure_direction(monitor: BenchmarkMonitor, store, direction_key: str,
                      settle_s: float = 1.0,
                      on_status: Callable[[str], None] | None = None,
                      wait_for_motion_s: float = 60.0,
                      motion: MotionGuard | None = None,
                      speed: float = 0.12, duration_s: float = 1.5,
                      still_debounce_s: float = 1.5,
                      ) -> dict[str, Any]:
    """Bir yön için başlangıç/bitiş pozundan mesafe ve sapma ölçer.

    motion None ise SALT ÖLÇÜM: operatör hareketi yapar, biz tekerleklerin
    dönmeye başlamasını ve durmasını tespit ederiz.
    """
    def say(m: str) -> None:
        if on_status:
            on_status(m)
        store.event("status", direction=direction_key, message=m)

    (sx, sy, sw), safe_deg = DIRECTION_CMD[direction_key]
    clearance = sector_clearance(monitor, safe_deg)

    # Başlangıç pozu — robot GERÇEKTEN ve KESINTISIZ dururken alınmalı.
    if not wheels_still_for(monitor, still_debounce_s, 20.0):
        say(f"[{direction_key}] robot durmuyor, ölçüm atlandı "
            f"(önceki hareket bitmemiş olabilir).")
        return _na(direction_key, clearance, "durus_yok")
    monitor.spin_for(settle_s)
    p0 = monitor.odom_pose
    if p0 is None:
        say("Odometri yok, ölçüm yapılamadı.")
        return _na(direction_key, clearance, "odometri_yok")

    moved_ok = True
    motion_window_s: float | None = None
    if motion is None:
        say(f"[{direction_key}] Hareketi ŞİMDİ joystick ile yapın "
            f"(en fazla {wait_for_motion_s:.0f} s bekliyorum)…")
        started = monitor.wait_until(
            lambda: not monitor.wheels_still(), wait_for_motion_s)
        if not started:
            say("Hareket algılanmadı.")
            return _na(direction_key, clearance, "hareket_yok")
        t_start = monitor.now_s()
        store.event("motion_started", direction=direction_key, t_ros_s=t_start)
        # Hareket bitene kadar bekle: tekerlekler KESINTISIZ durmali, yoksa
        # hareket ortasindaki anlik sifir okumasi pencereyi erken kapatiyor.
        if not wheels_still_for(monitor, still_debounce_s, wait_for_motion_s):
            say(f"[{direction_key}] hareket {wait_for_motion_s:.0f} s içinde "
                f"bitmedi.")
            return _na(direction_key, clearance, "hareket_bitmedi")
        motion_window_s = monitor.now_s() - t_start
        store.event("motion_stopped", direction=direction_key,
                    t_ros_s=monitor.now_s(),
                    motion_window_s=round(motion_window_s, 2))
    else:
        vx, vy, wz = sx * speed, sy * speed, sw * (speed / 0.12 * 0.4)
        wheel = N.max_wheel_rad_s(vx, vy, wz)
        if wheel > N.FIRMWARE_MAX_WHEEL_RAD_S:
            say(f"Komut reddedildi: teker {wheel:.1f} rad/s > "
                f"{N.FIRMWARE_MAX_WHEEL_RAD_S} limit.")
            return _na(direction_key, clearance, "limit_asildi")
        need = abs(speed) * duration_s + 0.25
        if safe_deg is not None and clearance is not None and clearance < need:
            say(f"Komut reddedildi: {direction_key} yönünde boşluk "
                f"{clearance:.2f} m < {need:.2f} m.")
            return _na(direction_key, clearance, "bosluk_yetersiz")
        say(f"[{direction_key}] otomatik hareket: "
            f"vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}, {duration_s:.1f} s")
        store.event("auto_motion", direction=direction_key,
                    vx=vx, vy=vy, wz=wz, duration_s=duration_s)
        moved_ok = motion.drive(vx, vy, wz, duration_s)

    wheels_still_for(monitor, still_debounce_s, 10.0)
    monitor.spin_for(settle_s)
    p1 = monitor.odom_pose
    if p1 is None:
        return _na(direction_key, clearance, "odometri_kayboldu")

    dx, dy = p1.x - p0.x, p1.y - p0.y
    # Gövde çerçevesine çevir (başlangıç yaw'ına göre)
    fwd = dx * math.cos(p0.yaw) + dy * math.sin(p0.yaw)
    left = -dx * math.sin(p0.yaw) + dy * math.cos(p0.yaw)
    yaw_deg = angle_diff_deg(p1.yaw, p0.yaw)
    dist = math.hypot(dx, dy)

    expect = next(d.expect for d in N.MECANUM_DIRECTIONS if d.key == direction_key)
    primary, lateral_dev, verdict = _classify(expect, fwd, left, yaw_deg)

    # Fiziksel makuliyet: olculen yer degistirme, hareket penceresinde
    # ulasilabilir hizin ustundeyse pencere gercek hareketle ORTUSMUYOR
    # demektir (dilim kaymasi). Boyle bir olcumu "yon dogru/ters" diye
    # raporlamak yaniltici olur.
    max_lin_mps = N.FIRMWARE_MAX_WHEEL_RAD_S * N.WHEEL_RADIUS_M  # 0.75 m/s
    window = max(motion_window_s or 0.0, 1e-6)
    if motion_window_s and dist / window > max_lin_mps * 1.15:
        verdict = "OLCUM_TUTARSIZ"

    say(f"[{direction_key}] ileri={fwd*100:+.1f} cm  sol={left*100:+.1f} cm  "
        f"dönme={yaw_deg:+.1f}°  -> {verdict}")
    return {
        "direction": direction_key,
        "clearance_m": clearance,
        "linear_distance_m": dist,
        "forward_component_m": fwd,
        "lateral_component_m": left,
        "angular_change_deg": yaw_deg,
        "primary_component_m_or_deg": primary,
        "lateral_deviation_m_or_deg": lateral_dev,
        "motion_window_s": motion_window_s,
        "direction_verdict": verdict,
        "auto_motion_completed": moved_ok if motion is not None else None,
        "skip_reason": None,
    }


def _classify(expect: str, fwd: float, left: float,
              yaw_deg: float) -> tuple[float, float, str]:
    """Beklenen eksene göre baskın bileşen, sapma ve karar."""
    if expect in ("ileri", "geri"):
        primary, dev = fwd, abs(left)
        ok = (fwd > 0) if expect == "ileri" else (fwd < 0)
        thresh = 0.02
    elif expect in ("sol", "sag"):
        primary, dev = left, abs(fwd)
        ok = (left > 0) if expect == "sol" else (left < 0)
        thresh = 0.02
    else:  # ccw / cw
        primary, dev = yaw_deg, math.hypot(fwd, left)
        ok = (yaw_deg > 0) if expect == "ccw" else (yaw_deg < 0)
        thresh = 2.0
    if abs(primary) < thresh:
        return primary, dev, "HAREKET_YOK"
    return primary, dev, "DOGRU" if ok else "TERS"


def _na(direction_key: str, clearance: float | None, reason: str) -> dict[str, Any]:
    """Ölçülemeyen yön: her metrik None, sebep kayıtlı. 0 ile doldurulmaz."""
    return {
        "direction": direction_key, "clearance_m": clearance,
        "linear_distance_m": None, "forward_component_m": None,
        "lateral_component_m": None, "angular_change_deg": None,
        "primary_component_m_or_deg": None, "lateral_deviation_m_or_deg": None,
        "direction_verdict": "OLCULEMEDI", "auto_motion_completed": None,
        "skip_reason": reason,
    }
