#!/usr/bin/env python3
"""remote_teleop_client.py — run this on your OWN PC (not the RPi5).

Reads a joystick plugged into THIS machine and drives the ecza-robotu rover
over the network, while showing its camera feed in a window. Talks to the
`remote_teleop` and `csi_camera` services already running on the Pi — no
changes needed on the Pi's local joystick setup, this is a fully separate,
additive control path (see docker-compose.yaml `remote_teleop` service).

Requirements (install once):
    pip3 install pygame opencv-python numpy

Usage:
    python3 remote_teleop_client.py <rpi-ip> [--joy-port 9092] \
        [--cam-host <camera-ip>] [--cam-port 8081]

Example:
    python3 remote_teleop_client.py 10.42.101.197

    # camera streamed from a separate device (e.g. a Raspberry Pi Zero)
    # instead of the RPi5 itself — joystick control still goes to the RPi5:
    python3 remote_teleop_client.py 10.42.101.197 --cam-host 10.42.101.xxx

Controls: identical to the Pi's local F710 mapping (see rover_params.yaml)
— left stick = strafe/forward, right stick = rotate, LT/RT = side pivot,
D-pad = axle pivot, Start = TELEOP/AUTO toggle, Y = save map, A = explore.

Window: press ESC or close the window to quit. If the joystick link drops,
the rover receives a zero command automatically (same watchdog behaviour as
the local setup) — this script does not need to handle that itself.

Note on axis/button numbering: verified live 2026-08-01 with joystick_probe.py
(a separate, simpler raw-input dump script in this same folder) that pygame's
D-pad (hat) and all 12 buttons already match joy_linux's numbering exactly
for this F710, needing no correction — but axes 0/1/2 (strafe, forward/back,
rotation) come out sign-INVERTED from what joy_linux reports for the same
stick position, on this PC's pygame/SDL build. The default behaviour here
already corrects that (see the `axes[0] = -axes[0]` block in `_stream`). If
a DIFFERENT joystick/PC needs a different fix, `--calibrate` records one
interactively (no numbers to read — it just asks you to move each control
in turn) instead of editing this flip by hand; `--flip-axes` inverts a
--calibrate mapping wholesale if it comes out backwards; `--raw` forces the
verified F710 default even if a --calibrate file exists.
"""

import argparse
import json
import os
import socket
import sys
import threading
import time

import cv2
import numpy as np
import pygame

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'

MAPPING_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'remote_teleop_mapping.json')

# rover_params.yaml's documented F710 layout — what index 0..5 / 0..11 below
# MEANS on the Pi side, regardless of how this PC's pygame numbers things.
AXIS_ROLES = ['sol stick SAĞA', 'sol stick YUKARI', 'sağ stick SAĞA', 'sağ stick YUKARI']
BUTTON_ROLES = ['X', 'A', 'B', 'Y', 'LB', 'RB', 'LT', 'RT', 'Back', 'Start', 'L3', 'R3']


def _sample_axes_peak(joy, duration=2.0):
    pygame.event.pump()
    peak = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        pygame.event.pump()
        for i in range(joy.get_numaxes()):
            v = joy.get_axis(i)
            if abs(v) > abs(peak[i]):
                peak[i] = v
        time.sleep(0.02)
    return peak


def _wait_for_button(joy, timeout=5.0):
    pygame.event.pump()
    baseline = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pygame.event.pump()
        for i in range(joy.get_numbuttons()):
            if joy.get_button(i) and not baseline[i]:
                return i
        time.sleep(0.02)
    return None


def run_calibration_wizard(joy) -> dict:
    """Interactive, no-typing-required mapping: for each control the Pi
    expects at a fixed index, ask the user to move/press it, and record
    whichever pygame axis/button actually moved. Saves the result so normal
    runs don't need to repeat this."""
    print()
    print('=== Kalibrasyon sihirbazı ===')
    print('Her adımda istenen kolu/tuşu birkaç saniye basılı tutun / itin.')
    print()

    axes_map = []
    for role in AXIS_ROLES:
        input(f'  -> {role} itin, basılı tutun, sonra ENTER\'a basın...')
        peak = _sample_axes_peak(joy)
        idx = max(range(len(peak)), key=lambda i: abs(peak[i]))
        sign = 1.0 if peak[idx] >= 0 else -1.0
        axes_map.append({'axis': idx, 'sign': sign})
        print(f'     tespit edildi: pygame axis {idx} (işaret {sign:+.0f})')

    if joy.get_numhats() > 0:
        print('  -> D-pad bir "hat" olarak algılandı, ayrıca kalibrasyon gerekmiyor.')
        axes_map.append({'hat': 0, 'component': 'x'})
        axes_map.append({'hat': 0, 'component': 'y'})
    else:
        for role, comp in [('D-pad SAĞA', 'x'), ('D-pad YUKARI', 'y')]:
            input(f'  -> {role} basın, basılı tutun, sonra ENTER\'a basın...')
            peak = _sample_axes_peak(joy)
            idx = max(range(len(peak)), key=lambda i: abs(peak[i]))
            sign = 1.0 if peak[idx] >= 0 else -1.0
            axes_map.append({'axis': idx, 'sign': sign})
            print(f'     tespit edildi: pygame axis {idx} (işaret {sign:+.0f})')

    print()
    buttons_map = []
    for role in BUTTON_ROLES:
        print(f'  -> {role} tuşuna basın (5 saniye içinde, atlamak için bekleyin)...')
        idx = _wait_for_button(joy, timeout=5.0)
        buttons_map.append(idx)
        print(f'     tespit edildi: pygame button {idx}' if idx is not None
              else '     algılanamadı, bu tuş boş geçildi')

    mapping = {'axes': axes_map, 'buttons': buttons_map}
    with open(MAPPING_FILE, 'w') as f:
        json.dump(mapping, f, indent=2)
    print()
    print(f'Kalibrasyon tamamlandı, kaydedildi: {MAPPING_FILE}')
    print('Artık normal çalıştırmalarda bu dosya otomatik kullanılacak.')
    print()
    return mapping


def load_mapping():
    if not os.path.exists(MAPPING_FILE):
        return None
    with open(MAPPING_FILE) as f:
        return json.load(f)


def flip_mapping(mapping: dict) -> dict:
    """Negate every axis's sign. The calibration wizard assumes 'push in the
    prompted direction' should read out as positive (matching rover_params.
    yaml's documented raw-axis convention) — if that documented convention
    doesn't actually hold for a given joystick/driver, every axis comes out
    inverted TOGETHER (not scrambled independently), so a single global flip
    fixes it without re-running the whole wizard."""
    flipped = {'axes': [], 'buttons': mapping['buttons']}
    for entry in mapping['axes']:
        e = dict(entry)
        if 'sign' in e:
            e['sign'] = -e['sign']
        else:  # hat-based D-pad entry
            e['sign'] = -1.0
        flipped['axes'].append(e)
    return flipped


class JoystickSender:
    """Reads the local joystick and streams it to the Pi's remote_teleop_bridge."""

    def __init__(self, host: str, port: int, rate_hz: float = 30.0, debug: bool = False,
                 mapping: dict = None):
        self._host = host
        self._port = port
        self._period = 1.0 / rate_hz
        self._running = True
        self._connected = False
        self._joy = None
        self._debug = debug
        self._mapping = mapping
        self._last_rescan = 0.0
        self._last_debug_print = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print('[joystick] No joystick detected yet — will keep checking '
                  '(plug it in any time, no need to restart this script).')
        else:
            self._acquire_joystick()

        while self._running:
            sock = None
            try:
                sock = socket.create_connection((self._host, self._port), timeout=5.0)
                sock.settimeout(2.0)
                print(f'[joystick] connected to {self._host}:{self._port}')
                self._connected = True
                self._stream(sock)
            except (ConnectionRefusedError, OSError, socket.timeout) as e:
                self._connected = False
                print(f'[joystick] link to Pi unavailable ({e}); retrying in 2s...')
                time.sleep(2.0)
            finally:
                if sock is not None:
                    sock.close()
                self._connected = False

    def _apply_mapping(self):
        pygame.event.pump()
        axes = []
        for entry in self._mapping['axes']:
            if 'hat' in entry:
                hx, hy = self._joy.get_hat(entry['hat'])
                v = hx if entry['component'] == 'x' else hy
                axes.append(float(v) * entry.get('sign', 1.0))
            else:
                axes.append(self._joy.get_axis(entry['axis']) * entry['sign'])
        buttons = []
        for idx in self._mapping['buttons']:
            buttons.append(self._joy.get_button(idx) if idx is not None else 0)
        return axes, buttons

    def _acquire_joystick(self) -> None:
        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        print(f'[joystick] Using: {self._joy.get_name()} '
              f'({self._joy.get_numaxes()} axes, {self._joy.get_numbuttons()} buttons, '
              f'{self._joy.get_numhats()} hats)')

    def _rescan_if_needed(self) -> None:
        # Hotplug (JOYDEVICEADDED) events aren't reliably delivered on every
        # SDL/driver combination — fall back to periodically re-enumerating
        # from scratch so a joystick plugged in after startup is still
        # picked up even if the event never arrives.
        now = time.monotonic()
        if self._joy is not None or now - self._last_rescan < 2.0:
            return
        self._last_rescan = now
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self._acquire_joystick()

    def _stream(self, sock: socket.socket) -> None:
        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED and self._joy is None:
                    self._acquire_joystick()
                elif event.type == pygame.JOYDEVICEREMOVED:
                    print('[joystick] unplugged — publishing zero until reconnected')
                    self._joy = None
            self._rescan_if_needed()
            if self._joy is not None:
                if self._mapping is not None:
                    axes, buttons = self._apply_mapping()
                else:
                    axes = [self._joy.get_axis(i) for i in range(self._joy.get_numaxes())]
                    buttons = [self._joy.get_button(i) for i in range(self._joy.get_numbuttons())]
                    # joy_linux reports the D-pad as axes[4]/axes[5]; pygame
                    # reports it as a separate hat. Fold it in so the Pi
                    # side sees the same 6-axis layout rover_params.yaml
                    # expects.
                    hx, hy = self._joy.get_hat(0) if self._joy.get_numhats() > 0 else (0, 0)
                    while len(axes) < 4:
                        axes.append(0.0)
                    axes = axes[:4] + [float(hx), float(hy)]
                    # Verified live 2026-08-01 on this exact F710 (Logitech,
                    # D-mode) + this PC's pygame/SDL build: axes 0/1/2
                    # (strafe, forward/back, rotation) all come out with the
                    # OPPOSITE sign from what joy_linux reports for the same
                    # physical stick position, even in --raw mode — this
                    # isn't a --calibrate artifact, both modes agreed.
                    # D-pad (hat, axes 4/5) and all buttons were confirmed
                    # correct via joystick_probe.py, so only 0/1/2 flip here.
                    axes[0] = -axes[0]
                    axes[1] = -axes[1]
                    axes[2] = -axes[2]
            else:
                axes = [0.0] * 6
                buttons = [0] * 12

            packet = json.dumps({'axes': axes, 'buttons': buttons}) + '\n'
            sock.sendall(packet.encode('utf-8'))

            if self._debug:
                now = time.monotonic()
                if now - self._last_debug_print > 1.0:
                    self._last_debug_print = now
                    axes_str = ' '.join(f'{a:+.2f}' for a in axes)
                    btn_str = ''.join(str(b) for b in buttons)
                    print(f'[joystick] axes=[{axes_str}] buttons={btn_str}')

            time.sleep(self._period)


def _read_mjpeg_frame(sock: socket.socket, buf: bytearray, max_frame: int = 2_000_000):
    """Pulls one JPEG frame out of an MJPEG multipart HTTP stream (blocking)."""
    while True:
        if not buf.startswith(SOI):
            idx = buf.find(SOI)
            if idx < 0:
                if len(buf) > max_frame:
                    del buf[:-2]
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionResetError('camera stream closed')
                buf.extend(chunk)
                continue
            del buf[:idx]

        end = buf.find(EOI, 2)
        if end < 0:
            if len(buf) > max_frame:
                buf.clear()
            chunk = sock.recv(65536)
            if not chunk:
                raise ConnectionResetError('camera stream closed')
            buf.extend(chunk)
            continue

        frame = bytes(buf[:end + 2])
        del buf[:end + 2]
        return frame


def _video_loop(host: str, port: int, status_fn) -> None:
    window = 'ecza-robotu — remote teleop (ESC to quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    while True:
        try:
            sock = socket.create_connection((host, port), timeout=5.0)
            sock.sendall(
                f'GET /stream HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n'.encode()
            )
            # Drop the HTTP header before the first MJPEG boundary/frame.
            buf = bytearray()
            while SOI not in buf:
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionResetError('camera stream closed before first frame')
                buf.extend(chunk)
            print(f'[video] connected to {host}:{port}')

            while True:
                jpg = _read_mjpeg_frame(sock, buf)
                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.putText(frame, status_fn(), (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    return
        except (ConnectionRefusedError, OSError, socket.timeout, ConnectionResetError) as e:
            print(f'[video] camera link unavailable ({e}); retrying in 2s...')
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, 'camera unavailable, retrying...', (20, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow(window, blank)
            if cv2.waitKey(2000) & 0xFF == 27:
                return


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rpi_ip', help="RPi5's IP address (e.g. 10.42.101.197)")
    ap.add_argument('--joy-port', type=int, default=9092, help='remote_teleop TCP port (default 9092)')
    ap.add_argument('--cam-host', default=None,
                     help="camera source IP — set this if the camera is streamed from a "
                          "SEPARATE device (e.g. a Raspberry Pi Zero) instead of the RPi5 "
                          "itself. Defaults to rpi_ip (camera on the same machine as the "
                          "joystick bridge) when not given.")
    ap.add_argument('--cam-port', type=int, default=8081, help='camera HTTP port (default 8081)')
    ap.add_argument('--debug', action='store_true',
                     help='print axes/buttons once per second as they are sent')
    ap.add_argument('--calibrate', action='store_true',
                     help='run the interactive mapping wizard (no video, joystick must '
                          'already be plugged in) and exit — do this once, or again if '
                          'controls feel scrambled')
    ap.add_argument('--flip-axes', action='store_true',
                     help='invert every calibrated axis — use this if everything feels '
                          'exactly backwards (right/left, forward/back, rotation all '
                          'reversed together) after calibrating, instead of recalibrating')
    ap.add_argument('--raw', action='store_true',
                     help='ignore any saved --calibrate mapping file and use the built-in '
                          'default (pygame axes/hat/buttons straight through, with the '
                          'empirically-verified axes[0:3] sign flip applied) — this is the '
                          'recommended default for the F710; --calibrate is only for a '
                          'different joystick/driver where this default turns out wrong')
    args = ap.parse_args()

    if args.calibrate:
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print('Joystick bulunamadı. Önce takıp tekrar deneyin.')
            sys.exit(1)
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f'Kullanılan joystick: {joy.get_name()}')
        run_calibration_wizard(joy)
        pygame.quit()
        sys.exit(0)

    mapping = None if args.raw else load_mapping()
    if args.raw:
        print('[joystick] --raw: kayıtlı eşleme yok sayılıyor, ham pygame verisi gönderiliyor')
    elif mapping is not None:
        if args.flip_axes:
            mapping = flip_mapping(mapping)
            print(f'[joystick] kayıtlı eşleme kullanılıyor (TERSİNE ÇEVRİLDİ): {MAPPING_FILE}')
        else:
            print(f'[joystick] kayıtlı eşleme kullanılıyor: {MAPPING_FILE}')
    else:
        print('[joystick] kayıtlı eşleme yok — ham pygame sırası kullanılacak. '
              'Kontroller karışık geliyorsa: python3 remote_teleop_client.py '
              f'{args.rpi_ip} --calibrate')

    sender = JoystickSender(args.rpi_ip, args.joy_port, debug=args.debug, mapping=mapping)
    joy_thread = threading.Thread(target=sender.run, daemon=True)
    joy_thread.start()

    def status():
        return 'joystick: connected' if sender.connected else 'joystick: reconnecting...'

    cam_host = args.cam_host or args.rpi_ip
    if args.cam_host:
        print(f'[video] kamera kaynağı: {cam_host}:{args.cam_port} (rpi_ip\'den ayrı)')

    try:
        _video_loop(cam_host, args.cam_port, status)
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()
        cv2.destroyAllWindows()
        pygame.quit()
        sys.exit(0)


if __name__ == '__main__':
    main()
