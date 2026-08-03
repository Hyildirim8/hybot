#!/usr/bin/env python3
"""joystick_probe.py — raw joystick inspector, run this on YOUR PC.

No networking, no rover, no calibration logic — just prints what pygame
sees from the joystick, one line per change, so we can figure out the
remote_teleop_client.py axis/button mapping together by watching exactly
what moves when you touch each control.

Usage:
    pip3 install pygame   # if not already installed
    python3 joystick_probe.py

Then move ONE control at a time (a stick, the D-pad, a button, a trigger)
and watch the terminal — only the line(s) that changed print. Press Ctrl+C
to quit.
"""

import sys
import time

import pygame


def main() -> None:
    pygame.init()
    pygame.joystick.init()

    print('Joystick bekleniyor... (takılıysa hemen algılanır, değilse takın)')
    joy = None
    while joy is None:
        pygame.event.pump()
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
        else:
            time.sleep(0.5)

    print(f'\nJoystick: {joy.get_name()}')
    print(f'{joy.get_numaxes()} eksen (axis), {joy.get_numbuttons()} tuş (button), '
          f'{joy.get_numhats()} D-pad (hat)\n')
    print('Şimdi tek tek her koldaki/tuştaki hareketi deneyin — sadece değişen satır yazılır.')
    print('Çıkmak için Ctrl+C.\n')

    last_axes = [round(joy.get_axis(i), 2) for i in range(joy.get_numaxes())]
    last_buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    last_hats = [joy.get_hat(i) for i in range(joy.get_numhats())]

    try:
        while True:
            pygame.event.pump()

            for i in range(joy.get_numaxes()):
                v = round(joy.get_axis(i), 2)
                if abs(v - last_axes[i]) > 0.05:
                    print(f'AXIS  {i}: {last_axes[i]:+.2f} -> {v:+.2f}')
                    last_axes[i] = v

            for i in range(joy.get_numbuttons()):
                v = joy.get_button(i)
                if v != last_buttons[i]:
                    print(f'BUTTON {i}: {last_buttons[i]} -> {v}')
                    last_buttons[i] = v

            for i in range(joy.get_numhats()):
                v = joy.get_hat(i)
                if v != last_hats[i]:
                    print(f'HAT   {i}: {last_hats[i]} -> {v}')
                    last_hats[i] = v

            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        sys.exit(0)


if __name__ == '__main__':
    main()
