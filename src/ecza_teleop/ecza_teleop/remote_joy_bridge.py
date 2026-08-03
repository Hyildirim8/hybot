#!/usr/bin/env python3
"""remote_joy_bridge.py — TCP → /joy bridge for remote (off-Pi) joystick control.

Lets a joystick plugged into a REMOTE machine (not the RPi5) drive the rover,
without touching the existing local setup (joy_linux + teleop_node) at all.
This node just republishes whatever it receives as sensor_msgs/Joy on /joy —
teleop_node.py can't tell the difference between this and a local F710, so
every existing behaviour (dead-man, strafe_invert, pivot mapping, scan
safety, AUTO/TELEOP handoff) keeps working unmodified.

Wire protocol: newline-delimited JSON over a plain TCP socket (no ROS2 or
extra pip packages needed on the remote client — just Python's stdlib):
  {"axes": [f, f, f, f, f, f], "buttons": [0/1, 0/1, ...]}\n

Only one client at a time is meaningful (last connection wins); a dropped
connection publishes one zeroed Joy message so the rover doesn't run away,
then waits for a new connection. This mirrors joy_linux's own behaviour when
a USB joystick is unplugged.

Parameters:
  port       int   9092        TCP port to listen on
  frame_id   str   'joy'       Joy message frame_id (cosmetic)
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy


class RemoteJoyBridge(Node):

    def __init__(self):
        super().__init__('remote_joy_bridge')

        self.declare_parameter('port', 9092)
        self.declare_parameter('frame_id', 'joy')

        self._port = int(self.get_parameter('port').value)
        self._frame_id = self.get_parameter('frame_id').value

        # Match joy_linux's actual QoS (RELIABLE) — teleop_node and
        # slam_manager both subscribe RELIABLE, so a BEST_EFFORT publisher
        # here is silently dropped by DDS (verified live 2026-08-01).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(Joy, 'joy', qos)

        self._running = True
        self._server_thread = threading.Thread(target=self._serve, daemon=True)
        self._server_thread.start()

        self.get_logger().info(f'remote_joy_bridge listening on tcp://0.0.0.0:{self._port}')

    def _serve(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self._port))
        srv.listen(1)
        srv.settimeout(1.0)

        while self._running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f'remote joystick connected: {addr[0]}:{addr[1]}')
            try:
                self._handle_client(conn)
            except (ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f'remote joystick connection lost: {e}')
            finally:
                conn.close()
                self._publish_zero()
                self.get_logger().info('remote joystick disconnected — publishing zero Joy')

        srv.close()

    def _handle_client(self, conn: socket.socket) -> None:
        conn.settimeout(1.0)
        buf = b''
        last_msg_time = time.monotonic()
        while self._running:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                # No data for 1s — treat as a dead link, not just a quiet stick.
                if time.monotonic() - last_msg_time > 1.0:
                    raise ConnectionResetError('no data for 1s (watchdog)')
                continue
            if not chunk:
                raise ConnectionResetError('remote end closed the socket')

            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                    self._publish(msg['axes'], msg['buttons'])
                    last_msg_time = time.monotonic()
                except (json.JSONDecodeError, KeyError, TypeError) as e:
                    self.get_logger().warn(
                        f'bad packet from remote joystick: {e}', throttle_duration_sec=5.0
                    )

    def _publish(self, axes, buttons) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self._pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish([0.0] * 6, [0] * 12)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RemoteJoyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
