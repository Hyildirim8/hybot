#!/usr/bin/env python3
"""RPi Camera Module 2 (CSI, CAM0 port) receiver for ecza-robotu.

The RPi5's Ubuntu 22.04 ROS container cannot run rpicam-vid/libcamera
directly — the imx219 needs the PiSP ISP stack tied to this exact host's
libcamera build (Raspberry Pi OS / Debian Trixie), which does not exist as
a matching package in the container's Jammy apt repos. So capture stays on
the HOST: scripts/csi_cam_stream.sh runs `rpicam-vid --listen -o tcp://...`
in a respawn loop (see scripts/ecza-robotu-csi-cam.service). This node is
just the TCP client living in ROS-land — it connects to that host-side
stream (reachable at 127.0.0.1 because containers use network_mode: host),
splits the byte stream on JPEG SOI/EOI markers, and republishes frames the
same way esp32cam_node.py does for the other camera.

Publishes:
  /camera_csi/image_raw/compressed  sensor_msgs/CompressedImage  (JPEG passthrough)

Also serves an MJPEG stream over plain HTTP (no ROS2 client needed), same
latest-frame-signaled design as esp32cam_node.py — this is the intended way
to view this feed. Do NOT add an rviz_default_plugins/Image display to
rover.rviz for this: that reliably segfaults rviz2 under this container's
Xvfb+llvmpipe setup even when Enabled: false (see rover.rviz's Map display
comment / memory rviz-crash-loop-image-display). Use rqt_image_view from a
networked machine, or open http://<rpi-ip>:<http_port>/ in a browser.
"""

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'
MAX_FRAME = 2_000_000

# Shared between the reader thread (writer) and HTTP server threads
# (readers) — module-level so MJPEGHandler (instantiated per-request by
# ThreadingHTTPServer) can reach the latest frame without a node reference.
_latest_frame: bytes = None
_frame_seq = 0
_frame_cond = threading.Condition()


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # suppress per-request access logs

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b"""
                <html><body style="margin:0;background:#000">
                <img src="/stream" style="width:100%;height:100vh;object-fit:contain">
                </body></html>
            """)
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last_seq = -1
            try:
                while True:
                    with _frame_cond:
                        while _frame_seq == last_seq:
                            _frame_cond.wait(timeout=1.0)
                        jpg = _latest_frame
                        last_seq = _frame_seq
                    if jpg is not None:
                        self.wfile.write(
                            b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
                        )
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


class RpicamNode(Node):

    def __init__(self):
        super().__init__('rpicam_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8090)
        self.declare_parameter('frame_id', 'csi_camera_link')
        self.declare_parameter('http_port', 8081)
        self.declare_parameter('reconnect_delay_s', 1.0)

        self._host = self.get_parameter('host').value
        self._port = int(self.get_parameter('port').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._http_port = int(self.get_parameter('http_port').value)
        self._reconnect_delay = float(self.get_parameter('reconnect_delay_s').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._pub_compressed = self.create_publisher(
            CompressedImage, 'camera_csi/image_raw/compressed', qos)

        # ThreadingHTTPServer, not plain HTTPServer: the /stream handler
        # never returns (infinite MJPEG write loop), so a single-threaded
        # server can only ever serve ONE viewer — every other client (a
        # second browser tab, remote_teleop_client.py, even curl from
        # localhost) hangs on connect until that one viewer disconnects.
        # See remote-teleop memory, 2026-08-01.
        self._http_server = ThreadingHTTPServer(('0.0.0.0', self._http_port), MJPEGHandler)
        self._http_server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        threading.Thread(target=self._http_server.serve_forever, daemon=True).start()
        self.get_logger().info(f'MJPEG stream: http://0.0.0.0:{self._http_port}/')

        self._stats = {"frames": 0, "bytes": 0}
        self._last_report = time.time()

        # The host-side rpicam-vid --listen only serves one client at a time
        # and exits when that client disconnects (scripts/csi_cam_stream.sh
        # respawns it) — so this loop expects to reconnect periodically, not
        # just once at startup.
        self._running = True
        self._reader_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f'rpicam_node ready — source tcp://{self._host}:{self._port}, '
            f'frame_id={self._frame_id}'
        )

    def _connect_loop(self):
        while self._running:
            try:
                self._stream_from(self._connect())
            except (ConnectionRefusedError, ConnectionResetError, OSError) as e:
                self.get_logger().warn(
                    f'CSI stream connection lost/unavailable: {e}; retrying in '
                    f'{self._reconnect_delay:.1f}s', throttle_duration_sec=5.0,
                )
            time.sleep(self._reconnect_delay)

    def _connect(self) -> socket.socket:
        sock = socket.create_connection((self._host, self._port), timeout=5.0)
        sock.settimeout(2.0)
        self.get_logger().info(f'Connected to CSI stream at {self._host}:{self._port}')
        return sock

    def _stream_from(self, sock: socket.socket) -> None:
        buf = bytearray()
        with sock:
            while self._running:
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionResetError('CSI stream closed by peer')
                buf.extend(chunk)

                # Drop any leading bytes before the first SOI so buf always
                # starts on a frame boundary once one has been seen.
                if not buf.startswith(SOI):
                    idx = buf.find(SOI)
                    if idx < 0:
                        if len(buf) > MAX_FRAME:
                            del buf[:-2]  # keep only a possible partial marker
                        continue
                    del buf[:idx]

                end = buf.find(EOI, 2)
                if end < 0:
                    if len(buf) > MAX_FRAME:
                        buf.clear()  # runaway frame with no EOI — resync
                    continue

                frame = bytes(buf[:end + 2])
                del buf[:end + 2]
                self._publish_frame(frame)

    def _publish_frame(self, frame: bytes) -> None:
        now = self.get_clock().now()
        msg = CompressedImage()
        msg.header = Header(stamp=now.to_msg(), frame_id=self._frame_id)
        msg.format = 'jpeg'
        msg.data = frame
        self._pub_compressed.publish(msg)

        global _latest_frame, _frame_seq
        with _frame_cond:
            _latest_frame = frame
            _frame_seq += 1
            _frame_cond.notify_all()

        self._stats["frames"] += 1
        self._stats["bytes"] += len(frame)
        self._report()

    def _report(self) -> None:
        now = time.time()
        span = now - self._last_report
        if span < 1.0:
            return
        fps = self._stats["frames"] / span
        kbps = self._stats["bytes"] / 1000 / span
        self.get_logger().info(
            f'FPS {fps:4.1f} | {kbps:5.0f} KB/s', throttle_duration_sec=0.0,
        )
        self._stats.update(frames=0, bytes=0)
        self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = RpicamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node._http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
