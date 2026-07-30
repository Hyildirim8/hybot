#!/usr/bin/env python3
"""ESP32-CAM Raw Binary Serial receiver for ecza-robotu.

Ports the protocol from ESP32-CAM-RealTime-Vision-Benchmark's
scripts/rec_cam_binary.py + firmware/binary.cpp (the "Recommended for
Navigation" method from that project's benchmark: 115.8ms avg latency,
7.5ms jitter, <1us CPU encode cost — far more stable than the WiFi/HTTP
variant, which had 55.6ms jitter and spikes up to 1.5s).

Wire protocol (UART, non-standard baud via BOTHER ioctl):
  0xAA 0xBB | uint32 size LE | uint16 CRC16-MODBUS(size) LE | JPEG bytes

Publishes:
  /camera/image_raw/compressed  sensor_msgs/CompressedImage  (JPEG passthrough)
  /camera/image_raw             sensor_msgs/Image  (only if publish_raw:=true)

Also serves an MJPEG stream over plain HTTP (no ROS2 client needed) at
http://<rpi-ip>:<http_port>/  — same latest-frame-signaled design as the
upstream benchmark script's stream server, reusing the already-decoded
JPEG bytes with no re-encode.

Requires the uart-clk-100m dtoverlay (already applied on this Pi — see
/boot/firmware/config.txt) for clean 4,000,000 baud on ttyAMA0.
"""

import fcntl
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
import serial

# BOTHER ioctl: required for non-standard baud rates (4,000,000) on Linux aarch64.
# struct termios2 layout: 4x uint32 + uint8 + uint8[19] + 2x uint32 = 44 bytes
_BOTHER  = 0x1000
_CBAUD   = 0x100f
_HUPCL   = 0x0400
_TCGETS2 = 0x802C542A
_TCSETS2 = 0x402C542B

HEADER = b'\xAA\xBB'
MAX_FRAME = 200_000

# Shared between the reader thread (writer) and HTTP server threads
# (readers) — module-level so MJPEGHandler (instantiated per-request by
# HTTPServer) can reach the latest frame without a node reference.
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


def _set_baudrate(fd, baudrate):
    buf = bytearray(44)
    fcntl.ioctl(fd, _TCGETS2, buf)
    c_cflag = struct.unpack_from('<I', buf, 8)[0]
    c_cflag &= ~_HUPCL
    c_cflag = (c_cflag & ~_CBAUD) | _BOTHER
    struct.pack_into('<I', buf, 8,  c_cflag)
    struct.pack_into('<I', buf, 36, baudrate)
    struct.pack_into('<I', buf, 40, baudrate)
    fcntl.ioctl(fd, _TCSETS2, buf)


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


class ESP32CamNode(Node):

    def __init__(self):
        super().__init__('esp32cam_node')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',   4_000_000)
        self.declare_parameter('frame_id',    'camera_link')
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('http_port',   8080)

        self._port      = self.get_parameter('serial_port').value
        self._baud      = int(self.get_parameter('baud_rate').value)
        self._frame_id  = self.get_parameter('frame_id').value
        self._pub_raw   = bool(self.get_parameter('publish_raw').value)
        self._http_port = int(self.get_parameter('http_port').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._pub_compressed = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', qos)
        self._pub_raw_img = None
        self._cv_bridge = None
        if self._pub_raw:
            import cv2
            import numpy
            from cv_bridge import CvBridge
            self._cv2 = cv2
            self._np = numpy
            self._cv_bridge = CvBridge()
            self._pub_raw_img = self.create_publisher(Image, 'camera/image_raw', qos)

        self._ser = self._connect()

        self._http_server = HTTPServer(('0.0.0.0', self._http_port), MJPEGHandler)
        self._http_server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        threading.Thread(target=self._http_server.serve_forever, daemon=True).start()
        self.get_logger().info(f'MJPEG stream: http://0.0.0.0:{self._http_port}/')

        self._buf = bytearray()
        self._stats = {"crc": 0, "size": 0, "marker": 0, "bytes": 0, "frames": 0}
        self._last_report = time.time()
        self._link_kbps = self._baud / 10 / 1000  # 8N1 → 10 bits/byte

        # Blocking read loop runs on its own thread — matches the upstream
        # script's design (tight blocking reads, no polling) and keeps CPU
        # usage near zero between frames, which is the whole reason binary
        # serial was chosen over the WiFi/HTTP variant (see module docstring).
        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _connect(self) -> serial.Serial:
        self.get_logger().info(f'Connecting to {self._port} at {self._baud} baud...')
        ser = serial.Serial(self._port, 115200, timeout=1)
        ser.rts = False           # hold ESP32 in reset while we configure UART
        _set_baudrate(ser.fd, self._baud)
        ser.reset_input_buffer()
        ser.rts = True            # release reset — ESP32 boots and streams at target baud
        time.sleep(2.0)           # camera init (OV2640 needs ~1-2s)
        ser.reset_input_buffer()
        self.get_logger().info(f'Connection established at {self._baud} baud.')
        return ser

    def _read(self, n: int) -> bytes:
        while len(self._buf) < n:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(max(waiting, n - len(self._buf)))
            if chunk:
                self._buf.extend(chunk)
        result = bytes(self._buf[:n])
        del self._buf[:n]
        return result

    def _sync(self):
        """Block until the buffer is advanced to the next frame header."""
        while self._running:
            waiting = self._ser.in_waiting
            if waiting > 0:
                self._buf.extend(self._ser.read(min(waiting, 4096)))
            idx = self._buf.find(HEADER)
            if idx >= 0:
                del self._buf[:idx + 2]
                return
            if len(self._buf) > 1:
                del self._buf[:-1]
            self._buf.extend(self._ser.read(1))

    def _read_loop(self):
        while self._running:
            try:
                self._read_frame()
            except serial.SerialException as e:
                self.get_logger().error(f'serial error, stopping reader: {e}')
                return
            except Exception as e:
                self.get_logger().warn(f'frame read error: {e}', throttle_duration_sec=5.0)

    def _read_frame(self):
        self._sync()
        if not self._running:
            return
        size_data = self._read(4)
        crc_data = self._read(2)

        if _crc16(size_data) != struct.unpack('<H', crc_data)[0]:
            self._stats["crc"] += 1
            return

        img_size = struct.unpack('<I', size_data)[0]
        if img_size < 100 or img_size > MAX_FRAME:
            self._stats["size"] += 1
            return

        img_data = self._read(img_size)
        if img_data[:2] != b'\xff\xd8' or img_data[-2:] != b'\xff\xd9':
            self._stats["marker"] += 1
            return

        now = self.get_clock().now()
        header = Header(stamp=now.to_msg(), frame_id=self._frame_id)

        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = img_data
        self._pub_compressed.publish(msg)

        global _latest_frame, _frame_seq
        with _frame_cond:
            _latest_frame = img_data
            _frame_seq += 1
            _frame_cond.notify_all()

        if self._pub_raw_img is not None:
            frame = self._cv2.imdecode(
                self._np.frombuffer(img_data, self._np.uint8),
                self._cv2.IMREAD_COLOR)
            if frame is not None:
                img_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = header
                self._pub_raw_img.publish(img_msg)

        self._stats["bytes"] += img_size
        self._stats["frames"] += 1
        self._report()

    def _report(self):
        now = time.time()
        span = now - self._last_report
        if span < 1.0:
            return
        frames = self._stats["frames"]
        fps = frames / span
        kbps = self._stats["bytes"] / 1000 / span
        util = 100 * kbps / self._link_kbps if self._link_kbps else 0.0
        self.get_logger().info(
            f'FPS {fps:4.1f} | {kbps:5.0f} KB/s ({util:4.1f}% link) | '
            f'rejects crc={self._stats["crc"]} size={self._stats["size"]} '
            f'marker={self._stats["marker"]}',
            throttle_duration_sec=0.0,
        )
        self._stats.update(crc=0, size=0, marker=0, bytes=0, frames=0)
        self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = ESP32CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node._reader_thread.join(timeout=2.0)
        node._http_server.shutdown()
        node._ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
