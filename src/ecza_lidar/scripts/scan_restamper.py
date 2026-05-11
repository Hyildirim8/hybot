#!/usr/bin/env python3
"""Restamp LaserScan messages so TF consumers do not reject stale scan times."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__("scan_restamper")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("max_publish_hz", 6.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        max_publish_hz = float(self.get_parameter("max_publish_hz").value)
        self._min_publish_ns = int(1e9 / max_publish_hz) if max_publish_hz > 0.0 else 0
        self._last_publish_ns = 0

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, qos)
        self.create_subscription(LaserScan, input_topic, self._scan_cb, qos)
        self.get_logger().info(
            f"scan_restamper {input_topic} -> {output_topic} "
            f"max_publish_hz={max_publish_hz:.2f}"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self._min_publish_ns and now_ns - self._last_publish_ns < self._min_publish_ns:
            return
        self._last_publish_ns = now_ns

        msg.header.stamp = now.to_msg()
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
