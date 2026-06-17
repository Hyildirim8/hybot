#!/usr/bin/env python3
"""Restamp LaserScan messages so TF consumers do not reject stale scan times.

Also drops unhealthy scans: the A2M12 on this rover intermittently produces
near-empty revolutions (2-13 valid rays out of 720). Publishing those poisons
SLAM matching, costmap marking and the explorer's distance estimates, so any
scan with fewer than min_valid_fraction valid rays is discarded — consumers
keep working from the last healthy scan.
"""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__("scan_restamper")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("slam_output_topic", "/scan_slam")
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("max_publish_hz", 5.0)
        self.declare_parameter("angle_downsample", 2)
        self.declare_parameter("slam_publish_hz", 2.0)
        self.declare_parameter("slam_angle_downsample", 4)
        self.declare_parameter("max_slam_angular_z", 0.25)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("min_valid_fraction", 0.10)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        slam_output_topic = self.get_parameter("slam_output_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        max_publish_hz = float(self.get_parameter("max_publish_hz").value)
        self._angle_downsample = max(1, int(self.get_parameter("angle_downsample").value))
        slam_publish_hz = float(self.get_parameter("slam_publish_hz").value)
        self._slam_angle_downsample = max(
            1, int(self.get_parameter("slam_angle_downsample").value)
        )
        self._max_slam_angular_z = float(self.get_parameter("max_slam_angular_z").value)
        odom_topic = self.get_parameter("odom_topic").value
        self._min_valid_fraction = float(self.get_parameter("min_valid_fraction").value)
        self._dropped_scans = 0
        self._min_publish_ns = int(1e9 / max_publish_hz) if max_publish_hz > 0.0 else 0
        self._slam_min_publish_ns = (
            int(1e9 / slam_publish_hz) if slam_publish_hz > 0.0 else 0
        )
        self._last_publish_ns = 0
        self._last_slam_publish_ns = 0
        self._angular_z = 0.0

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Nav2 costmap subscribes with RELIABLE; publish RELIABLE so data flows.
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self._slam_pub = self.create_publisher(LaserScan, slam_output_topic, pub_qos)
        self.create_subscription(LaserScan, input_topic, self._scan_cb, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self.get_logger().info(
            f"scan_restamper {input_topic} -> {output_topic} "
            f"max_publish_hz={max_publish_hz:.2f} angle_downsample={self._angle_downsample}; "
            f"{input_topic} -> {slam_output_topic} slam_publish_hz={slam_publish_hz:.2f} "
            f"slam_angle_downsample={self._slam_angle_downsample} "
            f"max_slam_angular_z={self._max_slam_angular_z:.2f}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._angular_z = float(msg.twist.twist.angular.z)

    def _scan_cb(self, msg: LaserScan) -> None:
        # Sağlıksız tarama filtresi: geçerli ışın oranı eşiğin altındaysa hiç
        # yayınlama — SLAM/costmap/keşif son sağlıklı taramayla devam eder.
        if self._min_valid_fraction > 0.0 and msg.ranges:
            lo = max(0.0, msg.range_min)
            valid = sum(
                1 for d in msg.ranges
                if math.isfinite(d) and lo <= d <= msg.range_max
            )
            if valid < len(msg.ranges) * self._min_valid_fraction:
                self._dropped_scans += 1
                self.get_logger().warn(
                    f"Bozuk tarama atıldı: {valid}/{len(msg.ranges)} geçerli ışın "
                    f"(toplam atılan: {self._dropped_scans})",
                    throttle_duration_sec=10.0,
                )
                return

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self._min_publish_ns and now_ns - self._last_publish_ns < self._min_publish_ns:
            publish_scan = False
        else:
            publish_scan = True
            self._last_publish_ns = now_ns

        publish_slam = True
        if self._slam_min_publish_ns and now_ns - self._last_slam_publish_ns < self._slam_min_publish_ns:
            publish_slam = False
        if abs(self._angular_z) > self._max_slam_angular_z:
            publish_slam = False

        if publish_scan:
            self._pub.publish(self._prepare_scan(msg, now, self._angle_downsample))
        if publish_slam:
            self._last_slam_publish_ns = now_ns
            self._slam_pub.publish(
                self._prepare_scan(msg, now, self._slam_angle_downsample)
            )

    def _prepare_scan(self, msg: LaserScan, now, downsample: int) -> LaserScan:
        out = copy.deepcopy(msg)
        out.header.stamp = now.to_msg()
        if self._frame_id:
            out.header.frame_id = self._frame_id
        if downsample > 1:
            ranges = list(out.ranges[::downsample])
            intensities = list(out.intensities[::downsample]) if out.intensities else []
            out.ranges = ranges
            out.intensities = intensities
            out.angle_increment *= float(downsample)
            out.angle_max = out.angle_min + out.angle_increment * max(0, len(ranges) - 1)
            out.time_increment *= float(downsample)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
