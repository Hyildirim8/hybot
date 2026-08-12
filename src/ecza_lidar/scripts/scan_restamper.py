#!/usr/bin/env python3
"""Restamp LaserScan messages so TF consumers do not reject stale scan times."""

import copy
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
        # Drop fragment scans (0 disables). The driver emits ~15 scan messages
        # per second while the lidar only completes ~12.7 rotations, so some
        # messages carry a slice of a rotation instead of a whole one. Measured
        # 2026-08-11 over 310 scans: 51% arrived with 0-25 valid points and 38%
        # with 150-400 — an almost empty middle. Publishing that mix makes
        # scan matching alternate between good and useless input, which is what
        # smeared the map. Gating on valid-point count keeps only whole
        # rotations; the array length never changes, so slam_toolbox's
        # first-scan laser registration still matches.
        self.declare_parameter("min_valid_points", 0)
        self.declare_parameter("slam_min_valid_points", 0)

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
        self._min_valid_points = max(0, int(self.get_parameter("min_valid_points").value))
        self._slam_min_valid_points = max(
            0, int(self.get_parameter("slam_min_valid_points").value)
        )
        self._dropped_fragments = 0
        self._min_publish_ns = int(1e9 / max_publish_hz) if max_publish_hz > 0.0 else 0
        self._slam_min_publish_ns = (
            int(1e9 / slam_publish_hz) if slam_publish_hz > 0.0 else 0
        )
        self._last_publish_ns = 0
        self._last_slam_publish_ns = 0
        self._angular_z = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, qos)
        self._slam_pub = self.create_publisher(LaserScan, slam_output_topic, qos)
        self.create_subscription(LaserScan, input_topic, self._scan_cb, qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.get_logger().info(
            f"scan_restamper {input_topic} -> {output_topic} "
            f"max_publish_hz={max_publish_hz:.2f} angle_downsample={self._angle_downsample}; "
            f"{input_topic} -> {slam_output_topic} slam_publish_hz={slam_publish_hz:.2f} "
            f"slam_angle_downsample={self._slam_angle_downsample} "
            f"max_slam_angular_z={self._max_slam_angular_z:.2f} "
            f"min_valid_points={self._min_valid_points} "
            f"slam_min_valid_points={self._slam_min_valid_points}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._angular_z = float(msg.twist.twist.angular.z)

    def _valid_point_count(self, msg: LaserScan) -> int:
        # inf/NaN both fail this comparison, so no isfinite() check is needed.
        lo, hi = msg.range_min, msg.range_max
        return sum(1 for r in msg.ranges if lo <= r <= hi)

    def _scan_cb(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        valid = -1
        if self._min_valid_points or self._slam_min_valid_points:
            valid = self._valid_point_count(msg)

        # A fragment must not advance the rate limiters, otherwise it consumes
        # the slot that the next whole rotation would have been published in.
        scan_ok = valid < 0 or valid >= self._min_valid_points
        slam_ok = valid < 0 or valid >= self._slam_min_valid_points
        if not (scan_ok or slam_ok):
            self._dropped_fragments += 1
            self.get_logger().debug(
                f"fragment scan atlandi ({valid} gecerli nokta; "
                f"toplam {self._dropped_fragments})",
                throttle_duration_sec=10.0,
            )

        publish_scan = scan_ok
        if self._min_publish_ns and now_ns - self._last_publish_ns < self._min_publish_ns:
            publish_scan = False

        publish_slam = slam_ok
        if self._slam_min_publish_ns and now_ns - self._last_slam_publish_ns < self._slam_min_publish_ns:
            publish_slam = False
        if abs(self._angular_z) > self._max_slam_angular_z:
            publish_slam = False

        if publish_scan:
            self._last_publish_ns = now_ns
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
