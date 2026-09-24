"""Offline sensor regression tests; no ROS node, motor, or I2C device is opened."""
import importlib.util
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

ROOT = Path(__file__).resolve().parents[2]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

Scan = load('scan_filter', 'src/ecza_lidar/scripts/scan_restamper.py').ScanRestamper
IMU = load('gyro_driver', 'src/ecza_imu/ecza_imu/imu_node.py').GY85Node
floor = load('covariance_adapter', 'src/ecza_kinematics/ecza_kinematics/laser_odom_covariance.py').with_covariance_floor


class ScanTimingTest(unittest.TestCase):
    def scan(self):
        msg = LaserScan()
        msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 345
        msg.range_min, msg.range_max = 0.1, 12.0
        msg.ranges = [1.0, 2.0, 3.0, 4.0]
        msg.intensities = [1.0] * 4
        msg.angle_min, msg.angle_increment = -1.0, 0.5
        msg.time_increment, msg.scan_time = 0.02, 0.08
        return msg

    def test_acquisition_stamp_survives_processing(self):
        msg = self.scan()
        out = Scan._prepare_scan(SimpleNamespace(_frame_id='laser_frame'), msg, 1)
        self.assertEqual(out.header.stamp, msg.header.stamp)
        self.assertEqual(out.scan_time, msg.scan_time)
        self.assertEqual(msg.header.frame_id, '')

    def test_downsampling_keeps_first_ray_time_and_timing_stride(self):
        msg = self.scan()
        out = Scan._prepare_scan(SimpleNamespace(_frame_id='laser_frame'), msg, 2)
        self.assertEqual(out.header.stamp, msg.header.stamp)
        self.assertAlmostEqual(out.time_increment, 0.04)
        self.assertEqual(list(out.ranges), [1.0, 3.0])
        self.assertEqual(len(msg.ranges), 4)

    def test_turn_does_not_interrupt_slam_scans(self):
        n = SimpleNamespace(_min_valid_points=0, _slam_min_valid_points=0,
            _min_publish_ns=0, _slam_min_publish_ns=0,
            _last_publish_ns=0, _last_slam_publish_ns=0,
            _angle_downsample=1, _slam_angle_downsample=1, _frame_id='laser_frame',
            _angular_z=2.0, _max_slam_angular_z=0.65,
            _pub=Mock(), _slam_pub=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=15_000_000_000)))
        n._prepare_scan = Scan._prepare_scan.__get__(n)
        Scan._scan_cb(n, self.scan())
        n._slam_pub.publish.assert_called_once()
        self.assertEqual(n._slam_pub.publish.call_args.args[0].header.stamp.sec, 12)


class GyroBiasTest(unittest.TestCase):
    def setUp(self):
        self.n = SimpleNamespace(_last_command_time=time.monotonic(),
            _last_wheel_time=time.monotonic(), _command_still=True, _wheels_still=True,
            _raw_window=deque(maxlen=5), _still_count=0, _still_required=5,
            _still_band=0.18, _still_abs_max=0.6, _bias_adapt_rate=0.05,
            _gyro_bias=(0.0, 0.0, -0.36))
        self.n._reset_bias_tracking = IMU._reset_bias_tracking.__get__(self.n)

    def feed(self, z=-0.06):
        for _ in range(100):
            IMU._track_bias(self.n, (0.0, 0.0, z), (0.0, 0.0, z + 0.36))

    def test_commanded_constant_turn_never_changes_bias(self):
        msg = Twist(); msg.angular.z = 0.3
        IMU._command_cb(self.n, msg)
        self.feed()
        self.assertEqual(self.n._gyro_bias[2], -0.36)

    def test_moving_wheels_block_adaptation_even_with_zero_command(self):
        IMU._wheel_cb(self.n, Float32MultiArray(data=[-1.0, 1.0, -1.0, 1.0]))
        self.feed()
        self.assertEqual(self.n._gyro_bias[2], -0.36)

    def test_stale_feedback_cannot_confirm_stillness(self):
        for attr in ('_last_command_time', '_last_wheel_time'):
            with self.subTest(attr=attr):
                setattr(self.n, attr, time.monotonic() - 1.0)
                self.feed()
                self.assertEqual(self.n._gyro_bias[2], -0.36)
                setattr(self.n, attr, time.monotonic())

    def test_stationary_temperature_drift_can_still_adapt(self):
        self.feed(-0.40)
        self.assertLess(abs(self.n._gyro_bias[2] + 0.40), 0.001)

    def test_brief_motion_clears_accumulated_stillness(self):
        self.feed(-0.4)
        IMU._wheel_cb(self.n, Float32MultiArray(data=[1.0] * 4))
        self.assertEqual(self.n._still_count, 0)
        self.assertEqual(len(self.n._raw_window), 0)

    def test_partial_or_invalid_wheel_feedback_is_not_stillness(self):
        for values in ([0.0], [math.nan, 0.0, 0.0, 0.0]):
            IMU._wheel_cb(self.n, Float32MultiArray(data=values))
            self.assertFalse(self.n._wheels_still)


class CovarianceTest(unittest.TestCase):
    def test_zero_covariance_gets_floor_without_changing_pose_or_time(self):
        msg = Odometry(); msg.header.stamp.sec = 42
        msg.pose.pose.position.x = 6.8; msg.pose.pose.orientation.w = 1.0
        out = floor(msg, 0.0025, 0.0225)
        self.assertEqual(out.header, msg.header)
        self.assertEqual(out.pose.pose, msg.pose.pose)
        self.assertEqual(out.pose.covariance[0], 0.0025)
        self.assertEqual(out.pose.covariance[35], 0.0225)
        self.assertEqual(msg.pose.covariance[35], 0.0)

    def test_higher_reported_uncertainty_is_retained(self):
        msg = Odometry(); msg.pose.covariance[35] = 0.5
        self.assertEqual(floor(msg, 0.0025, 0.0225).pose.covariance[35], 0.5)


if __name__ == '__main__':
    unittest.main()
