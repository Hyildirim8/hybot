"""Regression tests: callbacks only, no ROS node or motor publisher is created.

Run with the ROS environment sourced:
    python3 -m unittest discover -s src/ecza_teleop/test -v
"""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ecza_teleop.teleop_node import TeleopNode


def command(x=0.0, y=0.0, yaw=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = x, y, yaw
    return msg


class NavigationGateTest(unittest.TestCase):
    def setUp(self):
        # Bind the real callbacks to a plain object: no DDS participant exists.
        self.node = SimpleNamespace(
            _autonomous=True, _enable_scan_safety=True,
            _last_scan_time=time.monotonic(), _scan_timeout=0.5,
            _last_nav_time=time.monotonic(), _nav_timeout=0.5,
            _auto_half_length=0.23, _auto_half_width=0.16,
            _auto_margin=0.01, _auto_horizon=0.6,
            _scan_points=[(3.0, 0.0)], _auto_strafe_invert=False,
            _auto_enable_on_nav_cmd=False, _auto_enable_inhibited=False,
            _handoff_stop_until=0.0, _publish_cmd=Mock(), _publish_zero=Mock(),
            _front_angle_rad=math.radians(36), _lidar_angle_offset_rad=math.pi,
            _front_stop_distance=0.3, _front_clear_distance=0.5,
            _front_blocked=False, get_logger=lambda: Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        )
        for method in ('_auto_scan_safe', '_twist_has_motion', '_apply_scan_safety'):
            setattr(self.node, method, getattr(TeleopNode, method).__get__(self.node))

    def test_clear_path_preserves_all_command_axes(self):
        msg = command(0.2, 0.1, 0.3)
        TeleopNode._nav_cmd_cb(self.node, msg)
        self.assertEqual(self.node._publish_cmd.call_args.args[0], msg)

    def test_lateral_direction_is_not_reversed(self):
        for y in (-0.15, 0.15):
            TeleopNode._nav_cmd_cb(self.node, command(y=y))
            self.assertEqual(self.node._publish_cmd.call_args.args[0].linear.y, y)

    def test_walls_block_each_translation_direction(self):
        for point, msg in [((0.38, 0.0), command(x=0.3)),
                           ((-0.30, 0.0), command(x=-0.15)),
                           ((0.0, 0.24), command(y=0.15)),
                           ((0.0, -0.24), command(y=-0.15))]:
            with self.subTest(point=point):
                self.node._scan_points = [point]
                self.assertEqual(self.node._apply_scan_safety(msg), Twist())

    def test_emergency_stop_does_not_invent_rotation_or_keep_strafe(self):
        self.node._scan_points = [(0.29, 0.0)]
        self.assertEqual(self.node._apply_scan_safety(command(0.2, 0.1, 0.3)), Twist())

    def test_rotating_corner_is_checked(self):
        self.node._scan_points = [(0.21, 0.20)]
        self.assertFalse(self.node._auto_scan_safe(command(yaw=0.6)))

    def test_straight_motion_through_forty_cm_passage(self):
        self.node._scan_points = [(x, y) for x in (-0.3, 0.0, 0.3, 0.5)
                                  for y in (-0.20, 0.20)]
        self.assertTrue(self.node._auto_scan_safe(command(x=0.3)))

    def test_passage_narrower_than_body_is_blocked(self):
        self.node._scan_points = [(0.3, -0.15), (0.3, 0.15)]
        self.assertFalse(self.node._auto_scan_safe(command(x=0.3)))

    def test_straight_escape_from_close_wall_is_allowed(self):
        self.node._scan_points = [(0.22, 0.0)]
        self.assertTrue(self.node._auto_scan_safe(command(x=-0.15)))
        self.assertFalse(self.node._auto_scan_safe(command(x=0.15)))

    def test_reverse_escape_checks_rear_wall(self):
        self.node._scan_points = [(0.22, 0.0), (-0.31, 0.0)]
        self.assertFalse(self.node._auto_scan_safe(command(x=-0.15)))

    def test_stale_scan_stops_motion(self):
        self.node._last_scan_time = time.monotonic() - 1.0
        self.assertEqual(self.node._apply_scan_safety(command(y=0.1)), Twist())

    def test_watchdog_stops_if_nav_disappears(self):
        self.node._last_nav_time = time.monotonic() - 1.0
        TeleopNode._watchdog_cb(self.node)
        self.node._publish_zero.assert_called_once()

    def test_watchdog_stops_if_scan_disappears(self):
        self.node._last_scan_time = time.monotonic() - 1.0
        TeleopNode._watchdog_cb(self.node)
        self.node._publish_zero.assert_called_once()

    def test_fresh_streams_do_not_trigger_watchdog(self):
        TeleopNode._watchdog_cb(self.node)
        self.node._publish_zero.assert_not_called()

    def test_nonfinite_command_is_stopped(self):
        self.assertEqual(self.node._apply_scan_safety(command(x=math.nan)), Twist())

    def scan(self, age=0, ranges=None):
        scan = LaserScan()
        scan.header.stamp.sec = 100 - age
        scan.range_min, scan.range_max = 0.1, 12.0
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 2
        scan.ranges = ranges if ranges is not None else [0.38, 3.0, 3.0, 3.0]
        return scan

    def test_lidar_mount_rotation_maps_wall_to_robot_front(self):
        TeleopNode._scan_cb(self.node, self.scan())
        self.assertFalse(self.node._auto_scan_safe(command(x=0.3)))
        self.assertTrue(self.node._auto_scan_safe(command(x=-0.15)))

    def test_delayed_and_empty_scans_are_not_fresh(self):
        for scan in (self.scan(age=2), self.scan(ranges=[]),
                     self.scan(ranges=[math.inf, math.nan, 0.0])):
            TeleopNode._scan_cb(self.node, scan)
            self.assertEqual(self.node._last_scan_time, 0.0)
            self.assertFalse(self.node._auto_scan_safe(command(x=0.1)))


if __name__ == '__main__':
    unittest.main()
