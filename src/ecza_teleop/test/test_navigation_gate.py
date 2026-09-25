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
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
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
            _auto_wheel_surface_limit=0.75, _pivot_k=0.32,
            _scan_points=[(3.0, 0.0)], _auto_strafe_invert=False,
            _auto_enable_on_nav_cmd=False, _auto_enable_inhibited=False,
            _manual_teleop_lock=False, _nav_auto_enable_blocked_until=0.0,
            _publish_mode=Mock(),
            _handoff_stop_until=0.0, _publish_cmd=Mock(), _publish_zero=Mock(),
            _front_angle_rad=math.radians(36), _lidar_angle_offset_rad=math.pi,
            _front_stop_distance=0.3, _front_clear_distance=0.5,
            _front_blocked=False, _front_blind=False,
            _last_output=Twist(), _measured_twist=Twist(), _last_odom_time=time.monotonic(),
            get_logger=lambda: Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        )
        for method in ('_auto_scan_safe', '_twist_has_motion', '_apply_scan_safety', '_enforce_latest_scan'):
            setattr(self.node, method, getattr(TeleopNode, method).__get__(self.node))

    def test_clear_path_preserves_all_command_axes(self):
        msg = command(0.2, 0.1, 0.3)
        TeleopNode._nav_cmd_cb(self.node, msg)
        self.assertEqual(self.node._publish_cmd.call_args.args[0], msg)

    def test_increased_forward_speed_is_allowed_in_open_space(self):
        self.assertEqual(self.node._apply_scan_safety(command(x=0.6)).linear.x, 0.6)

    def test_combined_maximum_preserves_curve_within_motor_limit(self):
        msg = command(0.6, 0.05, 0.8)
        out = self.node._apply_scan_safety(msg)
        demand = abs(out.linear.x) + abs(out.linear.y) + 0.32 * abs(out.angular.z)
        self.assertLessEqual(demand, 0.75 + 1e-9)
        self.assertAlmostEqual(out.angular.z/out.linear.x, 0.8/0.6)
        self.assertAlmostEqual(out.linear.y/out.linear.x, 0.05/0.6)

    def test_increased_speed_stops_with_obstacle_in_braking_distance(self):
        self.node._scan_points = [(0.65, 0.0)]
        self.assertEqual(self.node._apply_scan_safety(command(x=0.6)), Twist())

    def test_exploration_request_enters_auto_without_motion_command(self):
        self.node._autonomous = False
        self.node._manual_teleop_lock = True
        self.node._auto_enable_inhibited = True
        self.node._nav_auto_enable_blocked_until = time.monotonic()+5
        TeleopNode._exploring_cb(self.node, Bool(data=True))
        self.assertTrue(self.node._autonomous)
        self.assertFalse(self.node._manual_teleop_lock)
        self.assertFalse(self.node._auto_enable_inhibited)
        self.assertEqual(self.node._nav_auto_enable_blocked_until, 0.0)
        self.node._publish_zero.assert_called_once()
        self.node._publish_mode.assert_called_once()
        self.node._publish_cmd.assert_not_called()

    def test_stale_nav_motion_cannot_override_manual_takeover(self):
        self.node._autonomous = False
        self.node._manual_teleop_lock = True
        self.node._auto_enable_on_nav_cmd = True
        TeleopNode._nav_cmd_cb(self.node, command(x=0.2))
        self.assertFalse(self.node._autonomous)
        self.node._publish_cmd.assert_not_called()

    def test_exploration_off_does_not_engage_auto(self):
        self.node._autonomous = False
        TeleopNode._exploring_cb(self.node, Bool(data=False))
        self.assertFalse(self.node._autonomous)
        self.node._publish_mode.assert_not_called()

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
                self.assertFalse(self.node._auto_scan_safe(msg))

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

    def test_far_obstacle_slows_command_without_changing_curve(self):
        self.node._auto_horizon = 1.0
        self.node._scan_points = [(0.48, 0.0)]
        output = self.node._apply_scan_safety(command(0.3, 0.02, 0.05))
        self.assertGreater(output.linear.x, 0.0)
        self.assertLess(output.linear.x, 0.3)
        self.assertAlmostEqual(output.angular.z / output.linear.x, 0.05 / 0.3)
        self.assertAlmostEqual(output.linear.y / output.linear.x, 0.02 / 0.3)

    def test_measured_momentum_cannot_be_hidden_by_slow_command(self):
        self.node._scan_points = [(0.48, 0.0)]
        self.node._measured_twist = command(x=0.5)
        self.assertEqual(self.node._apply_scan_safety(command(x=0.05)), Twist())

    def test_stale_odometry_stops_motion(self):
        self.node._last_odom_time = time.monotonic() - 1.0
        self.assertEqual(self.node._apply_scan_safety(command(x=0.2)), Twist())

    def test_new_scan_stops_previous_command_immediately(self):
        self.node._last_output = command(x=0.3)
        TeleopNode._scan_cb(self.node, self.scan(ranges=[0.28, 3.0, 3.0, 3.0]))
        self.node._publish_zero.assert_called_once()

    def test_blind_front_blocks_auto_until_front_returns(self):
        scan = self.scan()
        scan.angle_increment = math.pi / 180
        scan.ranges = [math.inf] * 360
        scan.ranges[40] = 0.25  # proximity evidence outside the blind front cone
        TeleopNode._scan_cb(self.node, scan)
        self.assertTrue(self.node._front_blind)
        self.assertEqual(self.node._apply_scan_safety(command(x=0.1)), Twist())
        self.assertEqual(self.node._apply_scan_safety(command(yaw=0.2)), Twist())
        scan.ranges[40] = 3.0
        TeleopNode._scan_cb(self.node, scan)
        self.assertTrue(self.node._front_blind)
        scan.ranges = [3.0] * 360
        TeleopNode._scan_cb(self.node, scan)
        self.assertFalse(self.node._front_blind)
        self.assertTrue(self.node._auto_scan_safe(command(x=0.1)))

    def test_door_posts_outside_body_do_not_latch_blind_wall(self):
        scan = self.scan()
        scan.angle_increment = math.pi / 180
        scan.ranges = [math.inf] * 360
        scan.ranges[40] = scan.ranges[320] = 0.35
        TeleopNode._scan_cb(self.node, scan)
        self.assertFalse(self.node._front_blind)
        self.assertTrue(self.node._auto_scan_safe(command(x=0.1)))

    def test_stationary_odom_jitter_does_not_block_straight_retreat(self):
        msg = Odometry()
        msg.header.stamp.sec = 100
        msg.twist.twist.angular.z = 0.0005
        TeleopNode._odom_cb(self.node, msg)
        self.node._scan_points = [(0.22, 0.0)]
        self.assertLess(self.node._apply_scan_safety(command(x=-0.1)).linear.x, 0.0)

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
