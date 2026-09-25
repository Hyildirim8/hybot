"""Synthetic maps and action callbacks only; no ROS node or motors."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav_msgs.msg import OccupancyGrid
from ecza_teleop.slam_manager_node import SlamManagerNode as Manager


def two_rooms(left_width=8, right_width=3):
    m = OccupancyGrid()
    w, h = left_width + right_width + 5, 8
    m.info.width, m.info.height, m.info.resolution = w, h, 0.1
    data = [100] * (w*h)
    starts = (1, left_width+3)
    for start, width in zip(starts, (left_width, right_width)):
        for y in range(1, 6):
            for x in range(start, start+width):
                data[y*w+x] = -1 if y == 5 else 0
    m.data = data
    return m, starts


class ExplorationPolicyTest(unittest.TestCase):
    def setUp(self):
        self.n = NS(_frontier_padding_m=0.0, _min_frontier=1,
            _visited_penalty_radius_sq=400.0,
            _goal_is_blacklisted=lambda target: False,
            _goal_generation=3, _goal_handle=Mock(), _active_goal=(1.0, 2.0),
            _goal_sent_at=time.monotonic(), _visited_positions=[],
            _enable_direct_explore=Mock(), _pub_status=Mock(),
            get_logger=lambda: Mock(), _failed_goals=[], _max_failed_goals=20)
        for name in ('_blacklist_active_goal', '_cancel_goal'):
            setattr(self.n, name, getattr(Manager, name).__get__(self.n))

    def test_toggle_waits_for_confirmed_auto_without_direct_motion(self):
        self.n._exploring = self.n._autonomous = False
        self.n._exploring_pub = Mock()
        self.n._user_goal_until = time.monotonic()+60
        Manager._toggle_exploration(self.n)
        self.assertTrue(self.n._exploring)
        self.assertFalse(self.n._autonomous)
        self.assertTrue(self.n._exploring_pub.publish.call_args.args[0].data)
        self.assertFalse(self.n._direct_explore_active)
        self.assertEqual(self.n._user_goal_until, 0)
        self.n._enable_direct_explore.assert_not_called()
        self.n._record_visited_position = Mock()
        Manager._explore_tick(self.n)
        self.n._record_visited_position.assert_not_called()

    def test_unvisited_frontier_beats_arbitrarily_large_old_frontier(self):
        m, starts = two_rooms(80, 1)
        self.n._visited_penalty_radius_sq = 4.5**2
        target = Manager._pick_frontier_from(self.n, m, [(4.05, 0.25)])
        self.assertAlmostEqual(target[0], (starts[1]+0.5)*m.info.resolution)

    def test_wall_keeps_neighbouring_room_unvisited(self):
        m, starts = two_rooms()
        target = Manager._pick_frontier_from(self.n, m, [(0.45, 0.25)])
        self.assertGreater(target[0], starts[1]*m.info.resolution)

    def test_frontier_goal_is_an_actual_free_cell_next_to_unknown(self):
        m, _ = two_rooms()
        target = Manager._pick_frontier_from(self.n, m, [])
        x, y = int(target[0]/m.info.resolution), int(target[1]/m.info.resolution)
        self.assertEqual(m.data[y*m.info.width+x], 0)
        self.assertIn(-1, [m.data[(y+dy)*m.info.width+x+dx]
                         for dx, dy in ((1,0),(-1,0),(0,1),(0,-1))])

    def test_map_origin_shift_does_not_change_visit_priority(self):
        m, starts = two_rooms()
        m.info.origin.position.x, m.info.origin.position.y = -12.0, 4.0
        target = Manager._pick_frontier_from(self.n, m, [(-11.55,4.25)])
        self.assertGreater(target[0], -12.0+starts[1]*m.info.resolution)

    def test_old_goal_result_cannot_erase_current_goal(self):
        Manager._on_goal_result(self.n, Mock(), 2)
        self.assertEqual(self.n._active_goal, (1.0, 2.0))
        self.n._enable_direct_explore.assert_not_called()

    def test_late_accepted_canceled_goal_is_canceled_again(self):
        handle = Mock(accepted=True)
        Manager._on_goal_accepted(self.n, Mock(result=lambda: handle), (9,9), 2)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.n._active_goal, (1.0, 2.0))

    def test_aborted_goal_is_blacklisted_instead_of_treated_as_cancel(self):
        Manager._on_goal_result(self.n, Mock(result=lambda: NS(status=6)), 3)
        self.assertEqual(self.n._failed_goals, [(1.0,2.0)])

    def test_completed_goal_is_remembered(self):
        Manager._on_goal_result(self.n, Mock(result=lambda: NS(status=4)), 3)
        self.assertEqual(self.n._visited_positions, [(1.0,2.0)])

    def test_timeout_blacklists_before_clearing_goal(self):
        self.n._exploring = self.n._autonomous = True
        self.n._user_goal_until = 0
        self.n._record_visited_position = Mock()
        self.n._pending_frontier = None
        self.n._goal_timeout = 1
        self.n._goal_sent_at = time.monotonic()-2
        Manager._explore_tick(self.n)
        self.assertEqual(self.n._failed_goals, [(1.0,2.0)])
        self.assertIsNone(self.n._active_goal)

    def test_pending_acceptance_does_not_start_another_goal(self):
        self.n._exploring = self.n._autonomous = True
        self.n._user_goal_until = 0
        self.n._record_visited_position = Mock()
        self.n._pending_frontier = (9,9)
        self.n._goal_handle = None
        self.n._goal_timeout = 45
        self.n._send_goal = Mock()
        Manager._explore_tick(self.n)
        self.n._send_goal.assert_not_called()

    def test_disabled_fallback_never_starts_unplanned_motion(self):
        self.n._direct_fallback_enabled = False
        self.n._direct_explore_active = True
        Manager._enable_direct_explore(self.n, 'test')
        self.assertFalse(self.n._direct_explore_active)


if __name__ == '__main__':
    unittest.main()
