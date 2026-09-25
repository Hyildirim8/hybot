"""Exercise the real A -> exploration -> AUTO subscription without motor access.

All motion, mode, joystick, map and cancellation endpoints are remapped to
an isolated namespace. ROS_DOMAIN_ID is left unchanged.
"""
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ecza_teleop.teleop_node import TeleopNode
from ecza_teleop.slam_manager_node import SlamManagerNode


class ModeHandoffROSTest(unittest.TestCase):
    def test_a_enters_exploration_and_start_returns_to_manual(self):
        prefix = '/ecza_mode_handoff_test'
        isolated = [
            '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_nav_smoothed',
            '/controller_manager/reference_unstamped', '/slam_manager/exploring',
            '/goal_pose', '/initialpose', '/map', '/scan', '/odometry/filtered',
            '/navigate_to_pose/_action/cancel_goal',
            '/navigate_through_poses/_action/cancel_goal',
            '/slam_toolbox/save_map', '/map_saver/save_map',
        ]
        args = ['--ros-args', '-r', '__ns:='+prefix,
                '-p', 'start_in_autonomous:=false',
                '-p', 'reject_extreme_axis_startup:=false']
        for topic in isolated:
            args += ['-r', topic+':='+prefix+topic]
        rclpy.init(args=args)
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            teleop = TeleopNode(); nodes.append(teleop)
            manager = SlamManagerNode(); nodes.append(manager)
            probe = rclpy.create_node('probe'); nodes.append(probe)
            for node in nodes:
                executor.add_node(node)
            outputs = []
            probe.create_subscription(Twist, prefix+'/cmd_vel', outputs.append, 10)
            joy = probe.create_publisher(Joy, 'joy', 10)

            def wait_for(predicate, timeout=5):
                end = time.monotonic()+timeout
                while not predicate() and time.monotonic()<end:
                    executor.spin_once(timeout_sec=0.05)
                self.assertTrue(predicate())

            def spin_for(seconds):
                end = time.monotonic()+seconds
                while time.monotonic()<end:
                    executor.spin_once(timeout_sec=0.05)

            def press(index):
                msg = Joy(); msg.axes = [0.0]*6; msg.buttons = [0]*12
                if index is not None:
                    msg.buttons[index] = 1
                joy.publish(msg)
                spin_for(0.25)

            wait_for(lambda: joy.get_subscription_count() >= 2)
            spin_for(0.3)
            self.assertFalse(teleop._autonomous)
            press(1)  # A: no navigation velocity exists in this test.
            wait_for(lambda: manager._exploring and manager._autonomous and teleop._autonomous)
            self.assertFalse(manager._direct_explore_active)
            press(None)
            spin_for(0.5)
            press(9)  # Start: explicit manual takeover must cancel discovery.
            wait_for(lambda: not manager._exploring and not teleop._autonomous)
            self.assertTrue(teleop._manual_teleop_lock)
            self.assertTrue(outputs)
            self.assertTrue(all(m.linear.x == m.linear.y == m.angular.z == 0 for m in outputs))
        finally:
            executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
