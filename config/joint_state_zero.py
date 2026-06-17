#!/usr/bin/env python3
"""Publishes zero-position /joint_states for the 4 wheel joints.

robot_state_publisher cannot publish TF for continuous joints without
receiving /joint_states. In sim mode there is no ros2_control, so this
lightweight node takes its place — wheels stay at position 0 (unspun)
which is fine for SLAM/Nav2 visualization.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEEL_JOINTS = [
    "fl_wheel_joint",
    "fr_wheel_joint",
    "rl_wheel_joint",
    "rr_wheel_joint",
]


class JointStateZero(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_zero")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self._cb)

    def _cb(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_JOINTS
        msg.position = [0.0] * len(WHEEL_JOINTS)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateZero()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
