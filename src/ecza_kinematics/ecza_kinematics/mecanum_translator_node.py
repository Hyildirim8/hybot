"""
mecanum_translator_node.py

Task 2 implementation: /cmd_vel -> per-wheel angular velocity commands.

Inputs:
  - /cmd_vel (geometry_msgs/msg/Twist)
Outputs:
  - /wheel_cmd_rad_s (std_msgs/msg/Float64MultiArray)
    order: [FL, FR, RL, RR], unit: rad/s

Wheel order used everywhere:
  1 FL, 2 FR, 3 RL, 4 RR

Mecanum inverse kinematics (standard X-forward, Y-left frame):
  w_fl = (vx - vy - (lx + ly) * wz) / r
  w_fr = (vx + vy + (lx + ly) * wz) / r
  w_rl = (vx + vy - (lx + ly) * wz) / r
  w_rr = (vx - vy + (lx + ly) * wz) / r

The signs match these canonical motions:
  Forward:       FL+, FR+, RL+, RR+
  Backward:      FL-, FR-, RL-, RR-
  Right strafe:  FL+, FR-, RL-, RR+
  Left strafe:   FL-, FR+, RL+, RR-
  Turn right:    FL+, FR-, RL+, RR-
  Turn left:     FL-, FR+, RL-, RR+

Diagonal and arc trajectories naturally emerge from simultaneous vx/vy/wz terms.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MecanumTranslatorNode(Node):
    def __init__(self) -> None:
        super().__init__("mecanum_translator_node")

        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_base", 0.38)
        self.declare_parameter("wheel_separation_width", 0.26)
        self.declare_parameter("max_wheel_rad_s", 30.0)

        self._r = float(self.get_parameter("wheel_radius").value)
        self._lx = float(self.get_parameter("wheel_base").value) / 2.0
        self._ly = float(self.get_parameter("wheel_separation_width").value) / 2.0
        self._k = self._lx + self._ly
        self._max_w = float(self.get_parameter("max_wheel_rad_s").value)

        self._pub = self.create_publisher(Float64MultiArray, "/wheel_cmd_rad_s", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)

        self.get_logger().info(
            f"mecanum_translator_node ready r={self._r:.3f} lx={self._lx:.3f} ly={self._ly:.3f} max_w={self._max_w:.1f}"
        )

    def _clip(self, value: float) -> float:
        return max(-self._max_w, min(self._max_w, value))

    def _cmd_cb(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        fl = (vx - vy - self._k * wz) / self._r
        fr = (vx + vy + self._k * wz) / self._r
        rl = (vx + vy - self._k * wz) / self._r
        rr = (vx - vy + self._k * wz) / self._r

        out = Float64MultiArray()
        out.data = [
            self._clip(fl),
            self._clip(fr),
            self._clip(rl),
            self._clip(rr),
        ]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumTranslatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
