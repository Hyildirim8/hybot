"""
encoder_verifier_node.py

Task 3 implementation: compare expected wheel speed from /cmd_vel against
actual wheel speed from /joint_states and warn on direction or speed mismatch.

Inputs:
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /joint_states (sensor_msgs/msg/JointState)

Checks:
  - Direction mismatch: expected sign != actual sign (beyond deadband)
  - Speed mismatch: abs(actual - expected) / max(abs(expected), eps) > tolerance

Warnings help identify:
  - motor polarity wiring errors (wrong sign)
  - mechanical binding / slip (large speed error)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class WheelState:
    expected: float = 0.0
    actual: float = 0.0


class EncoderVerifierNode(Node):
    def __init__(self) -> None:
        super().__init__("encoder_verifier_node")

        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_base", 0.38)
        self.declare_parameter("wheel_separation_width", 0.26)
        self.declare_parameter("tolerance_percent", 5.0)
        self.declare_parameter("min_expected_rad_s", 0.3)
        self.declare_parameter("warn_period_sec", 1.0)

        self._r = float(self.get_parameter("wheel_radius").value)
        self._lx = float(self.get_parameter("wheel_base").value) / 2.0
        self._ly = float(self.get_parameter("wheel_separation_width").value) / 2.0
        self._k = self._lx + self._ly

        tol_pct = float(self.get_parameter("tolerance_percent").value)
        self._tol = max(0.0, tol_pct) / 100.0
        self._min_expected = float(self.get_parameter("min_expected_rad_s").value)
        self._warn_period = float(self.get_parameter("warn_period_sec").value)

        self._wheels: Dict[str, WheelState] = {
            "fl_wheel_joint": WheelState(),
            "fr_wheel_joint": WheelState(),
            "rl_wheel_joint": WheelState(),
            "rr_wheel_joint": WheelState(),
        }
        self._last_warn: Dict[str, float] = {k: 0.0 for k in self._wheels}

        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)

        self.get_logger().info(
            f"encoder_verifier_node ready tol={tol_pct:.1f}% min_expected={self._min_expected:.2f}rad/s"
        )

    @staticmethod
    def _sgn(v: float, eps: float = 1e-6) -> int:
        if v > eps:
            return 1
        if v < -eps:
            return -1
        return 0

    def _cmd_cb(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        # Keep identical to mecanum translator equation for apples-to-apples checks.
        self._wheels["fl_wheel_joint"].expected = (vx - vy - self._k * wz) / self._r
        self._wheels["fr_wheel_joint"].expected = (vx + vy + self._k * wz) / self._r
        self._wheels["rl_wheel_joint"].expected = (vx + vy - self._k * wz) / self._r
        self._wheels["rr_wheel_joint"].expected = (vx - vy + self._k * wz) / self._r

    def _joint_cb(self, msg: JointState) -> None:
        if not msg.name or not msg.velocity:
            return

        idx_by_name = {name: i for i, name in enumerate(msg.name)}
        now = self.get_clock().now().nanoseconds * 1e-9

        for joint_name, wheel in self._wheels.items():
            idx = idx_by_name.get(joint_name)
            if idx is None or idx >= len(msg.velocity):
                continue

            wheel.actual = float(msg.velocity[idx])
            expected = wheel.expected
            actual = wheel.actual

            if abs(expected) < self._min_expected:
                continue

            # 1) Direction check
            exp_sign = self._sgn(expected)
            act_sign = self._sgn(actual)
            if act_sign != 0 and exp_sign != act_sign:
                self._warn_throttled(
                    joint_name,
                    now,
                    f"direction mismatch expected={expected:+.3f}rad/s actual={actual:+.3f}rad/s "
                    f"(possible motor polarity wiring issue)",
                )
                continue

            # 2) Magnitude check
            denom = max(abs(expected), 1e-6)
            rel_err = abs(actual - expected) / denom
            if rel_err > self._tol:
                self._warn_throttled(
                    joint_name,
                    now,
                    f"speed mismatch expected={expected:+.3f}rad/s actual={actual:+.3f}rad/s "
                    f"error={rel_err*100:.1f}% > tol={self._tol*100:.1f}% "
                    f"(possible slip/binding)",
                )

    def _warn_throttled(self, key: str, now: float, message: str) -> None:
        if now - self._last_warn[key] >= self._warn_period:
            self.get_logger().warn(f"[{key}] {message}")
            self._last_warn[key] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderVerifierNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
