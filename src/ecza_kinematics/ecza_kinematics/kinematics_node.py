"""
kinematics_node.py — cmd_vel → wheel_velocities for a 4-wheel mecanum rover.

Mecanum inverse kinematics:
  Given a desired body velocity (vx, vy, ωz), compute the angular velocity
  of each wheel (rad/s).

  Wheel layout (top-down view, X forward):
    FL (+x, +y)   FR (+x, -y)
    RL (-x, +y)   RR (-x, -y)

  Roller orientation (O-config, rollers at ±45°):
    FL and RR: rollers at +45° (top-left to bottom-right diagonal)
    FR and RL: rollers at -45° (top-right to bottom-left diagonal)

  Physical wheel spin directions (vy sign convention: REP 103 / ROS Twist —
  vy>0=left, vy<0=right — matches ros2_controllers' mecanum_drive_controller):
    Forward  (vx>0):        FL+ FR+ RL+ RR+
    Strafe L (vy>0):        FL- FR+ RL+ RR-
    Strafe R (vy<0):        FL+ FR- RL- RR+
    CCW rot  (wz>0):        FL- FR+ RL- RR+
    CW  rot  (wz<0):        FL+ FR- RL+ RR-

  Equations (must match mecanum_drive_controller.cpp's FRONT_LEFT/FRONT_RIGHT/
  REAR_LEFT/REAR_RIGHT formulas exactly, since that's the node actually driving
  the rover — this node is a visualization/reference-only path):
    ω_FL = (1/r) * ( vx - vy - (lx+ly)*wz)
    ω_FR = (1/r) * ( vx + vy + (lx+ly)*wz)
    ω_RL = (1/r) * ( vx + vy - (lx+ly)*wz)
    ω_RR = (1/r) * ( vx - vy + (lx+ly)*wz)

  where:
    r   = wheel_radius (m)
    lx  = wheel_base / 2          (half front-rear distance)
    ly  = wheel_separation_width / 2  (half left-right distance)

Published topic: /wheel_velocities (std_msgs/Float32MultiArray)
  data = [FL, FR, RL, RR]  (rad/s, positive = forward)

Parameters (from rover_params.yaml):
  wheel_radius           (float, default 0.08) m
  wheel_base             (float, default 0.38) m
  wheel_separation_width (float, default 0.26) m
  max_linear_speed       (float, default 0.5)  m/s  — used for clamping
  max_angular_speed      (float, default 1.0)  rad/s
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


class MecanumKinematicsNode(Node):
    def __init__(self) -> None:
        super().__init__("kinematics_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("wheel_radius", 0.08)  # must match rover_params.yaml
        self.declare_parameter("wheel_base", 0.38)
        self.declare_parameter("wheel_separation_width", 0.26)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)

        r = self.get_parameter("wheel_radius").value
        wb = self.get_parameter("wheel_base").value
        ws = self.get_parameter("wheel_separation_width").value

        self._r = r
        self._lx = wb / 2.0
        self._ly = ws / 2.0

        # ── Publishers / Subscribers ──────────────────────────────────────
        self._wheel_pub = self.create_publisher(
            Float32MultiArray, "wheel_velocities", 10
        )
        self._cmd_sub = self.create_subscription(
            Twist, "cmd_vel", self._cmd_cb, 10
        )

        # ── Joint state integration for RViz wheel animation ──────────────
        # Integrates wheel_velocities → joint positions so wheels visually
        # spin in RViz proportional to the commanded velocity.
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._joint_names = [
            "fl_wheel_joint", "fr_wheel_joint",
            "rl_wheel_joint", "rr_wheel_joint",
        ]
        self._joint_pos = [0.0, 0.0, 0.0, 0.0]
        self._last_wheel_vel = [0.0, 0.0, 0.0, 0.0]
        self._last_time = self.get_clock().now()
        # Timer at 30 Hz to integrate and publish joint states
        self.create_timer(1.0 / 30.0, self._joint_timer_cb)

        self.get_logger().info(
            f"mecanum kinematics ready — r={self._r}m, "
            f"lx={self._lx}m, ly={self._ly}m"
        )

    def _cmd_cb(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        r = self._r
        k = self._lx + self._ly  # combined half-wheelbase factor

        # Inverse kinematics — rad/s for each wheel
        # vy convention: REP 103 / ROS Twist — vy > 0 = strafe left, vy < 0 = strafe right
        #   Strafe Left  (vy>0): FL- FR+ RL+ RR-
        #   Strafe Right (vy<0): FL+ FR- RL- RR+
        # Keep these signs aligned with the hardware path:
        # mecanum_drive_controller -> wheel_bridge -> ESP32 -> /wheel_velocities.
        fl = ( vx - vy - k * wz) / r
        fr = ( vx + vy + k * wz) / r
        rl = ( vx + vy - k * wz) / r
        rr = ( vx - vy + k * wz) / r

        self._last_wheel_vel = [fl, fr, rl, rr]

        out = Float32MultiArray()
        out.data = [float(fl), float(fr), float(rl), float(rr)]
        self._wheel_pub.publish(out)

    def _joint_timer_cb(self) -> None:
        """Integrate wheel velocities into joint positions and publish."""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        for i, vel in enumerate(self._last_wheel_vel):
            self._joint_pos[i] += vel * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self._joint_names
        js.position = list(self._joint_pos)
        js.velocity = list(self._last_wheel_vel)
        self._joint_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
