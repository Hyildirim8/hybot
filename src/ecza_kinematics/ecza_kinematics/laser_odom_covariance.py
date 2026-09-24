"""Give RF2O's unpopulated covariance a nonzero uncertainty before EKF fusion."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


def with_covariance_floor(msg, position_variance, yaw_variance):
    out = copy.deepcopy(msg)
    # RF2O currently leaves all covariance entries at zero. These are tuning
    # floors (5 cm position / ~8.6 deg yaw), not measured accuracy claims.
    for index, floor in ((0, position_variance), (7, position_variance),
                         (14, 1.0), (21, 1.0), (28, 1.0), (35, yaw_variance)):
        value = out.pose.covariance[index]
        out.pose.covariance[index] = max(value, floor) if math.isfinite(value) else floor
    # Twist is not used by this robot's EKF, but must not advertise certainty
    # to another consumer either (RF2O does not estimate lateral velocity).
    for index, floor in ((0, 0.04), (7, 1.0), (14, 1.0),
                         (21, 1.0), (28, 1.0), (35, 0.09)):
        value = out.twist.covariance[index]
        out.twist.covariance[index] = max(value, floor) if math.isfinite(value) else floor
    return out


class LaserOdomCovariance(Node):
    def __init__(self):
        super().__init__('laser_odom_covariance')
        self.declare_parameter('position_variance', 0.0025)
        self.declare_parameter('yaw_variance', 0.0225)
        self._position_variance = float(self.get_parameter('position_variance').value)
        self._yaw_variance = float(self.get_parameter('yaw_variance').value)
        if self._position_variance <= 0.0 or self._yaw_variance <= 0.0:
            raise ValueError('Covariance floors must be positive')
        self._pub = self.create_publisher(Odometry, '/odom_rf2o', 5)
        self.create_subscription(Odometry, '/odom_rf2o_raw', self._callback,
                                 qos_profile_sensor_data)

    def _callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            self.get_logger().warn('Ignoring nonfinite RF2O pose', throttle_duration_sec=5.0)
            return
        self._pub.publish(with_covariance_floor(msg, self._position_variance,
                                               self._yaw_variance))


def main():
    rclpy.init()
    node = LaserOdomCovariance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
