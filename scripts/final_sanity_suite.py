import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

TH = 0.7


def sign4(v):
    out = []
    for x in v:
        if abs(x) < TH:
            out.append(0)
        elif x > 0:
            out.append(1)
        else:
            out.append(-1)
    return out


class N(Node):
    def __init__(self):
        super().__init__("final_sanity_suite")
        self.pub = self.create_publisher(Twist, "/controller_manager/reference_unstamped", 10)
        self.enc = []
        self.odom = []
        self.create_subscription(
            Float32MultiArray,
            "/wheel_velocities",
            lambda m: self.enc.append(list(m.data)),
            10,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            lambda m: self.odom.append((m.twist.twist.linear.x, m.twist.twist.angular.z)),
            10,
        )


def run_case(n, name, tw, sec=2.0):
    n.enc.clear()
    n.odom.clear()
    st = time.time()
    while time.time() - st < sec:
        n.pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.02)

    em = np.array(n.enc[-70:]) if len(n.enc) >= 5 else np.zeros((1, 4))
    om = np.array(n.odom[-70:]) if len(n.odom) >= 5 else np.zeros((1, 2))
    m = em.mean(axis=0)
    o = om.mean(axis=0)
    print(
        f"{name}: sign={sign4(m)} "
        f"mean={[round(float(x), 2) for x in m]} "
        f"odom_vx_wz={[round(float(x), 4) for x in o]}"
    )

    z = Twist()
    zt = time.time()
    while time.time() - zt < 0.8:
        n.pub.publish(z)
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.02)


def main():
    rclpy.init()
    n = N()
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    f = Twist()
    f.linear.x = 0.25
    l = Twist()
    l.linear.y = 0.35
    r = Twist()
    r.linear.y = -0.35
    t = Twist()
    t.angular.z = -0.9

    run_case(n, "FORWARD", f)
    run_case(n, "STRAFE_LEFT", l)
    run_case(n, "STRAFE_RIGHT", r)
    run_case(n, "TURN_RIGHT", t)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
