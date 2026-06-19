#!/usr/bin/env python3
"""Re-stamp Isaac Sim /tf messages with wall-clock time.

Isaac Sim publishes transforms (world→base_footprint, wheel joints) with
simulation time (seconds since Play ~0-10000 s).  RPi nodes use wall-clock
time (~1.78 × 10⁹ s Unix epoch).  TF2 cannot match these timestamps, so
SLAM and Nav2 costmap lookups fail silently.

This node intercepts transforms whose timestamps are clearly simulation time
(header.stamp.sec < SIM_TIME_THRESHOLD) and whose parent frame is a frame
that Isaac Sim owns (world), then re-publishes them stamped at
clock.now() (wall clock, because use_sim_time=false).

Re-published transforms still carry the same child/parent frame IDs, so the
TF chain  odom(static)→world→base_footprint→base_link→laser_frame  works.

Loop prevention: once re-stamped, the same message comes back with a wall-
clock stamp (> SIM_TIME_THRESHOLD), so it is not re-published a second time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage

# Isaac Sim sim time is seconds since Play-pressed, typically < 1e6.
# Wall clock (Unix time) is ~1.78e9.  A threshold of 1e8 (~3 years since epoch)
# safely separates the two.
SIM_TIME_THRESHOLD = 100_000_000

# Re-stamp transforms whose *parent* frame is owned by Isaac Sim.
# 'base_footprint' = wheel joint dynamics (re-stamped so RViz shows spinning wheels).
# 'world' = ground-truth robot pose (world→base_footprint) published by Isaac Sim.
ISAAC_SIM_PARENT_FRAMES = {'base_footprint', 'world'}


class TFRestamper(Node):
    def __init__(self) -> None:
        super().__init__('tf_restamper_sim')

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub = self.create_publisher(TFMessage, '/tf', qos)
        self.create_subscription(TFMessage, '/tf', self._tf_cb, qos)

        self.get_logger().info(
            'tf_restamper_sim: re-stamping Isaac Sim TF (world→*, base_footprint→*) to wall-clock time'
        )

    def _tf_cb(self, msg: TFMessage) -> None:
        now = self.get_clock().now().to_msg()
        restamped = []

        for t in msg.transforms:
            stamp_sec = t.header.stamp.sec
            if (
                stamp_sec < SIM_TIME_THRESHOLD
                and t.header.frame_id in ISAAC_SIM_PARENT_FRAMES
            ):
                self.get_logger().info(f"Restamping {t.header.frame_id}->{t.child_frame_id}: sim_time={stamp_sec}.{t.header.stamp.nanosec:09d} -> wall_time={now.sec}.{now.nanosec:09d}", throttle_duration_sec=1.0)
                t.header.stamp = now
                restamped.append(t)

        if restamped:
            out = TFMessage()
            out.transforms = restamped
            self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
