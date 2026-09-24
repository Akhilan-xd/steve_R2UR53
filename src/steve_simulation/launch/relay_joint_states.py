#!/usr/bin/env python3
"""Republish /joint_states on a reliable topic.

joint_state_broadcaster publishes with the system-default QoS, which is
best-effort on this setup. joint_state_publisher subscribes reliably, so it
never sees the arm move and keeps the spawn pose forever. MoveIt then plans
the next motion from that stale pose and the trajectory controller aborts
with PATH_TOLERANCE_VIOLATED.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(JointState, "/joint_states_hw", reliable)
        self.create_subscription(JointState, "/joint_states", self.forward, best_effort)

    def forward(self, msg: JointState):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
