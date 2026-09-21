#!/usr/bin/env python3
"""Send a Nav2 NavigateToPose goal for a 2D pose in the map frame."""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Goal x in map frame [m]')
    parser.add_argument('--y', type=float, required=True, help='Goal y in map frame [m]')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in map frame [rad]')
    parser.add_argument('--frame', default='map', help='Pose frame_id')
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('navigate_to_pose_client')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for /navigate_to_pose...')
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error('navigate_to_pose action server is not available')
        rclpy.shutdown()
        return 1

    qx, qy, qz, qw = yaw_to_quaternion(args.yaw)
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = args.frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.position.z = 0.0
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    node.get_logger().info(
        f'Sending goal x={args.x:.2f} y={args.y:.2f} yaw={args.yaw:.2f}')
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('Goal was rejected')
        rclpy.shutdown()
        return 1

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    status = result_future.result().status
    node.get_logger().info(f'Navigation finished with status {status}')
    rclpy.shutdown()
    return 0 if status == 4 else 1


if __name__ == '__main__':
    sys.exit(main())
