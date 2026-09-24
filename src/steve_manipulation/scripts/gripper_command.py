#!/usr/bin/env python3
"""Send a GripperCommand to the Robotiq 2F-85.

  ros2 run steve_manipulation gripper_command.py open
  ros2 run steve_manipulation gripper_command.py close
  ros2 run steve_manipulation gripper_command.py --position 0.4

Joint units are radians of the left knuckle: 0.0 = open, ~0.79 = fully closed.
"""

import argparse
import sys

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        nargs="?",
        choices=["open", "close"],
        help="Named gripper command",
    )
    parser.add_argument(
        "--position",
        type=float,
        default=None,
        help="Left knuckle position in radians (overrides open/close)",
    )
    parser.add_argument("--effort", type=float, default=20.0)
    parser.add_argument(
        "--action",
        default="/robotiq_gripper_controller/gripper_cmd",
        help="GripperCommand action name",
    )
    args = parser.parse_args(argv)

    if args.position is None:
        if args.command == "close":
            position = 0.79
        else:
            position = 0.0
    else:
        position = args.position

    rclpy.init()
    node = rclpy.create_node("steve_gripper_command")
    client = ActionClient(node, GripperCommand, args.action)

    node.get_logger().info(f"Waiting for {args.action}...")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(
            "Gripper action server is not available. "
            "Is robotiq_gripper_controller spawned?"
        )
        rclpy.shutdown()
        return 1

    goal = GripperCommand.Goal()
    goal.command.position = position
    goal.command.max_effort = args.effort
    node.get_logger().info(f"Commanding gripper position={position:.3f} rad")

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("Gripper goal was rejected")
        rclpy.shutdown()
        return 1

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    node.get_logger().info(
        f"Reached={result.reached_goal} stalled={result.stalled} "
        f"position={result.position:.3f}"
    )
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
