#!/usr/bin/env python3
"""Send a GripperCommand to the Robotiq 2F-85.

  ros2 run steve_manipulation gripper_command.py open
  ros2 run steve_manipulation gripper_command.py close
  ros2 run steve_manipulation gripper_command.py --position 0.4

Joint units are radians of the left knuckle: 0.0 = open, ~0.79 = fully closed.
"""

import argparse
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState


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
    parser.add_argument("--effort", type=float, default=8.0)
    parser.add_argument(
        "--speed",
        type=float,
        default=0.12,
        help="Knuckle speed in rad/s. The controller has no velocity limit, so this walks the setpoint.",
    )
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

    current = read_knuckle(node)
    node.get_logger().info(
        f"Gripper {current:.3f} -> {position:.3f} rad at {args.speed:.2f} rad/s"
    )
    for waypoint, last in waypoints(current, position, args.speed):
        goal = GripperCommand.Goal()
        goal.command.position = waypoint
        goal.command.max_effort = args.effort
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            node.get_logger().error("Gripper goal was rejected")
            rclpy.shutdown()
            return 1
        if not last:
            pace(node, 0.02 / args.speed if args.speed > 0.0 else 0.0)
            continue
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        node.get_logger().info(
            f"Reached={result.reached_goal} stalled={result.stalled} "
            f"position={result.position:.3f}"
        )
    rclpy.shutdown()
    return 0


def read_knuckle(node, timeout=1.0) -> float:
    holder = {"pos": None}

    def callback(msg: JointState):
        if "robotiq_85_left_knuckle_joint" not in msg.name:
            return
        index = msg.name.index("robotiq_85_left_knuckle_joint")
        holder["pos"] = float(msg.position[index])

    subs = [
        node.create_subscription(JointState, "/joint_states", callback, 10),
        node.create_subscription(JointState, "/joint_states_complete", callback, 10),
    ]
    end = time.monotonic() + timeout
    while holder["pos"] is None and time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    for sub in subs:
        node.destroy_subscription(sub)
    return 0.0 if holder["pos"] is None else holder["pos"]


def waypoints(current: float, position: float, speed: float):
    distance = position - current
    if abs(distance) < 0.01 or speed <= 0.0:
        yield position, True
        return
    step = 0.02 if distance > 0.0 else -0.02
    cursor = current
    points = []
    while (step > 0.0 and cursor + step < position) or (step < 0.0 and cursor + step > position):
        cursor += step
        points.append(cursor)
    points.append(position)
    for index, point in enumerate(points):
        yield point, index == len(points) - 1


def pace(node, seconds: float):
    end = time.monotonic() + max(seconds, 0.0)
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


if __name__ == "__main__":
    sys.exit(main())
