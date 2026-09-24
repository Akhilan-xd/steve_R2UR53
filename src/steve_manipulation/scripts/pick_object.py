#!/usr/bin/env python3
"""Move Steve's UR5e with MoveIt 2, then optionally run a known-pose pick.

This talks to the move_group action server the same way the RViz MotionPlanning
plugin does. Joint-space named poses are the reliable first step; Cartesian
pose goals need a reachable (x, y, z, orientation) for gripper_tcp.

Examples:

  ros2 run steve_manipulation pick_object.py --named ready
  ros2 run steve_manipulation pick_object.py --named home
  ros2 run steve_manipulation pick_object.py --pick

Teach a new ready pose from RViz (Plan + Execute first, then):

  ros2 run steve_manipulation pick_object.py --dump
  ros2 run steve_manipulation pick_object.py --save-ready
"""

import argparse
import re
import sys
import time
from pathlib import Path

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

MOVEIT_ERRORS = {
    getattr(MoveItErrorCodes, name): name
    for name in (
        "SUCCESS",
        "FAILURE",
        "PLANNING_FAILED",
        "INVALID_MOTION_PLAN",
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        "CONTROL_FAILED",
        "TIMED_OUT",
        "PREEMPTED",
        "START_STATE_IN_COLLISION",
        "GOAL_IN_COLLISION",
        "INVALID_GOAL_CONSTRAINTS",
        "INVALID_LINK_NAME",
        "NO_IK_SOLUTION",
    )
    if hasattr(MoveItErrorCodes, name)
}


# Keep in sync with config/mmo_700.srdf <group_state> entries.
NAMED_POSES = {
    "home": {
        "ur5eshoulder_pan_joint": 0.0,
        "ur5eshoulder_lift_joint": -1.5708,
        "ur5eelbow_joint": 2.7,
        "ur5ewrist_1_joint": -1.2,
        "ur5ewrist_2_joint": 0.0,
        "ur5ewrist_3_joint": 0.0,
    },
    "ready": {
        "ur5eshoulder_pan_joint": 0.0,
        "ur5eshoulder_lift_joint": -1.2,
        "ur5eelbow_joint": 1.9,
        "ur5ewrist_1_joint": -1.57,
        "ur5ewrist_2_joint": 1.57,
        "ur5ewrist_3_joint": 0.0,
    },
}

ARM_JOINTS = list(NAMED_POSES["home"].keys())
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.79
EE_LINK = "gripper_tcp"
GROUP = "ur_manipulator"
CUBE_ID = "pick_cube"
GRIPPER_TOUCH_LINKS = [
    "gripper_tcp",
    "gripper_mount_link",
    "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_right_knuckle_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_right_finger_tip_link",
    "robotiq_85_right_inner_knuckle_link",
]


def format_joint_dict(joints: dict) -> dict:
    return {name: round(float(joints[name]), 4) for name in ARM_JOINTS}


def pose_as_python(joints: dict) -> str:
    lines = ['    "ready": {']
    for name in ARM_JOINTS:
        lines.append(f'        "{name}": {joints[name]},')
    lines.append("    },")
    return "\n".join(lines)


def pose_as_srdf(joints: dict) -> str:
    lines = ['  <group_state name="ready" group="ur_manipulator">']
    for name in ARM_JOINTS:
        lines.append(f'    <joint name="{name}" value="{joints[name]}"/>')
    lines.append("  </group_state>")
    return "\n".join(lines)


def read_arm_joints(node, timeout=5.0) -> dict:
    """Read the six UR5e joints from the live robot (after RViz Execute)."""
    holder = {"msg": None}

    def callback(msg: JointState):
        holder["msg"] = msg

    node.create_subscription(JointState, "/joint_states_complete", callback, 10)
    node.create_subscription(JointState, "/joint_states", callback, 10)
    end = time.time() + timeout
    while holder["msg"] is None and time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    if holder["msg"] is None:
        raise RuntimeError(
            "No /joint_states yet. Is simulation running, and did you Execute in RViz?"
        )

    by_name = dict(zip(holder["msg"].name, holder["msg"].position))
    missing = [name for name in ARM_JOINTS if name not in by_name]
    if missing:
        raise RuntimeError(f"Joint state is missing: {missing}")
    return format_joint_dict(by_name)


def save_ready_pose(joints: dict) -> None:
    """Write the captured ready pose into pick_object.py and mmo_700.srdf."""
    script_path = Path(__file__).resolve()
    script = script_path.read_text()
    script, n_py = re.subn(
        r'    "ready": \{.*?\n    \},',
        pose_as_python(joints),
        script,
        count=1,
        flags=re.DOTALL,
    )
    if n_py != 1:
        raise RuntimeError(f"Could not update ready pose in {script_path}")
    script_path.write_text(script)

    from ament_index_python.packages import get_package_share_directory

    srdf_path = Path(get_package_share_directory("steve_manipulation")) / "config" / "mmo_700.srdf"
    srdf = srdf_path.read_text()
    srdf, n_srdf = re.subn(
        r'  <group_state name="ready" group="ur_manipulator">.*?</group_state>',
        pose_as_srdf(joints),
        srdf,
        count=1,
        flags=re.DOTALL,
    )
    if n_srdf != 1:
        raise RuntimeError(f"Could not update ready pose in {srdf_path}")
    srdf_path.write_text(srdf)

    print(f"Updated ready pose in:\n  {script_path}\n  {srdf_path}")


class SteveArm:
    def __init__(self, node):
        self.node = node
        self.move = ActionClient(node, MoveGroup, "move_action")
        self.cartesian = node.create_client(GetCartesianPath, "compute_cartesian_path")
        self.execute = ActionClient(node, ExecuteTrajectory, "execute_trajectory")
        self.gripper = ActionClient(
            node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )
        self.scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wait(self, timeout=30.0):
        self.node.get_logger().info("Waiting for /move_action (move_group)...")
        if not self.move.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("move_group is not running. Launch steve_manipulation first.")
        self.node.get_logger().info("Waiting for gripper action...")
        if not self.gripper.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().warn(
                "Gripper action not available; arm motions will still work."
            )

    def _send_move(self, request: MotionPlanRequest, plan_only=False) -> bool:
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.replan_delay = 0.5

        send_future = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("MoveGroup rejected the goal")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        error = result.error_code.val
        if error != MoveItErrorCodes.SUCCESS:
            name = MOVEIT_ERRORS.get(error, "UNKNOWN")
            self.node.get_logger().error(
                f"MoveGroup failed with {name} ({error}). "
                "CONTROL_FAILED usually means Gazebo lagged the planned path."
            )
            return False
        self.node.get_logger().info("Motion succeeded")
        return True

    def _base_request(self) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.05
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = "base_link"
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -0.2
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 1.8
        return req

    def move_named(self, name: str) -> bool:
        if name not in NAMED_POSES:
            raise ValueError(f"Unknown pose '{name}'. Try: {list(NAMED_POSES)}")
        self.node.get_logger().info(f"Planning to named pose '{name}'")
        req = self._base_request()
        constraints = Constraints()
        constraints.name = name
        for joint, value in NAMED_POSES[name].items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        return self._send_move(req)

    def move_pose(self, pose: PoseStamped) -> bool:
        self.node.get_logger().info(
            f"Planning to pose in {pose.header.frame_id}: "
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f})"
        )
        req = self._base_request()
        constraints = Constraints()
        constraints.name = "ee_pose"

        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = EE_LINK
        pos.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose.pose)

        ori = OrientationConstraint()
        ori.header = pose.header
        ori.link_name = EE_LINK
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.15
        ori.absolute_y_axis_tolerance = 0.15
        ori.absolute_z_axis_tolerance = 0.15
        ori.weight = 1.0

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)
        req.goal_constraints.append(constraints)
        return self._send_move(req)

    def cartesian_to(self, pose: PoseStamped, step=0.01) -> bool:
        if not self.cartesian.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("compute_cartesian_path missing; using pose goal")
            return self.move_pose(pose)

        request = GetCartesianPath.Request()
        request.header = pose.header
        request.start_state.is_diff = True
        request.group_name = GROUP
        request.link_name = EE_LINK
        request.waypoints.append(pose.pose)
        request.max_step = step
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        future = self.cartesian.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or response.fraction < 0.9:
            frac = 0.0 if response is None else response.fraction
            self.node.get_logger().warn(
                f"Cartesian path only covered {frac:.2f}; falling back to pose goal"
            )
            return self.move_pose(pose)

        if not self.execute.wait_for_server(timeout_sec=5.0):
            return self.move_pose(pose)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        send_future = self.execute.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return self.move_pose(pose)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        error = MoveItErrorCodes.FAILURE if result is None else result.result.error_code.val
        if error != MoveItErrorCodes.SUCCESS:
            name = MOVEIT_ERRORS.get(error, "UNKNOWN")
            self.node.get_logger().error(
                f"Cartesian execution failed with {name} ({error})"
            )
            return False
        return True

    def set_gripper(self, position: float, effort: float = 20.0) -> bool:
        if not self.gripper.server_is_ready():
            self.node.get_logger().warn("Skipping gripper command (server missing)")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        send_future = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("Gripper goal rejected")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        return True

    def attach_cube(self) -> bool:
        """Treat the cube as grasped so lift does not collide with it on the stand."""
        if not self.scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("No /apply_planning_scene; skip attaching cube")
            return False

        attached = AttachedCollisionObject()
        attached.link_name = EE_LINK
        attached.object.id = CUBE_ID
        attached.object.header.frame_id = EE_LINK
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, 0.04]
        attached.object.primitives.append(primitive)
        pose = Pose()
        pose.orientation.w = 1.0
        attached.object.primitive_poses.append(pose)
        attached.touch_links = list(GRIPPER_TOUCH_LINKS)

        remove = CollisionObject()
        remove.id = CUBE_ID
        remove.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(remove)
        scene.allowed_collision_matrix.default_entry_names.append(CUBE_ID)
        scene.allowed_collision_matrix.default_entry_values.append(True)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or not response.success:
            self.node.get_logger().warn("Could not attach pick_cube to the gripper")
            return False
        self.node.get_logger().info("Attached pick_cube to gripper_tcp")
        return True


def make_pose(frame, x, y, z, qx, qy, qz, qw, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def run_pick(arm: SteveArm, args) -> int:
    stamp = arm.node.get_clock().now().to_msg()
    grasp = make_pose(
        args.frame, args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw, stamp
    )
    pregrasp = make_pose(
        args.frame,
        args.x,
        args.y,
        args.z + args.approach,
        args.qx,
        args.qy,
        args.qz,
        args.qw,
        stamp,
    )

    steps = [
        ("named ready", lambda: arm.move_named("ready")),
        ("open gripper", lambda: arm.set_gripper(GRIPPER_OPEN)),
        ("pregrasp", lambda: arm.move_pose(pregrasp)),
        ("approach", lambda: arm.cartesian_to(grasp)),
        ("close gripper", lambda: arm.set_gripper(GRIPPER_CLOSED)),
        ("attach cube", lambda: arm.attach_cube() or True),
        ("lift", lambda: arm.cartesian_to(pregrasp)),
        ("named ready", lambda: arm.move_named("ready")),
    ]
    for label, fn in steps:
        arm.node.get_logger().info(f"=== {label} ===")
        if not fn():
            arm.node.get_logger().error(f"Pick aborted at: {label}")
            return 1
        time.sleep(1.0)
    arm.node.get_logger().info("Pick sequence finished")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--named", choices=sorted(NAMED_POSES), help="Go to a named joint pose")
    parser.add_argument("--pick", action="store_true", help="Run the scripted pick sequence")
    parser.add_argument(
        "--dump",
        action="store_true",
        help="Print the current UR5e joints (use after RViz Plan + Execute)",
    )
    parser.add_argument(
        "--save-ready",
        action="store_true",
        help="Overwrite the ready pose with the current UR5e joints",
    )
    parser.add_argument(
        "--x",
        type=float,
        default=0.20,
        help="Grasp x in --frame [m]. Lines up with the UR5e shoulder (base_link x≈0.20)",
    )
    parser.add_argument(
        "--y",
        type=float,
        default=-0.55,
        help="Grasp y in --frame [m]. World cube (x=-0.20, y=0.55) is base_link (0.20, -0.55) at robot yaw=pi",
    )
    parser.add_argument("--z", type=float, default=0.82, help="Grasp z in --frame [m]")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--approach", type=float, default=0.12, help="Pregrasp height offset [m]")
    parser.add_argument(
        "--qx", type=float, default=1.0, help="gripper_tcp orientation (default: z-down)"
    )
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=0.0)
    args = parser.parse_args(argv)

    if not args.named and not args.pick and not args.dump and not args.save_ready:
        parser.error("Pass --named ready|home, --pick, --dump, or --save-ready")

    rclpy.init()
    node = rclpy.create_node("steve_pick_object")
    try:
        if args.dump or args.save_ready:
            joints = read_arm_joints(node)
            print("Current UR5e joints (radians):")
            for name, value in joints.items():
                print(f"  {name}: {value}")
            if args.save_ready:
                save_ready_pose(joints)
                node.get_logger().info(
                    "Restart move_group so RViz Goal State 'ready' picks up the SRDF."
                )
            return 0

        arm = SteveArm(node)
        arm.wait()
        if args.named:
            ok = arm.move_named(args.named)
            return 0 if ok else 1
        return run_pick(arm, args)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
