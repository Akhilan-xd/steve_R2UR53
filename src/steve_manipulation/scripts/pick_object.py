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
import math
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
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
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
    # Parked on the arm's side. The wrist camera looks along the pan axis,
    # and the links stay outside the pan camera's view of the cube.
    "home": {
        "ur5eshoulder_pan_joint": -0.9423,
        "ur5eshoulder_lift_joint": -0.8950,
        "ur5eelbow_joint": 1.8669,
        "ur5ewrist_1_joint": -1.1554,
        "ur5ewrist_2_joint": -0.9506,
        "ur5ewrist_3_joint": 0.1067,
    },
    # Same physical pose as shoulder_lift=4.7123 / wrist_1=5.3232, folded onto
    # the turn the arm actually spawns on. The +2π copies force a full
    # rotation through the cabinet, which the planner cannot solve.
    "ready": {
        "ur5eshoulder_pan_joint": 1.57,
        "ur5eshoulder_lift_joint": -1.5709,
        "ur5eelbow_joint": 2.530,
        "ur5ewrist_1_joint": -0.960,
        "ur5ewrist_2_joint": 1.57,
        "ur5ewrist_3_joint": 0.0,
    },
    # Wrist camera stepped forward from the folded pose so it looks at the
    # cube while the base is still about a meter from the stand.
    "see": {
        "ur5eshoulder_pan_joint": 0.614,
        "ur5eshoulder_lift_joint": -0.994,
        "ur5eelbow_joint": 1.530,
        "ur5ewrist_1_joint": -0.515,
        "ur5ewrist_2_joint": 0.169,
        "ur5ewrist_3_joint": -0.357,
    },
    # Wrist optical axis points at the cube once the base is beside the stand.
    "look": {
        "ur5eshoulder_pan_joint": -0.4,
        "ur5eshoulder_lift_joint": -1.01,
        "ur5eelbow_joint": 2.2,
        "ur5ewrist_1_joint": -1.29,
        "ur5ewrist_2_joint": 1.2,
        "ur5ewrist_3_joint": 0.0,
    },
}

# URDF revolute limits. Elbow is ±π; the other arm joints are ±2π.
JOINT_BOUNDS = {
    "ur5eshoulder_pan_joint": (-6.2832, 6.2832),
    "ur5eshoulder_lift_joint": (-6.2832, 6.2832),
    "ur5eelbow_joint": (-3.1416, 3.1416),
    "ur5ewrist_1_joint": (-6.2832, 6.2832),
    "ur5ewrist_2_joint": (-6.2832, 6.2832),
    "ur5ewrist_3_joint": (-6.2832, 6.2832),
}

ARM_JOINTS = list(NAMED_POSES["home"].keys())
GRIPPER_OPEN = 0.0
# 0.8 rad is fully shut (0 mm). The pad meshes meet a 40 mm cube near 0.45 rad.
# 0.50 is only a small step past that. A stiffer close drives the fingers
# through the cube, because Gazebo places these joints by position.
GRIPPER_CLOSED = 0.50
# Knuckle speed while closing. The position controller jumps to whatever
# setpoint it is given, so the client has to walk the setpoint itself.
GRIPPER_CLOSE_SPEED = 0.25  # rad/s, about 2 s from open to the grasp
GRIPPER_OPEN_SPEED = 0.4
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


def wrap_near(value: float, reference: float, lower: float, upper: float) -> float:
    """Pick the 2π copy of value closest to reference that stays inside limits."""
    two_pi = 2.0 * math.pi
    k = round((reference - value) / two_pi)
    best = None
    best_dist = None
    for offset in range(k - 3, k + 4):
        candidate = value + offset * two_pi
        if candidate < lower - 1e-4 or candidate > upper + 1e-4:
            continue
        dist = abs(candidate - reference)
        if best_dist is None or dist < best_dist:
            best = candidate
            best_dist = dist
    if best is None:
        return min(max(value, lower), upper)
    return best


def wrap_pose(joints: dict, reference: dict) -> dict:
    wrapped = {}
    for name in ARM_JOINTS:
        lower, upper = JOINT_BOUNDS[name]
        wrapped[name] = round(wrap_near(float(joints[name]), float(reference[name]), lower, upper), 4)
    return wrapped


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
    joints = wrap_pose(joints, NAMED_POSES["home"])
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
        self.grasp = node.create_client(SetBool, "/grasp_cube")

    def wait(self, timeout=30.0):
        self.node.get_logger().info("Waiting for /move_action (move_group)...")
        if not self.move.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("move_group is not running. Launch steve_manipulation first.")
        self.node.get_logger().info("Waiting for gripper action...")
        if not self.gripper.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().warn(
                "Gripper action not available; arm motions will still work."
            )

    def _wait(self, future, tick=None, timeout=None):
        """Spin until `future` finishes. `tick` keeps the base moving.

        Returns None if `timeout` (wall seconds) runs out first.
        """
        end = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if end is not None and time.monotonic() > end:
                return None
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if tick is not None:
                tick()
        return future.result()

    def _send_move(self, request: MotionPlanRequest, plan_only=False, tick=None) -> bool:
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = plan_only
        # A timed intercept needs the one trajectory we just measured.
        # Replanning would replace it with a path of a different duration.
        goal.planning_options.replan = not plan_only
        goal.planning_options.replan_attempts = 5
        goal.planning_options.replan_delay = 0.5

        send_future = self.move.send_goal_async(goal)
        handle = self._wait(send_future, tick)
        if handle is None or not handle.accepted:
            self.node.get_logger().error("MoveGroup rejected the goal")
            return False

        result_future = handle.get_result_async()
        result = self._wait(result_future, tick)
        if result is None:
            self.node.get_logger().error("MoveGroup returned no result")
            return False
        result = result.result
        error = result.error_code.val
        self._last_trajectory = result.planned_trajectory if error == MoveItErrorCodes.SUCCESS else None
        if error != MoveItErrorCodes.SUCCESS:
            name = MOVEIT_ERRORS.get(error, "UNKNOWN")
            if error == MoveItErrorCodes.CONTROL_FAILED:
                hint = " Gazebo lagged the planned path."
            elif error == MoveItErrorCodes.FAILURE:
                hint = (
                    " The planner found no collision-free path. "
                    "A joint target a full turn from the current pose does this."
                )
            else:
                hint = ""
            self.node.get_logger().error(f"MoveGroup failed with {name} ({error}).{hint}")
            return False
        self.node.get_logger().info("Motion succeeded")
        return True

    def execute_trajectory(self, trajectory, tick=None) -> bool:
        """Run a trajectory that was already planned, without planning again."""
        if trajectory is None or not trajectory.joint_trajectory.points:
            self.node.get_logger().error("No trajectory to execute")
            return False
        if not self.execute.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("ExecuteTrajectory action is not available")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute.send_goal_async(goal)
        handle = self._wait(send_future, tick)
        if handle is None or not handle.accepted:
            self.node.get_logger().error("Trajectory was rejected")
            return False
        result = self._wait(handle.get_result_async(), tick)
        error = MoveItErrorCodes.FAILURE if result is None else result.result.error_code.val
        if error != MoveItErrorCodes.SUCCESS:
            name = MOVEIT_ERRORS.get(error, "UNKNOWN")
            self.node.get_logger().error(f"Trajectory execution failed with {name} ({error})")
            return False
        return True

    def cartesian_path(self, poses, tick=None, duration=None):
        """Straight path through poses. Times are scaled to `duration` when set."""
        if not poses:
            return None
        if not self.cartesian.wait_for_service(timeout_sec=5.0):
            return None
        request = GetCartesianPath.Request()
        request.header = poses[0].header
        request.start_state.is_diff = True
        request.group_name = GROUP
        request.link_name = EE_LINK
        for pose in poses:
            request.waypoints.append(pose.pose)
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.max_velocity_scaling_factor = 0.9
        request.max_acceleration_scaling_factor = 0.55
        request.avoid_collisions = True
        response = self._wait(self.cartesian.call_async(request), tick)
        if response is None or response.fraction < 0.95 or not response.solution.joint_trajectory.points:
            frac = 0.0 if response is None else response.fraction
            self.node.get_logger().warn(f"Cartesian path only covered {frac:.2f}")
            return None
        if duration is not None:
            _scale_trajectory(response.solution, duration)
        return response.solution

    def _base_request(self) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.9
        req.max_acceleration_scaling_factor = 0.55
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = "base_link"
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -0.2
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 1.8
        return req

    def move_named(self, name: str, tick=None) -> bool:
        if name not in NAMED_POSES:
            raise ValueError(f"Unknown pose '{name}'. Try: {list(NAMED_POSES)}")
        self.node.get_logger().info(f"Planning to named pose '{name}'")
        try:
            current = read_arm_joints(self.node, timeout=2.0)
        except RuntimeError:
            current = NAMED_POSES["home"]
        target = wrap_pose(NAMED_POSES[name], current)
        self.node.get_logger().info(
            "Joint target: " + ", ".join(f"{joint}={value:.3f}" for joint, value in target.items())
        )
        req = self._base_request()
        constraints = Constraints()
        constraints.name = name
        for joint, value in target.items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        return self._send_move(req, tick=tick)

    def move_pose(self, pose: PoseStamped, tick=None, plan_only=False, planning_time=10.0) -> bool:
        self.node.get_logger().info(
            f"Planning to pose in {pose.header.frame_id}: "
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f})"
        )
        req = self._base_request()
        req.allowed_planning_time = float(planning_time)
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
        return self._send_move(req, plan_only=plan_only, tick=tick)

    def cartesian_to(self, pose: PoseStamped, step=0.005, tick=None) -> bool:
        if not self.cartesian.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("compute_cartesian_path missing; using pose goal")
            return self.move_pose(pose, tick=tick)

        request = GetCartesianPath.Request()
        request.header = pose.header
        request.start_state.is_diff = True
        request.group_name = GROUP
        request.link_name = EE_LINK
        request.waypoints.append(pose.pose)
        request.max_step = step
        request.jump_threshold = 0.0
        request.max_velocity_scaling_factor = 0.9
        request.max_acceleration_scaling_factor = 0.55
        # Every waypoint, including the arm links, is checked against the
        # planning scene. pick_stand is a hard obstacle; only a full path is
        # executed. A short path means the straight line hits the stand.
        request.avoid_collisions = True

        future = self.cartesian.call_async(request)
        response = self._wait(future, tick)
        if response is None or response.fraction < 0.999:
            frac = 0.0 if response is None else response.fraction
            self.node.get_logger().warn(
                f"Cartesian path only covered {frac:.2f} without hitting the stand; "
                "falling back to a collision-checked joint-space plan"
            )
            return self.move_pose(pose, tick=tick)

        if not self.execute.wait_for_server(timeout_sec=5.0):
            return self.move_pose(pose, tick=tick)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        send_future = self.execute.send_goal_async(goal)
        handle = self._wait(send_future, tick)
        if handle is None or not handle.accepted:
            return self.move_pose(pose, tick=tick)
        result_future = handle.get_result_async()
        result = self._wait(result_future, tick)
        error = MoveItErrorCodes.FAILURE if result is None else result.result.error_code.val
        if error != MoveItErrorCodes.SUCCESS:
            name = MOVEIT_ERRORS.get(error, "UNKNOWN")
            self.node.get_logger().error(
                f"Cartesian execution failed with {name} ({error})"
            )
            return False
        return True

    def _knuckle_position(self, timeout=1.0) -> float:
        holder = {"pos": None}

        def callback(msg: JointState):
            if "robotiq_85_left_knuckle_joint" not in msg.name:
                return
            index = msg.name.index("robotiq_85_left_knuckle_joint")
            holder["pos"] = float(msg.position[index])

        subs = [
            self.node.create_subscription(JointState, "/joint_states", callback, 10),
            self.node.create_subscription(JointState, "/joint_states_complete", callback, 10),
        ]
        end = time.monotonic() + timeout
        while holder["pos"] is None and time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for sub in subs:
            self.node.destroy_subscription(sub)
        return 0.0 if holder["pos"] is None else holder["pos"]

    def _gripper_goal(self, position: float, effort: float, wait: bool, tick=None) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        send_future = self.gripper.send_goal_async(goal)
        handle = self._wait(send_future, tick)
        if handle is None or not handle.accepted:
            self.node.get_logger().error("Gripper goal rejected")
            return False
        if not wait:
            return True
        # Fingers stopped on the cube can leave the action without a result.
        # The knuckle position is what matters, so do not wait forever.
        if self._wait(handle.get_result_async(), tick, timeout=4.0) is None:
            reached = self._knuckle_position()
            self.node.get_logger().warn(
                f"Gripper result did not arrive. Knuckle is at {reached:.2f} rad, goal {position:.2f}."
            )
        return True

    def _pace(self, seconds: float, tick=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if tick is not None:
                tick()

    def set_gripper(self, position: float, effort: float = 8.0, speed: float = GRIPPER_CLOSE_SPEED, tick=None) -> bool:
        """Walk the knuckle setpoint to `position`.

        GripperActionController writes the goal position straight to the
        hardware, so one command from 0 to 0.79 slams the fingers shut.
        """
        if not self.gripper.server_is_ready():
            self.node.get_logger().warn("Skipping gripper command (server missing)")
            return False

        current = self._knuckle_position()
        distance = position - current
        if abs(distance) < 0.01 or speed <= 0.0:
            return self._gripper_goal(position, effort, wait=True, tick=tick)

        step = 0.02 if distance > 0.0 else -0.02
        waypoints = []
        cursor = current
        while (step > 0.0 and cursor + step < position) or (step < 0.0 and cursor + step > position):
            cursor += step
            waypoints.append(cursor)
        waypoints.append(position)
        dt = abs(step) / speed
        self.node.get_logger().info(
            f"Gripper {current:.2f} -> {position:.2f} rad at {speed:.2f} rad/s"
        )
        for index, waypoint in enumerate(waypoints):
            last = index == len(waypoints) - 1
            if not self._gripper_goal(waypoint, effort, wait=last, tick=tick):
                return False
            if not last:
                self._pace(dt, tick)
        return True

    def grasp_cube(self, hold: bool) -> bool:
        """Weld or release the Gazebo cube so it moves with the palm."""
        if not self.grasp.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("No /grasp_cube service")
            return False
        request = SetBool.Request()
        request.data = hold
        future = self.grasp.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else "no response"
            self.node.get_logger().error(f"Grasp weld failed ({detail})")
            return False
        self.node.get_logger().info(response.message)
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
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            self.node.get_logger().warn("Could not attach pick_cube to the gripper")
            return False
        self.node.get_logger().info("Attached pick_cube to gripper_tcp")
        return True

    def set_pick_scene(self, x: float, y: float, z: float, tick=None, allow_gripper=False) -> bool:
        """Move the stand and cube in the planning scene onto a perceived pose.

        publish_pick_scene writes the spawn pose once. After the base drives,
        that box is no longer where the stand is.
        """
        if not self.scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("No /apply_planning_scene; arm may hit the stand")
            return False

        def box(object_id, xyz, size):
            obj = CollisionObject()
            obj.id = object_id
            obj.header.frame_id = "base_link"
            obj.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(size)
            pose = Pose()
            pose.position.x = float(xyz[0])
            pose.position.y = float(xyz[1])
            pose.position.z = float(xyz[2])
            pose.orientation.w = 1.0
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            return obj

        scene = PlanningScene()
        scene.is_diff = True
        # The 0.25 m stand plus 1 cm on each side. No collision matrix is sent:
        # a diff with entry names replaces the SRDF matrix, which puts the arm
        # in self-collision. The grasp is made legal through the geometry.
        if allow_gripper:
            # Final approach: the cube is the target, so it is not an obstacle,
            # and the fingertips may pass 2 cm into the stand top.
            scene.world.collision_objects.append(
                box("pick_stand", (x, y, 0.39), (0.27, 0.27, 0.78))
            )
            remove = CollisionObject()
            remove.id = CUBE_ID
            remove.header.frame_id = "base_link"
            remove.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove)
        else:
            scene.world.collision_objects.append(
                box("pick_stand", (x, y, 0.4), (0.27, 0.27, 0.8))
            )
            scene.world.collision_objects.append(box(CUBE_ID, (x, y, z), (0.04, 0.04, 0.04)))
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene.call_async(request)
        response = self._wait(future, tick)
        if response is None or not response.success:
            self.node.get_logger().warn("Could not move the pick scene onto the perceived cube")
            return False
        self.node.get_logger().info(
            f"Planning scene cube at base_link ({x:.3f}, {y:.3f}, {z:.3f})"
        )
        return True

    def set_place_scene(self, x, y, yaw, size, top, tick=None, lowered=False) -> bool:
        """Put the kitchen table into the scene at base_link (x, y, yaw), and drop the stand.

        The stand box is in base_link, so it drove along with the robot.
        `lowered` sinks the table 3 cm, so the held cube and the fingertips
        may reach the real top during the final descent.
        """
        if not self.scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("No /apply_planning_scene; arm may hit the table")
            return False
        height = top - (0.03 if lowered else 0.0)
        table = CollisionObject()
        table.id = "kitchen_table"
        table.header.frame_id = "base_link"
        table.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(height)]
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(height / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        table.primitives.append(primitive)
        table.primitive_poses.append(pose)
        stand = CollisionObject()
        stand.id = "pick_stand"
        stand.header.frame_id = "base_link"
        stand.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend([stand, table])
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait(self.scene.call_async(request), tick, timeout=5.0)
        if response is None or not response.success:
            self.node.get_logger().warn("Could not put the kitchen table into the planning scene")
            return False
        return True

    def detach_cube(self, tick=None) -> bool:
        """Remove the held cube from the planning scene once it is on the table."""
        if not self.scene.wait_for_service(timeout_sec=2.0):
            return False
        attached = AttachedCollisionObject()
        attached.link_name = EE_LINK
        attached.object.id = CUBE_ID
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait(self.scene.call_async(request), tick, timeout=5.0)
        return response is not None and response.success


def make_pose(frame, x, y, z, qx, qy, qz, qw, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = (z+0.03)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _quat_from_matrix(R):
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def horizontal_grasp(x, y):
    """Side grasp. Gripper points at the cube and the fingers open sideways.

    Gripper +Z is the approach, in the base XY plane, from the robot toward
    the cube. Gripper +X is horizontal, which is the direction the Robotiq
    pads close, so neither finger swings down into the stand.
    Returns (qx, qy, qz, qw) and the unit approach (ax, ay).
    """
    dist = math.hypot(x, y)
    if dist < 1e-4:
        ax, ay = 1.0, 0.0
    else:
        ax, ay = x / dist, y / dist
    # Columns are gripper X, Y, Z expressed in base_link.
    rotation = [
        [-ay, 0.0, ax],
        [ax, 0.0, ay],
        [0.0, 1.0, 0.0],
    ]
    return _quat_from_matrix(rotation), (ax, ay)


def pose_at(frame, x, y, z, quat, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
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
        ("open gripper", lambda: arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED)),
        ("release cube", lambda: arm.grasp_cube(False)),
        ("pregrasp", lambda: arm.move_pose(pregrasp)),
        ("approach", lambda: arm.cartesian_to(grasp)),
        ("hold cube", lambda: arm.grasp_cube(True)),
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


# UR5e shoulder in base_link, and how far gripper_tcp can sit for a side grasp.
# The datasheet reach is 0.85 m to the flange. 0.70 m leaves room for the
# horizontal wrist pose, so MoveIt is not asked for a point it cannot solve.
SHOULDER_XYZ = (0.20, 0.0, 0.93)
TCP_REACH = 0.70
# 0.20 m keeps the pregrasp outside the 0.15 m stand keep-out.
PREGRASP_BACK = 0.20


def shoulder_distance(x: float, y: float, z: float) -> float:
    sx, sy, sz = SHOULDER_XYZ
    return math.sqrt((x - sx) ** 2 + (y - sy) ** 2 + (z - sz) ** 2)


def grasp_in_reach(x: float, y: float, z: float) -> bool:
    """True when gripper_tcp can sit on the cube itself."""
    return shoulder_distance(x, y, z) <= TCP_REACH


def horizontal_targets(x, y, z, frame, stamp):
    """Side-grasp poses MoveIt can actually solve.

    The gripper points from the shoulder toward the cube. The pregrasp is
    20 cm short of the cube. While that cube is outside the UR5e, the
    pregrasp is pulled back onto the reach sphere so the arm can start
    toward the cube instead of being given an unreachable goal.
    Returns pregrasp, grasp, approach xy, and whether the cube itself is in reach.
    """
    sx, sy, sz = SHOULDER_XYZ
    dx, dy, dz = x - sx, y - sy, z - sz
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-4:
        ax, ay, az = 1.0, 0.0, 0.0
    else:
        ax, ay, az = dx / dist, dy / dist, dz / dist
    horizontal = math.hypot(dx, dy)
    if horizontal < 1e-4:
        hax, hay = 1.0, 0.0
    else:
        hax, hay = dx / horizontal, dy / horizontal
    quat, _approach = horizontal_grasp(hax, hay)
    in_reach = dist <= TCP_REACH
    if in_reach:
        px = x - hax * PREGRASP_BACK
        py = y - hay * PREGRASP_BACK
        pz = z
    else:
        px = sx + ax * TCP_REACH
        py = sy + ay * TCP_REACH
        pz = sz + az * TCP_REACH
    pregrasp = pose_at(frame, px, py, pz, quat, stamp)
    grasp = pose_at(frame, x, y, z, quat, stamp)
    return pregrasp, grasp, (hax, hay), in_reach


def _seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def trajectory_seconds(trajectory) -> float:
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    return _seconds(points[-1].time_from_start)


def _scale_trajectory(trajectory, duration: float):
    """Stretch joint times so the path takes `duration` seconds."""
    points = trajectory.joint_trajectory.points
    current = _seconds(points[-1].time_from_start)
    if current < 1e-3 or duration < 1e-3:
        return
    scale = duration / current
    for point in points:
        scaled = _seconds(point.time_from_start) * scale
        point.time_from_start.sec = int(scaled)
        point.time_from_start.nanosec = int((scaled - int(scaled)) * 1e9)
        if point.velocities:
            point.velocities = [value / scale for value in point.velocities]
        if point.accelerations:
            point.accelerations = [value / (scale * scale) for value in point.accelerations]


def lead_xyz(x, y, z, vx, vy, horizon):
    """Where a world-fixed cube sits in base_link after `horizon` seconds.

    A positive base command decreases the cube's base_link coordinates.
    """
    return x - vx * horizon, y - vy * horizon, z


def matched_base_speed(which: str, duration: float, cube_x: float, cube_y: float, gap: float):
    """Base velocity with the same speed as the hand closing on the cube.

    The hand covers the pregrasp distance in `duration` seconds. The base
    uses that same speed along the cube, unless that would drive through
    the stand, in which case it keeps the speed but slides along the stand.
    Returns vx, vy, and the arm speed.
    """
    dist = math.hypot(cube_x, cube_y)
    if dist < 1e-3:
        return 0.04, 0.0, 0.04
    ux, uy = cube_x / dist, cube_y / dist
    if which == "grasp":
        travel = PREGRASP_BACK
    else:
        travel = max(0.20, min(0.45, dist - 0.35))
    v_arm = travel / max(duration, 0.25)
    v_arm = min(max(v_arm, 0.04), 0.20)
    room = (gap - 0.10) / max(duration, 0.25)
    if room >= v_arm:
        return ux * v_arm, uy * v_arm, v_arm
    # Same speed as the arm, along the stand, so the body does not close the gap.
    return -uy * v_arm, ux * v_arm, v_arm


def intercept_move(arm: SteveArm, here, vx, vy, frame, tick, which: str, set_speed=None, clearance=None) -> bool:
    """Plan the grasp for the cube's position at the moment the arm arrives.

    The first plan measures how long the arm needs. The second plan aims at
    the cube shifted by the base speed over that same planning-plus-motion
    time, and that plan is the one that runs.
    """
    horizon = 2.5
    trajectory = None
    for attempt in range(2):
        x, y, z = here()
        fx, fy, fz = lead_xyz(x, y, z, vx, vy, horizon)
        stamp = arm.node.get_clock().now().to_msg()
        pregrasp, grasp, _axes, _in_reach = horizontal_targets(fx, fy, fz, frame, stamp)
        target = pregrasp if which == "pregrasp" else grasp
        arm.set_pick_scene(fx, fy, fz, tick=tick)
        started = arm.node.get_clock().now()
        ok = arm.move_pose(target, tick=tick, plan_only=True, planning_time=2.0)
        elapsed = (arm.node.get_clock().now() - started).nanoseconds * 1e-9
        trajectory = getattr(arm, "_last_trajectory", None)
        if not ok or trajectory is None or not trajectory.joint_trajectory.points:
            return False
        duration = trajectory_seconds(trajectory)
        # Match once, from the measured arm motion. The second plan and the
        # execution both use that speed, so the lead stays honest.
        if attempt == 0 and set_speed is not None and clearance is not None:
            vx, vy, v_arm = matched_base_speed(which, duration, x, y, clearance())
            set_speed(vx, vy)
            arm.node.get_logger().info(
                f"Arm {v_arm:.3f} m/s over {duration:.2f} s, "
                f"base matched to ({vx:.3f}, {vy:.3f}) m/s"
            )
        horizon = elapsed + duration
        point = target.pose.position
        arm.node.get_logger().info(
            f"Intercept {which}: base ({vx:.3f}, {vy:.3f}) m/s, "
            f"arm {duration:.2f} s plus {elapsed:.2f} s planning, "
            f"goal ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"
        )
    return arm.execute_trajectory(trajectory, tick)


def track_and_close(arm: SteveArm, x, y, z, vx, vy, frame, tick) -> bool:
    """Keep the gripper on the cube while the fingers close and the base moves.

    The cube drifts in base_link at minus the base velocity. The hand follows
    that drift for the whole close, so the pads meet the cube instead of the
    point where it was when the close started.
    """
    duration = abs(GRIPPER_CLOSED - GRIPPER_OPEN) / GRIPPER_CLOSE_SPEED
    stamp = arm.node.get_clock().now().to_msg()
    quat, _axes = horizontal_grasp(x, y)
    poses = []
    steps = 8
    for index in range(1, steps + 1):
        sample = duration * index / steps
        px, py, pz = lead_xyz(x, y, z, vx, vy, sample)
        poses.append(pose_at(frame, px, py, pz, quat, stamp))
    trajectory = arm.cartesian_path(poses, tick=tick, duration=duration)
    if trajectory is None:
        arm.node.get_logger().warn("Tracking path unavailable; closing on the current cube")
        return arm.set_gripper(GRIPPER_CLOSED, tick=tick)

    started = arm.node.get_clock().now()
    next_step = -1.0

    def follow():
        nonlocal next_step
        if tick is not None:
            tick()
        elapsed = (arm.node.get_clock().now() - started).nanoseconds * 1e-9
        if elapsed < next_step:
            return
        next_step = elapsed + 0.15
        fraction = min(1.0, max(0.0, elapsed / duration))
        position = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * fraction
        arm._gripper_goal(position, 8.0, wait=False, tick=None)

    arm.node.get_logger().info(
        f"Tracking the cube for {duration:.1f} s at base ({vx:.3f}, {vy:.3f}) m/s while the fingers close"
    )
    return arm.execute_trajectory(trajectory, follow)


def solve_horizontal(
    arm: SteveArm,
    x,
    y,
    z,
    frame="base_link",
    tick=None,
    plan_only=True,
    planning_time=None,
) -> bool:
    """Ask MoveIt for a reachable horizontal grasp of the cube it has now.

    Returns (planned, cube_in_reach). The goal is the real pregrasp when the
    cube is inside the arm, and a point on the reach sphere aimed at the cube
    otherwise. plan_only leaves the arm where it is. The base can keep moving
    through `tick` while the solver runs.
    """
    stamp = arm.node.get_clock().now().to_msg()
    pregrasp, _grasp, _approach, in_reach = horizontal_targets(x, y, z, frame, stamp)
    if tick is not None:
        tick()
    if planning_time is None:
        planning_time = 2.0 if plan_only else 8.0
    arm.set_pick_scene(x, y, z, tick=tick)
    point = pregrasp.pose.position
    arm.node.get_logger().info(
        f"MoveIt goal ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}), "
        f"{shoulder_distance(point.x, point.y, point.z):.2f} m from the shoulder"
        + ("" if in_reach else " (short of the cube, still in reach)")
    )
    ok = arm.move_pose(
        pregrasp,
        tick=tick,
        plan_only=plan_only,
        planning_time=planning_time,
    )
    return ok, in_reach


def hand_on_cube(arm, x, y, z, limit=0.04) -> bool:
    """True when gripper_tcp is on the cube. A miss must not be welded."""
    buffer = Buffer()
    TransformListener(buffer, arm.node)
    deadline = time.time() + 2.0
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(arm.node, timeout_sec=0.05)
        try:
            transform = buffer.lookup_transform("base_link", EE_LINK, Time())
        except Exception:
            continue
        point = transform.transform.translation
        distance = math.sqrt((point.x - x) ** 2 + (point.y - y) ** 2 + (point.z - z) ** 2)
        arm.node.get_logger().info(f"Gripper to cube {distance * 100:.1f} cm")
        return distance <= limit
    arm.node.get_logger().error("Could not compare the gripper with the cube")
    return False


def grasp_at_slot(
    arm: SteveArm, x, y, z, tick=None, on_slot=None, on_grasped=None, pose_fn=None, on_slide=None
) -> int:
    """Pick the cube at one fixed base_link pose.

    The base is responsible for keeping the cube on this pose. The arm
    plans to that same pose, then pinches only once the live cube is
    actually there, so the hand and the cube meet.
    """
    stamp = arm.node.get_clock().now().to_msg()
    pregrasp, grasp, (ax, ay), in_reach = horizontal_targets(x, y, z, "base_link", stamp)
    if not in_reach:
        point = grasp.pose.position
        arm.node.get_logger().error(
            f"Grasp pose ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}) is outside the arm"
        )
        return 1
    arm.node.get_logger().info(
        f"Planning the camera pose exactly: base_link ({x:.3f}, {y:.3f}, {z:.3f}), "
        f"approach ({ax:.2f}, {ay:.2f})"
    )

    def aligned(limit: float, timeout: float) -> bool:
        if on_slot is None:
            return True
        deadline = arm.node.get_clock().now() + Duration(seconds=timeout)
        while arm.node.get_clock().now() < deadline and rclpy.ok():
            if on_slot(limit):
                return True
            arm._pace(0.1, tick)
        return False

    steps = [
        ("scene", lambda: arm.set_pick_scene(x, y, z, tick=tick) or True),
        ("open gripper", lambda: arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED, tick=tick)),
        ("release cube", lambda: arm.grasp_cube(False)),
        ("pregrasp", lambda: arm.move_pose(pregrasp, tick=tick, planning_time=3.0)),
        ("align cube", lambda: aligned(0.03, 3.0)),
        (
            "approach",
            lambda: arm.set_pick_scene(x, y, z, tick=tick, allow_gripper=True)
            and arm.move_pose(grasp, tick=tick, planning_time=5.0)
            and arm.set_gripper(GRIPPER_CLOSED, tick=tick),
        ),
        (
            "check held",
            lambda: hand_on_cube(
                arm, *((lambda: (x, y, z)) if pose_fn is None else pose_fn)()
            ),
        ),
        ("hold cube", lambda: arm.grasp_cube(True)),
        ("attach cube", lambda: arm.attach_cube() or True),
        ("cube grasped", lambda: True if on_grasped is None else on_grasped() or True),
        ("home", lambda: arm.move_named("home", tick=tick)),
    ]
    for label, fn in steps:
        arm.node.get_logger().info(f"=== {label} ===")
        if tick is not None:
            tick()
        if not fn():
            arm.node.get_logger().error(f"Pick aborted at: {label}")
            return 1
        arm._pace(0.2, tick)
    arm.node.get_logger().info("Perceived pick finished at home")
    return 0


def run_perceived_pick(
    arm: SteveArm,
    x: float,
    y: float,
    z: float,
    frame="base_link",
    tick=None,
    hold=None,
    latest=None,
    on_grasped=None,
    base_speed=None,
    set_speed=None,
    clearance=None,
) -> int:
    """Execute a horizontal grasp on a known cube pose, then fold home.

    The gripper comes in from the robot side at the cube's own height.
    When `base_speed` reports a steady base velocity, the grasp is aimed
    where the cube will be when the arm arrives, and the fingers follow
    the cube while they close.
    """
    still = hold if hold is not None else tick

    def here():
        if latest is None:
            return x, y, z
        point = latest()
        if point is None:
            return x, y, z
        return point

    cx, cy, cz = here()
    stamp = arm.node.get_clock().now().to_msg()
    pregrasp, _grasp, (ax, ay), _in_reach = horizontal_targets(cx, cy, cz, frame, stamp)
    vx, vy = (0.0, 0.0) if base_speed is None else base_speed()
    arm.node.get_logger().info(
        f"Horizontal grasp of cube at {frame} ({cx:.3f}, {cy:.3f}, {cz:.3f}), "
        f"approach ({ax:.2f}, {ay:.2f}), base ({vx:.3f}, {vy:.3f}) m/s"
    )

    def approach():
        if base_speed is not None:
            speed = base_speed()
            return intercept_move(
                arm, here, speed[0], speed[1], frame, still, "grasp", set_speed, clearance
            )
        px, py, pz = here()
        fresh_stamp = arm.node.get_clock().now().to_msg()
        _pre, grasp, _axes, _in_reach = horizontal_targets(px, py, pz, frame, fresh_stamp)
        arm.set_pick_scene(px, py, pz, tick=still)
        return arm.cartesian_to(grasp, tick=still)

    def pregrasp_move():
        if base_speed is not None:
            speed = base_speed()
            return intercept_move(
                arm, here, speed[0], speed[1], frame, tick, "pregrasp", set_speed, clearance
            )
        return arm.move_pose(pregrasp, tick=tick)

    def pinch():
        if base_speed is None:
            return arm.set_gripper(GRIPPER_CLOSED, tick=still)
        px, py, pz = here()
        speed = base_speed()
        return track_and_close(arm, px, py, pz, speed[0], speed[1], frame, still)

    steps = [
        ("scene", tick, lambda: arm.set_pick_scene(cx, cy, cz, tick=tick) or True),
        ("open gripper", tick, lambda: arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED, tick=tick)),
        ("release cube", tick, lambda: arm.grasp_cube(False)),
        ("pregrasp", tick, pregrasp_move),
        ("approach", still, approach),
        ("hold cube", still, lambda: arm.grasp_cube(True)),
        ("close gripper", still, pinch),
        ("cube grasped", still, lambda: True if on_grasped is None else on_grasped() or True),
        ("attach cube", still, lambda: arm.attach_cube() or True),
        ("lift", tick, lambda: arm.move_named("home", tick=tick) if base_speed is not None else arm.cartesian_to(pregrasp, tick=tick)),
        ("home", tick, lambda: True if base_speed is not None else arm.move_named("home", tick=tick)),
    ]
    for label, beat, fn in steps:
        arm.node.get_logger().info(f"=== {label} ===")
        if beat is not None:
            beat()
        if not fn():
            arm.node.get_logger().error(f"Pick aborted at: {label}")
            return 1
        arm._pace(0.3, beat)
    arm.node.get_logger().info("Perceived pick finished at home")
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
