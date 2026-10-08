#!/usr/bin/env python3
"""Whole-body SQP pick of the red cube.

One optimisation moves the omnidirectional base and the UR5e together, so
the arm unfolds to the pregrasp while the base drives to the stand. When
the base is close, the hand descends and the gripper closes on the cube.

The solve runs in its own process and takes longer than a control step.
An EKF on /odom and the sent twist predicts the robot state at the time
each new plan will take over, and the node streams the active plan to
/cmd_vel and the arm at EXEC_DT against the clock (see wbc_estimator).

Do not run Nav2 at the same time: this node owns /cmd_vel.

  ros2 launch steve_manipulation manipulation.launch.py run_wbc:=true
"""

import math
import os
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fetch_cube import BETWEEN_M, CubeWatch, PalmSensor, PanTracker, _yaw  # noqa: E402
from wbc_estimator import BaseEKF, PlanStreamer  # noqa: E402
from wbc_kinematics import Chain  # noqa: E402
from wbc_solver import Limits, SolverProcess, WholeBodyMPC  # noqa: E402
from wbc_task import ARM_JOINTS, HOME, KEEPOUT_LINKS, Phase, PickPlan  # noqa: E402

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.50
GRIPPER_OPEN_SPEED = 0.4
GRIPPER_CLOSE_SPEED = 0.25
# Shortest time between two solve requests.
CONTROL_DT = 0.1
# The active plan goes out to /cmd_vel and the arm at these periods.
EXEC_DT = 0.02
ARM_DT = 0.05
ARM_LOOKAHEAD = 0.20
# Bounds on the expected solve time the plans are shifted by.
LATENCY_MIN = CONTROL_DT
LATENCY_MAX = 2.0
ODOM_STALE = 0.5


def _duration(seconds):
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    return MsgDuration(sec=sec, nanosec=int((seconds - sec) * 1e9))


def _pose(frame, xyz, rotation=None):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz)
    if rotation is None:
        msg.pose.orientation.w = 1.0
        return msg
    q = _quat(rotation)
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
    return msg


def _quat(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        )
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (
            0.25 * s,
            (rotation[0, 1] + rotation[1, 0]) / s,
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[2, 1] - rotation[1, 2]) / s,
        )
    if rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return (
            (rotation[0, 1] + rotation[1, 0]) / s,
            0.25 * s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
        )
    s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return (
        (rotation[0, 2] + rotation[2, 0]) / s,
        (rotation[1, 2] + rotation[2, 1]) / s,
        0.25 * s,
        (rotation[1, 0] - rotation[0, 1]) / s,
    )


def _to_odom(base, xyz):
    x, y, z = xyz
    c, s = math.cos(base[2]), math.sin(base[2])
    return np.array([base[0] + c * x - s * y, base[1] + s * x + c * y, z])


def _rotate_odom(yaw, vector):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1], vector[2]])


def _stamp(msg_stamp):
    return Time.from_msg(msg_stamp).nanoseconds * 1e-9


def _quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GripperDriver:
    """Walk the Robotiq knuckle setpoint. The action server jumps to whatever it is given."""

    def __init__(self, node: Node):
        self.node = node
        self.client = ActionClient(node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd")
        self.knuckle = 0.0
        self.target = None
        self.speed = GRIPPER_CLOSE_SPEED
        self.sent = None
        self._next = 0.0
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(JointState, "/joint_states_complete", self._on_joints, 10)

    def _on_joints(self, msg: JointState):
        if "robotiq_85_left_knuckle_joint" not in msg.name:
            return
        self.knuckle = float(msg.position[msg.name.index("robotiq_85_left_knuckle_joint")])

    def ready(self, timeout=15.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def set_target(self, position, speed=GRIPPER_CLOSE_SPEED):
        self.target = float(position)
        self.speed = float(speed)
        self._next = 0.0

    def done(self, tol=0.04):
        return self.target is not None and abs(self.knuckle - self.target) < tol

    def tick(self, now):
        if self.target is None or now < self._next:
            return
        current = self.sent if self.sent is not None else self.knuckle
        delta = self.target - current
        step = 0.02 if self.speed > 0.0 else abs(delta)
        if abs(delta) <= step + 1e-6:
            command = self.target
        else:
            command = current + math.copysign(step, delta)
        if self.sent is not None and abs(command - self.sent) < 1e-4:
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(command)
        goal.command.max_effort = 8.0
        self.client.send_goal_async(goal)
        self.sent = command
        self._next = now + (step / self.speed if self.speed > 0.0 else 0.0)


class WbcPick:
    def __init__(self, node: Node):
        self.node = node
        self.log = node.get_logger()
        self.cmd = node.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = node.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.phase_pub = node.create_publisher(String, "/wbc/phase", 10)
        self.hand_pub = node.create_publisher(PoseStamped, "/wbc/hand_target", 10)
        self.base_pub = node.create_publisher(PoseStamped, "/wbc/base_target", 10)
        self.twist = Twist()
        self.watch = CubeWatch(node)
        self.pan = PanTracker(node, self.watch.tf_buffer)
        self.palm = PalmSensor(node, self.watch.tf_buffer)
        self.gripper = GripperDriver(node)
        self.grasp = node.create_client(SetBool, "/grasp_cube")
        self.joints = None
        self.u = np.zeros(9)
        self.ekf = BaseEKF()
        self.last_odom = None
        self.last_cube_update = 0.0
        self.grasp_started = None
        self.welded = False
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(JointState, "/joint_states_complete", self._on_joints, 10)
        node.create_subscription(Odometry, "/odom", self._on_odom, 50)

    def _on_joints(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in ARM_JOINTS):
            self.joints = np.array([by_name[name] for name in ARM_JOINTS], dtype=float)

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        stamp = _stamp(msg.header.stamp)
        twist = msg.twist.twist
        self.ekf.update(
            stamp,
            (pose.position.x, pose.position.y, _quat_yaw(pose.orientation)),
            (twist.linear.x, twist.linear.y, twist.angular.z),
        )
        self.last_odom = stamp

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        end = self.node.get_clock().now() + Duration(seconds=seconds)
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self._send_base(np.zeros(3))

    def _send_base(self, twist):
        self.twist.linear.x = float(twist[0])
        self.twist.linear.y = float(twist[1])
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = float(twist[2])
        self.cmd.publish(self.twist)
        self.ekf.command(self.now(), twist)

    def _send_arm(self, q, dq):
        if self.joints is None:
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        goal = np.clip(q + dq * ARM_LOOKAHEAD, -2.0 * math.pi, 2.0 * math.pi)
        point.positions = [float(value) for value in goal]
        point.time_from_start = _duration(ARM_LOOKAHEAD)
        trajectory.points = [point]
        self.arm_pub.publish(trajectory)

    def _stop(self):
        self._send_base(np.zeros(3))
        if self.joints is not None:
            self._send_arm(self.joints, np.zeros(6))

    def base_pose(self):
        """Filtered base pose in odom now. TF feeds the EKF when /odom is quiet."""
        now = self.now()
        if self.last_odom is None or now - self.last_odom > ODOM_STALE:
            try:
                transform = self.watch.tf_buffer.lookup_transform("odom", "base_link", Time())
            except Exception:
                return None
            t = transform.transform.translation
            self.ekf.update(_stamp(transform.header.stamp), (t.x, t.y, _yaw(transform)))
        pose, _twist = self.ekf.state_at(now)
        return pose

    def cube_odom(self, base):
        xyz = self.watch.best()
        if xyz is None:
            return None
        return _to_odom(base, xyz)

    def cube_offset_odom(self, base, q, plan):
        offset = self.palm.offset()
        if offset is not None:
            return _rotate_odom(base[2], offset)
        cube = self.watch.fresh(max_age=0.5)
        if cube is None:
            return None
        hand_p, _ = plan.hand(base, q)
        return self.cube_odom(base) - hand_p

    def weld(self, hold):
        if not self.grasp.wait_for_service(timeout_sec=2.0):
            self.log.error("No /grasp_cube service")
            return False
        request = SetBool.Request()
        request.data = bool(hold)
        future = self.grasp.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else "no response"
            self.log.error(f"Grasp weld failed ({detail})")
            return False
        self.welded = bool(hold)
        self.log.info(response.message)
        return True

    def wait_gripper(self, position, speed, timeout=8.0):
        self.gripper.set_target(position, speed)
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok() and not self.gripper.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.gripper.tick(self.now())
            self._send_base(np.zeros(3))
        return self.gripper.done()

    def wait_state(self, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.joints is not None and self.base_pose() is not None:
                return True
        return False

    def _publish_task(self, plan, task, now_msg):
        phase = String()
        phase.data = plan.phase.value
        self.phase_pub.publish(phase)
        if task.hand_position is not None:
            msg = _pose("odom", task.hand_position, task.hand_rotation)
            msg.header.stamp = now_msg
            self.hand_pub.publish(msg)
        if task.base_xy is not None:
            yaw = task.base_yaw if task.base_yaw is not None else 0.0
            rotation = np.array(
                [
                    [math.cos(yaw), -math.sin(yaw), 0.0],
                    [math.sin(yaw), math.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            msg = _pose("odom", (task.base_xy[0], task.base_xy[1], 0.0), rotation)
            msg.header.stamp = now_msg
            self.base_pub.publish(msg)

    def _refresh_cube(self, plan, base, now):
        if plan.phase != Phase.APPROACH or now - self.last_cube_update < 0.4:
            return
        xyz = self.cube_odom(base)
        if xyz is None:
            return
        shift = float(np.linalg.norm(xyz[:2] - plan.cube[:2]))
        if shift < 0.015 or shift > 0.20:
            return
        try:
            plan.set_cube(xyz)
            self.last_cube_update = now
            self.log.info(
                f"Cube update from {self.watch.source}: odom "
                f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"
            )
        except RuntimeError as exc:
            self.log.warn(str(exc))

    def run(self) -> int:
        xml = robot_description(self.node)
        chain = Chain(xml, "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS)
        mpc = WholeBodyMPC(chain, limits=Limits())
        solver = SolverProcess(xml, "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS, limits=mpc.limits)
        try:
            return self._run(chain, mpc, solver)
        finally:
            solver.close()

    def _run(self, chain, mpc, solver) -> int:
        self.log.info("Whole-body SQP ready. Waiting for the robot state.")
        if not self.wait_state():
            self.log.error("No odom pose or arm joints")
            return 1
        if not self.gripper.ready():
            self.log.warn("Gripper action is missing. The fingers will not close.")
        self.weld(False)
        self.wait_gripper(GRIPPER_OPEN, GRIPPER_OPEN_SPEED)

        self.log.info("Waiting for the pan or wrist camera to measure the cube")
        cube = None
        end = self.node.get_clock().now() + Duration(seconds=60.0)
        while self.node.get_clock().now() < end and rclpy.ok() and cube is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self._send_base(np.zeros(3))
            self.pan.aim(self.watch.best())
            base = self.base_pose()
            if base is None:
                continue
            cube = self.cube_odom(base)
        if cube is None or self.base_pose() is None:
            self.log.error("Neither camera returned a depth pose of the red cube")
            return 1
        base = self.base_pose()
        q = self.joints.copy()
        self.log.info(
            f"{self.watch.source} sees the cube at odom "
            f"({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f})"
        )
        try:
            plan = PickPlan(chain, base, cube, now=self.now(), log=self.log.info)
        except RuntimeError as exc:
            self.log.error(str(exc))
            return 1
        self.last_cube_update = self.now()
        self.log.info(
            "Approach: the base drives to the stand while the arm unfolds to the pregrasp"
        )

        stream = PlanStreamer(mpc.dt, mpc.limit_input, len(self.u), LATENCY_MIN, LATENCY_MAX, steps=mpc.N)
        task = None
        last_request = -math.inf
        last_exec = None
        last_arm = -math.inf
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.005)
            now = self.now()
            base = self.base_pose()
            if base is None or self.joints is None:
                self._stop()
                continue
            q = self.joints.copy()
            self.pan.aim(self.watch.best())
            self.gripper.tick(now)

            result = solver.poll()
            if result is not None:
                U, _X, solve_seconds, _qp_iterations = result
                took, late = stream.receive(now, U, task.base_speed_scale)
                self.log.info(
                    f"{plan.phase.value}: base ({base[0]:.2f}, {base[1]:.2f}, {base[2]:.2f}) "
                    f"cmd ({self.u[0]:.2f}, {self.u[1]:.2f}, {self.u[2]:.2f}) "
                    f"|dq|={np.max(np.abs(self.u[3:])):.2f}  solve={1000 * solve_seconds:.0f} ms "
                    f"latency={1000 * took:.0f} ms" + (f" ({1000 * late:.0f} ms late)" if late > 0.0 else ""),
                    throttle_duration_sec=1.0,
                )

            if last_exec is None or now - last_exec >= EXEC_DT:
                dt = EXEC_DT if last_exec is None else min(max(now - last_exec, 1e-3), 0.1)
                last_exec = now
                self.u = stream.step(now, dt)
                self._send_base(self.u[:3])
                if now - last_arm >= ARM_DT:
                    last_arm = now
                    self._send_arm(q, self.u[3:])

            if not stream.waiting and now - last_request >= CONTROL_DT:
                last_request = now
                self._refresh_cube(plan, base, now)
                phase = plan.update(base, q, now)
                if phase == Phase.AT_GRASP:
                    offset = self.cube_offset_odom(base, q, plan)
                    if offset is None:
                        self.log.warn("No hand-to-cube reading. Closing on the planned grasp.")
                        if self.weld(True):
                            plan.start_grasp(now)
                            self.gripper.set_target(GRIPPER_CLOSED, GRIPPER_CLOSE_SPEED)
                            self.grasp_started = now
                    elif float(np.linalg.norm(offset)) <= BETWEEN_M:
                        self.log.info(
                            f"Cube is {np.linalg.norm(offset) * 100:.1f} cm from the pads"
                        )
                        if self.weld(True):
                            plan.start_grasp(now)
                            self.gripper.set_target(GRIPPER_CLOSED, GRIPPER_CLOSE_SPEED)
                            self.grasp_started = now
                        else:
                            plan.missed(offset, now, base, q)
                    else:
                        plan.missed(offset, now, base, q)
                if plan.phase == Phase.GRASP:
                    closed = self.gripper.done()
                    waited = self.grasp_started is not None and now - self.grasp_started > 2.5
                    if closed or waited:
                        plan.start_lift(now, base, q)
                if plan.phase in (Phase.DONE, Phase.FAILED):
                    break

                request = stream.request(now, self.ekf, q)
                task = plan.task(request.base, request.applies)
                solver.submit(request, task)
                self._publish_task(plan, task, self.node.get_clock().now().to_msg())

        self._stop()
        self.pan.park()
        if plan.phase == Phase.DONE:
            self.log.info("=" * 60)
            self.log.info("TASK COMPLETED SUCCESSFULLY")
            self.log.info("Whole-body pick: the base approached the stand while the")
            self.log.info("arm moved to the pregrasp, then the gripper took the red cube.")
            self.log.info("=" * 60)
            return 0
        self.log.error("=" * 60)
        self.log.error(f"TASK FAILED: {plan.reason}")
        self.log.error("=" * 60)
        if self.welded:
            self.weld(False)
        return 1


def robot_description(node: Node, timeout=30.0) -> str:
    xml = _description_from_topic(node, timeout=min(10.0, timeout))
    if xml:
        return xml
    xml = _description_from_parameter(node, timeout=timeout)
    if xml:
        return xml
    raise RuntimeError("No robot_description from robot_state_publisher")


def _description_from_topic(node, timeout):
    holder = {"xml": None}

    def callback(msg):
        holder["xml"] = msg.data

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(String, "/robot_description", callback, qos)
    end = time.monotonic() + timeout
    while holder["xml"] is None and time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return holder["xml"]


def _description_from_parameter(node, timeout):
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    request = GetParameters.Request()
    request.names = ["robot_description"]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    result = future.result()
    if result is None or not result.values:
        return None
    return result.values[0].string_value


def main(argv=None):
    rclpy.init(args=argv)
    node = rclpy.create_node(
        "wbc_pick",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    pick = None
    code = 1
    try:
        pick = WbcPick(node)
        code = pick.run()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(str(exc) or type(exc).__name__)
        code = 1
    finally:
        if pick is not None:
            pick._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
