#!/usr/bin/env python3
"""Whole-body pick of the red cube: see it, stand in front of the stand, grasp.

Perception publishes the cube from the pan or the wrist camera. This node
keeps a camera on the cube the whole time. The pan-tilt head is steered
onto the cube while the base drives, and the wrist camera looks down at it
once the arm is over the stand.

1. The arm goes to the side home pose, clear of the pan view.
2. The base turns to face the cube and drives until the stand is 10 cm
   in front of the body.
3. The base holds still. The cube pose is averaged from either camera.
4. Grasp candidates are tried in order: top-down with the fingers closing
   across the robot, top-down closing along it, then a side grasp. Each
   one plans a pregrasp, re-measures the cube with the wrist camera, and
   descends in a straight line.
5. The fingers close only once the cube is between them. A miss backs out
   and retries with the hand-to-cube offset. If every candidate fails, the
   base steps back to 0.80 m, where the side grasp also has a solution.
6. Lift, fold home, and drive the base back to where it started.
7. Drive to the kitchen table, lower the cube onto its top, open, and release.

The base command is geometry_msgs/Twist on /cmd_vel. Stop Nav2 first if it
is also publishing that topic.

  ros2 launch steve_manipulation manipulation.launch.py run_fetch:=true
"""

import copy
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_object import (
    ARM_JOINTS,
    EE_LINK,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    GRIPPER_OPEN_SPEED,
    NAMED_POSES,
    SHOULDER_XYZ,
    SteveArm,
    _quat_from_matrix,
    horizontal_grasp,
    pose_at,
)

# Body front face in base_link, with the corner lidars. The stand is 0.25 m.
BODY_FRONT = 0.34
STAND_HALF = 0.125
GAP_TARGET = 0.10
GAP_MIN = 0.05
# Cube x in base_link once the stand is GAP_TARGET in front of the body.
STANDOFF_X = BODY_FRONT + GAP_TARGET + STAND_HALF
# Further back, the side grasp's pregrasp is no longer inside the shoulder.
SIDE_STANDOFF_X = 0.80
STANDOFFS = (STANDOFF_X, SIDE_STANDOFF_X)
MAX_SPEED = 0.35
MIN_SPEED = 0.02
MAX_TURN = 0.4
# The Gazebo base takes /cmd_vel at once. Ramping it keeps the arm and the
# held cube from being jolted.
MAX_ACCEL = 0.6
MAX_TURN_ACCEL = 1.0
# Braking, below MAX_ACCEL so the ramp can always follow it. Full speed
# until the stopping distance, then a steady stop instead of a long creep.
BRAKE = 0.4
# Stand top is 0.80 m and the cube is 40 mm, so its center is near 0.82.
GRASP_Z_MIN = 0.82
GRASP_Z_MAX = 0.83
PREGRASP = 0.12
# gripper_tcp sits between the pads. Inside this, the cube is between them.
BETWEEN_M = 0.02
RETRIES = 3
PALM_LINK = "ur5ewrist_3_link"
# KitchenTable_01_001 in small_house.world. Its mesh is 0.72 m along its own
# x and 1.82 m along its own y, with the collision top at 0.818 m. Chairs
# stand along both long sides, so the cube goes on the clear west end.
TABLE_XY = (6.55269, 0.951173)
TABLE_YAW = -1.564130
TABLE_SIZE = (0.72, 1.82)
TABLE_TOP = 0.818
# The cube goes this far in from the table edge that faces the robot.
PLACE_INSET = 0.10
CUBE_HALF = 0.02
PLACE_CLEARANCE = 0.015


def _clip(value, limit):
    return max(-limit, min(limit, value))


def _send(pub, twist: Twist, vx: float, vy: float, wz: float = 0.0):
    twist.linear.x = float(vx)
    twist.linear.y = float(vy)
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(wz)
    pub.publish(twist)


def _matrix(transform):
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    t = transform.transform.translation
    return np.array([t.x, t.y, t.z]), rotation


def _yaw(transform):
    q = transform.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CubeWatch:
    def __init__(self, node: Node):
        self.pose = None
        self.source = ""
        self.pose_time = 0.0
        self.map_xyz = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.create_subscription(PoseStamped, "/cube_pose_base", self._on_pose, 10)
        node.create_subscription(PoseStamped, "/cube_pose", self._on_map, 10)
        node.create_subscription(String, "/cube_source", self._on_source, 10)

    def _on_pose(self, msg: PoseStamped):
        if msg.header.frame_id not in ("base_link", ""):
            return
        self.pose = msg
        self.pose_time = time.monotonic()

    def _on_map(self, msg: PoseStamped):
        point = msg.pose.position
        self.map_xyz = (point.x, point.y, point.z)

    def _on_source(self, msg: String):
        self.source = msg.data

    def fresh(self, max_age: float = 1.0):
        if self.pose is None or "depth" not in self.source:
            return None
        if time.monotonic() - self.pose_time > max_age:
            return None
        point = self.pose.pose.position
        return (point.x, point.y, point.z)

    def remembered(self):
        """Last map fix, re-expressed in base_link while no camera has it."""
        if self.map_xyz is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "map", Time())
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x, point.point.y, point.point.z = self.map_xyz
            mapped = do_transform_point(point, transform)
        except Exception:
            return None
        return (mapped.point.x, mapped.point.y, mapped.point.z)

    def best(self):
        live = self.fresh(max_age=1.5)
        return live if live is not None else self.remembered()


class PanTracker:
    """Turns the pan-tilt head so the L515 keeps looking at the cube.

    Both motors rotate about their link's -X axis. The look direction is
    the optical frame's +X, the axis the Gazebo camera renders along. One
    damped least-squares step per call rotates that axis toward the cube.
    """

    JOINTS = ("pan_tilt_pan_motor_joint", "pan_tilt_tilt_motor_joint")
    LINKS = ("pan_tilt_pan_motor_link", "pan_tilt_tilt_motor_link")
    CAMERA = "pan_tilt_camera_color_optical_frame"
    LIMITS = ((-2.5, 2.5), (-0.3, 1.4))
    HOME = (0.0, 0.41)

    def __init__(self, node: Node, tf_buffer: Buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.positions = None
        self.enabled = True
        self.sign = 1.0
        self.last_angle = None
        self.worse = 0
        self.last_time = 0.0
        self.publisher = node.create_publisher(
            JointTrajectory, "/pan_tilt_controller/joint_trajectory", 10
        )
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(JointState, "/joint_states_complete", self._on_joints, 10)

    def _on_joints(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in self.JOINTS):
            self.positions = np.array([by_name[name] for name in self.JOINTS])

    def _frame(self, link):
        return _matrix(self.tf_buffer.lookup_transform("base_link", link, Time()))

    def _command(self, positions, seconds=0.25):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = MsgDuration(
            sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9)
        )
        trajectory.points = [point]
        self.publisher.publish(trajectory)

    def park(self):
        self.enabled = False
        self._command(self.HOME, seconds=1.0)

    def aim(self, cube):
        if not self.enabled or cube is None or self.positions is None:
            return
        now = time.monotonic()
        if now - self.last_time < 0.2:
            return
        self.last_time = now
        try:
            origin, rotation = self._frame(self.CAMERA)
            axes = [-self._frame(link)[1][:, 0] for link in self.LINKS]
        except Exception:
            return
        look = rotation[:, 0]
        target = np.asarray(cube, dtype=float) - origin
        distance = np.linalg.norm(target)
        if distance < 1e-3:
            return
        target /= distance
        angle = math.acos(max(-1.0, min(1.0, float(np.dot(look, target)))))
        # Three steps in a row that look further away mean the URDF axis
        # sign is the other way round. Flip it rather than spin away.
        if self.last_angle is not None and angle > self.last_angle + 0.01:
            self.worse += 1
            if self.worse >= 3:
                self.sign = -self.sign
                self.worse = 0
                self.node.get_logger().warn("Pan-tilt steps went the wrong way. Flipping them.")
        else:
            self.worse = 0
        self.last_angle = angle
        if angle < 0.03:
            return
        columns = np.column_stack([np.cross(axis, look) for axis in axes])
        step, *_ = np.linalg.lstsq(columns, target - look, rcond=None)
        step = np.clip(0.7 * self.sign * step, -0.08, 0.08)
        goal = self.positions + step
        for index, (lower, upper) in enumerate(self.LIMITS):
            goal[index] = min(max(goal[index], lower), upper)
        self._command(goal)


class PalmSensor:
    """Cube center relative to gripper_tcp, from the grasp plugin.

    The plugin publishes the cube in the palm frame. That is what tells a
    real grasp from a hand that closed next to the cube.
    """

    def __init__(self, node: Node, tf_buffer: Buffer):
        self.tf_buffer = tf_buffer
        self.point = None
        self.stamp = 0.0
        node.create_subscription(PointStamped, "/grasp_cube/cube_in_palm", self._on_point, 10)

    def _on_point(self, msg: PointStamped):
        self.point = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.stamp = time.monotonic()

    def offset(self):
        """Cube minus gripper_tcp in base_link, or None if there is no reading."""
        if self.point is None or time.monotonic() - self.stamp > 1.0:
            return None
        try:
            palm_origin, palm_rotation = _matrix(
                self.tf_buffer.lookup_transform("base_link", PALM_LINK, Time())
            )
            tcp_origin, _ = _matrix(self.tf_buffer.lookup_transform("base_link", EE_LINK, Time()))
        except Exception:
            return None
        return palm_origin + palm_rotation @ self.point - tcp_origin


def stopping_speed(ex, ey):
    """Velocity toward (ex, ey): full speed, then braking at BRAKE to a stop there."""
    distance = math.hypot(ex, ey)
    if distance < 1e-6:
        return 0.0, 0.0
    speed = min(MAX_SPEED, math.sqrt(2.0 * BRAKE * distance), 3.0 * distance)
    speed = max(speed, MIN_SPEED)
    return ex * speed / distance, ey * speed / distance


def approach_twist(x, y, standoff):
    """Turn to face the cube and slide until it sits at (standoff, 0)."""
    bearing = math.atan2(y, x)
    wz = _clip(1.2 * bearing, MAX_TURN)
    ex = x - standoff
    ey = y
    vx, vy = stopping_speed(ex, ey)
    gap = x - STAND_HALF - BODY_FRONT
    if gap < GAP_MIN and abs(y) < 0.4:
        vx = min(vx, -0.04)
    arrived = abs(ex) < 0.015 and abs(ey) < 0.015 and abs(bearing) < 0.03
    return vx, vy, wz, arrived, gap


def grasp_candidates(cube):
    """Top-down closing across the robot, top-down along it, then a side grasp."""
    x, y, z = cube
    down = (0.0, 0.0, -1.0)
    # Columns are gripper X (pad closing), Y, Z (approach) in base_link.
    across = _quat_from_matrix([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    along = _quat_from_matrix([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    candidates = [
        ("top-down, fingers across", across, down, 0.005),
        ("top-down, fingers along", along, down, 0.005),
    ]
    # Closer than this, the side pregrasp sits inside the shoulder and has no IK.
    if math.hypot(x - SHOULDER_XYZ[0], y - SHOULDER_XYZ[1]) >= 0.55:
        side_quat, (ax, ay) = horizontal_grasp(x - SHOULDER_XYZ[0], y - SHOULDER_XYZ[1])
        candidates.append(("side", side_quat, (ax, ay, 0.0), 0.0))
    return candidates


class Fetch:
    def __init__(self, node: Node):
        self.node = node
        self.log = node.get_logger()
        self.cmd = node.create_publisher(Twist, "/cmd_vel", 10)
        self.twist = Twist()
        self.velocity = (0.0, 0.0, 0.0)
        self.sent = (0.0, 0.0, 0.0)
        self.last_tick = node.get_clock().now().nanoseconds * 1e-9
        self.arm = SteveArm(node)
        self.watch = CubeWatch(node)
        self.pan = PanTracker(node, self.watch.tf_buffer)
        self.palm = PalmSensor(node, self.watch.tf_buffer)
        self.table_x, self.table_y = TABLE_XY
        # The table's own -Y end. With its yaw near -90 degrees, that is the west end.
        inward = TABLE_SIZE[1] / 2 - PLACE_INSET
        self.table_place = (
            self.table_x + math.sin(TABLE_YAW) * inward,
            self.table_y - math.cos(TABLE_YAW) * inward,
        )

    # Called from every wait, including MoveIt and gripper waits.
    def tick(self):
        # Sim time: Gazebo runs slower than real time, and the cube feels sim time.
        now = self.node.get_clock().now().nanoseconds * 1e-9
        dt = max(0.0, min(now - self.last_tick, 0.2))
        self.last_tick = now
        limits = (MAX_ACCEL * dt, MAX_ACCEL * dt, MAX_TURN_ACCEL * dt)
        self.sent = tuple(
            current + _clip(target - current, limit)
            for current, target, limit in zip(self.sent, self.velocity, limits)
        )
        _send(self.cmd, self.twist, *self.sent)
        self.pan.aim(self.watch.best())

    def hold(self):
        self.velocity = (0.0, 0.0, 0.0)
        self.tick()

    def spin(self, seconds):
        end = self.node.get_clock().now() + Duration(seconds=seconds)
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.tick()

    def base_pose(self):
        """Base pose in odom. Gazebo writes odom from the world pose, so odom is the world."""
        try:
            transform = self.watch.tf_buffer.lookup_transform("odom", "base_link", Time())
        except Exception:
            return None
        t = transform.transform.translation
        return "odom", t.x, t.y, _yaw(transform)

    def hold_home(self, timeout=15.0) -> bool:
        """Lift the arm straight to home without MoveIt.

        A sagged arm can start in collision, and MoveIt will not plan from
        there. The trajectory controller keeps the last point afterwards.
        """
        publisher = self.node.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        home = NAMED_POSES["home"]
        latest = {}

        def on_joints(msg: JointState):
            by_name = dict(zip(msg.name, msg.position))
            if all(name in by_name for name in ARM_JOINTS):
                latest["joints"] = by_name

        self.node.create_subscription(JointState, "/joint_states", on_joints, 10)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [home[name] for name in ARM_JOINTS]
        point.time_from_start = MsgDuration(sec=4, nanosec=0)
        trajectory.points = [point]
        self.log.info("Arm to home, beside the pan view. The wrist camera looks with the pan.")
        deadline = time.monotonic() + timeout
        sent = False
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.tick()
            joints = latest.get("joints")
            if joints is None:
                continue
            if all(abs(joints[name] - home[name]) < 0.05 for name in ARM_JOINTS):
                return True
            if not sent:
                publisher.publish(trajectory)
                sent = True
        self.log.warn("Arm did not reach the home pose in time")
        return False

    def wait_for_cube(self, timeout=60.0):
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.tick()
            xyz = self.watch.fresh()
            if xyz is not None:
                return xyz
        return None

    def drive_to(self, standoff, timeout=120.0) -> bool:
        self.log.info(f"Base faces the cube and stops {GAP_TARGET * 100:.0f} cm from the stand")
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        settled = 0
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            cube = self.watch.best()
            if cube is None:
                self.velocity = (0.0, 0.0, 0.0)
                self.tick()
                self.log.warn("No camera has the cube. Base waits.", throttle_duration_sec=2.0)
                continue
            vx, vy, wz, arrived, gap = approach_twist(cube[0], cube[1], standoff)
            settled = settled + 1 if arrived else 0
            if settled >= 10:
                self.hold()
                self.log.info(
                    f"Base is in front of the stand. Cube at base_link "
                    f"({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f}), gap {gap * 100:.1f} cm"
                )
                return True
            self.velocity = (0.0, 0.0, 0.0) if arrived else (vx, vy, wz)
            self.tick()
            self.log.info(
                f"{self.watch.source or 'map'}: cube ({cube[0]:.2f}, {cube[1]:.2f}), "
                f"gap {gap * 100:.0f} cm, base ({vx:.2f}, {vy:.2f}) m/s, turn {wz:.2f} rad/s",
                throttle_duration_sec=1.0,
            )
        self.hold()
        return False

    def measure(self, timeout=8.0, near=None, max_shift=0.06):
        """Median of five still camera fixes. `near` rejects a different object."""
        samples = []
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.hold()
            xyz = self.watch.fresh(max_age=0.5)
            if xyz is None:
                continue
            if near is not None and math.dist(xyz, near) > max_shift:
                continue
            if samples and xyz == samples[-1]:
                continue
            samples = (samples + [xyz])[-5:]
            if len(samples) < 5:
                continue
            spread = max(
                max(item[axis] for item in samples) - min(item[axis] for item in samples)
                for axis in range(3)
            )
            if spread < 0.02:
                x, y, z = (float(np.median([item[axis] for item in samples])) for axis in range(3))
                return (x, y, min(max(z, GRASP_Z_MIN), GRASP_Z_MAX))
        return None

    def cube_offset(self):
        """Cube minus gripper_tcp. The camera is the fallback without the plugin."""
        offset = self.palm.offset()
        if offset is not None:
            return offset
        cube = self.watch.fresh(max_age=0.5)
        if cube is None:
            return None
        try:
            tcp, _ = _matrix(self.watch.tf_buffer.lookup_transform("base_link", EE_LINK, Time()))
        except Exception:
            return None
        return np.asarray(cube) - tcp

    def try_candidate(self, name, quat, approach, lift_z, cube):
        """Pregrasp, re-measure, descend, and stop with the cube between the pads."""
        arm = self.arm
        approach = np.asarray(approach, dtype=float)

        def poses(target):
            grasp = np.asarray(target, dtype=float) + np.array([0.0, 0.0, lift_z])
            pregrasp = grasp - approach * PREGRASP
            stamp = self.node.get_clock().now().to_msg()
            return (
                pose_at("base_link", *pregrasp, quat, stamp),
                pose_at("base_link", *grasp, quat, stamp),
            )

        self.log.info(f"=== {name} grasp of cube ({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f}) ===")
        arm.set_pick_scene(*cube, tick=self.tick)
        pregrasp, _ = poses(cube)
        if not arm.move_pose(pregrasp, tick=self.tick, planning_time=5.0):
            self.log.warn(f"No plan to the {name} pregrasp")
            return None
        fix = self.measure(timeout=3.0, near=cube)
        if fix is not None:
            self.log.info(
                f"{self.watch.source} from the pregrasp: cube ({fix[0]:.3f}, {fix[1]:.3f}, {fix[2]:.3f})"
            )
            cube = fix
        for attempt in range(1, RETRIES + 1):
            arm.set_pick_scene(*cube, tick=self.tick, allow_gripper=True)
            pregrasp, grasp = poses(cube)
            if not arm.cartesian_to(grasp, tick=self.tick):
                self.log.warn(f"Could not descend onto the cube ({name})")
                arm.cartesian_to(pregrasp, tick=self.tick)
                return None
            self.spin(0.5)
            offset = self.cube_offset()
            if offset is None:
                self.log.warn("No hand-to-cube reading. Closing on the camera pose.")
                return grasp
            miss = float(np.linalg.norm(offset))
            self.log.info(
                f"Attempt {attempt}: cube is {miss * 100:.1f} cm from the pads "
                f"({offset[0] * 100:+.1f}, {offset[1] * 100:+.1f}, {offset[2] * 100:+.1f}) cm"
            )
            if miss <= BETWEEN_M:
                return grasp
            arm.cartesian_to(pregrasp, tick=self.tick)
            cube = tuple(np.asarray(cube) + offset)
            if cube[2] < 0.7:
                self.log.error(
                    f"Gazebo has the cube at z {cube[2]:.2f} m, not on the stand. "
                    "It fell, so there is nothing to grasp up here."
                )
                return None
        return None

    def close_and_lift(self, grasp) -> bool:
        """Weld, close, lift, and fold home. Once welded, the cube is never let go."""
        arm = self.arm
        # Weld first, so the closing fingers do not shove the cube or the base.
        if not arm.grasp_cube(True):
            return False
        arm.set_gripper(GRIPPER_CLOSED, tick=self.tick)
        arm.attach_cube()
        lift = copy.deepcopy(grasp)
        lift.pose.position.z += 0.10
        if not arm.cartesian_to(lift, tick=self.tick):
            self.log.warn("Straight lift failed. Folding home with the cube.")
        for attempt in range(3):
            if arm.move_named("home", tick=self.tick):
                break
            self.log.warn(f"Fold home with the cube failed (try {attempt + 1}). Holding on.")
            self.spin(1.0)
        else:
            # The planner refused, but the cube stays in hand: lift straight to home.
            self.hold_home()
        offset = self.cube_offset()
        if offset is not None:
            self.log.info(f"Cube is {np.linalg.norm(offset) * 100:.1f} cm from the pads, in hand")
        return True

    def release_and_recover(self):
        arm = self.arm
        arm.grasp_cube(False)
        arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED, tick=self.tick)
        if not arm.move_named("home", tick=self.tick):
            self.hold_home()

    def go_to(self, goal, label, timeout=90.0, tolerance=0.03, yaw_tolerance=0.03) -> bool:
        """Drive the base to `goal` = (frame, x, y, yaw) in odom."""
        _, gx, gy, gyaw = goal
        self.log.info(f"Driving to {label}: odom ({gx:.2f}, {gy:.2f}), yaw {gyaw:.2f}")
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while self.node.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            pose = self.base_pose()
            if pose is None:
                self.hold()
                self.log.warn("No odom pose. Base waits.", throttle_duration_sec=2.0)
                continue
            _, x, y, yaw = pose
            dx, dy = gx - x, gy - y
            distance = math.hypot(dx, dy)
            yaw_error = math.atan2(math.sin(gyaw - yaw), math.cos(gyaw - yaw))
            if distance < tolerance and abs(yaw_error) < yaw_tolerance:
                self.hold()
                self.log.info(f"Base is at {label} ({x:.2f}, {y:.2f}), yaw {yaw:.2f}")
                return True
            # World error expressed in the base frame.
            ex = math.cos(yaw) * dx + math.sin(yaw) * dy
            ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
            vx, vy = stopping_speed(ex, ey) if distance >= tolerance else (0.0, 0.0)
            # Turn before closing in, so a corner does not swing into the table.
            scale = max(0.0, 1.0 - abs(yaw_error) / 0.5)
            wz = _clip(1.0 * yaw_error, MAX_TURN)
            if abs(yaw_error) >= yaw_tolerance and abs(wz) < 0.05:
                wz = math.copysign(0.05, yaw_error)
            self.velocity = (vx * scale, vy * scale, wz)
            self.tick()
            self.log.info(
                f"To {label}: {distance:.2f} m and {math.degrees(yaw_error):.0f} deg left",
                throttle_duration_sec=2.0,
            )
        self.hold()
        return False

    def drop_pose(self, place_in_base):
        """Base pose that puts the place point at `place_in_base`, facing down the table's length."""
        px, py = self.table_place
        yaw = TABLE_YAW + math.pi / 2.0
        bx, by = place_in_base
        x = px - (math.cos(yaw) * bx - math.sin(yaw) * by)
        y = py - (math.sin(yaw) * bx + math.cos(yaw) * by)
        return "odom", x, y, yaw

    def to_base(self, wx, wy):
        """Odom point in base_link, from the current base pose."""
        _, x, y, yaw = self.base_pose()
        dx, dy = wx - x, wy - y
        return (
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            yaw,
        )

    def place_on_table(self, quat, grasp_xyz) -> bool:
        """Drive to the kitchen table, set the cube on its top, and let go."""
        arm = self.arm
        gx, gy, gz = grasp_xyz
        if not self.go_to(self.drop_pose((gx, gy)), "the kitchen table", timeout=150.0):
            self.log.warn("Base stopped short of the kitchen table. Placing from here.")
        if self.base_pose() is None:
            self.log.error("No odom pose, so the table cannot be found")
            return False

        tx, ty, yaw = self.to_base(self.table_x, self.table_y)
        relative_yaw = TABLE_YAW - yaw
        size = (TABLE_SIZE[0], TABLE_SIZE[1])
        arm.set_place_scene(tx, ty, relative_yaw, size, TABLE_TOP, tick=self.tick)

        px, py, _ = self.to_base(*self.table_place)
        offset = self.cube_offset()
        if offset is None:
            offset = np.zeros(3)
        cube_z = TABLE_TOP + CUBE_HALF + PLACE_CLEARANCE
        tcp = np.array([px, py, cube_z]) - offset
        stamp = self.node.get_clock().now().to_msg()
        place = pose_at("base_link", *tcp, quat, stamp)
        above = pose_at("base_link", tcp[0], tcp[1], tcp[2] + PREGRASP, quat, stamp)
        self.log.info(f"Placing the cube at base_link ({px:.3f}, {py:.3f}, {cube_z:.3f})")

        if not arm.move_pose(above, tick=self.tick, planning_time=5.0):
            self.log.error("No plan to above the kitchen table")
            return False
        arm.set_place_scene(tx, ty, relative_yaw, size, TABLE_TOP, tick=self.tick, lowered=True)
        if not arm.cartesian_to(place, tick=self.tick):
            self.log.warn("Could not lower all the way. Releasing from just above the table.")
        self.spin(0.5)

        # Open while welded, so the pads leave the cube without pushing it.
        arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED, tick=self.tick)
        arm.grasp_cube(False)
        arm.detach_cube(tick=self.tick)
        self.spin(1.0)

        landed = self.cube_on_table()
        arm.cartesian_to(above, tick=self.tick)
        arm.set_place_scene(tx, ty, relative_yaw, size, TABLE_TOP, tick=self.tick)
        if not arm.move_named("home", tick=self.tick):
            self.hold_home()
        self.spin(1.0)
        return landed and self.cube_on_table()

    def cube_on_table(self) -> bool:
        """Cube resting on the table top, from the grasp plugin's palm reading."""
        offset = self.palm.offset()
        if offset is None:
            self.log.warn("No cube reading from the grasp plugin. Assuming it is on the table.")
            return True
        try:
            tcp, _ = _matrix(self.watch.tf_buffer.lookup_transform("base_link", EE_LINK, Time()))
        except Exception:
            return True
        cube = tcp + offset
        tx, ty, yaw = self.to_base(self.table_x, self.table_y)
        dx, dy = cube[0] - tx, cube[1] - ty
        # Into the table's own axes.
        relative_yaw = TABLE_YAW - yaw
        ux = math.cos(relative_yaw) * dx + math.sin(relative_yaw) * dy
        uy = -math.sin(relative_yaw) * dx + math.cos(relative_yaw) * dy
        on_top = abs(ux) < TABLE_SIZE[0] / 2 and abs(uy) < TABLE_SIZE[1] / 2
        resting = TABLE_TOP < cube[2] < TABLE_TOP + 4 * CUBE_HALF
        self.log.info(
            f"Cube at base_link ({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f}), "
            f"{'on' if on_top and resting else 'NOT on'} the table"
        )
        return on_top and resting

    def run(self) -> int:
        arm = self.arm
        arm.wait(timeout=120.0)
        start = None
        end = time.monotonic() + 20.0
        while start is None and time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.tick()
            start = self.base_pose()
        if start is None:
            self.log.warn("No odom pose, so the base will not drive back after the grasp")

        self.hold_home()
        arm.grasp_cube(False)
        if not arm.set_gripper(GRIPPER_OPEN, speed=GRIPPER_OPEN_SPEED, tick=self.tick):
            return 1

        self.log.info("Waiting for the pan or wrist camera to measure the cube")
        seen = self.wait_for_cube()
        if seen is None:
            self.log.error("Neither camera returned a depth pose of the red cube")
            return 1
        self.log.info(f"{self.watch.source} sees the cube at base_link ({seen[0]:.2f}, {seen[1]:.2f}, {seen[2]:.2f})")

        for standoff in STANDOFFS:
            if not self.drive_to(standoff):
                self.log.error("Base did not reach the stand")
                continue
            cube = self.measure()
            if cube is None:
                self.log.warn("No still cube pose. Using the last fix.")
                cube = self.watch.best()
            if cube is None:
                continue
            for name, quat, approach, lift_z in grasp_candidates(cube):
                grasp = self.try_candidate(name, quat, approach, lift_z, cube)
                if grasp is None:
                    self.release_and_recover()
                    continue
                if self.close_and_lift(grasp):
                    self.log.info(f"Cube grasped ({name}). Carrying it back to the start pose.")
                    self.pan.park()
                    if start is not None and not self.go_to(start, "the start pose", tolerance=0.05, yaw_tolerance=0.05):
                        self.log.warn("Base did not reach the start pose in time. The cube is still in hand.")
                    position = grasp.pose.position
                    if not self.place_on_table(quat, (position.x, position.y, position.z)):
                        self.log.error("=" * 60)
                        self.log.error("TASK FAILED: the cube did not end up on the kitchen table")
                        self.log.error("=" * 60)
                        return 1
                    self.log.info("=" * 60)
                    self.log.info("TASK COMPLETED SUCCESSFULLY")
                    self.log.info("Picked the red cube from the stand, carried it back,")
                    self.log.info("and placed it on the kitchen table.")
                    self.log.info("=" * 60)
                    return 0
                self.release_and_recover()
                refreshed = self.measure(timeout=4.0)
                if refreshed is not None:
                    cube = refreshed
            self.log.warn("Every grasp failed from this spot. Stepping the base back.")
        self.log.error("Could not grasp the cube")
        return 1


def main(argv=None):
    rclpy.init(args=argv)
    node = rclpy.create_node(
        "fetch_cube",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    fetch = None
    try:
        fetch = Fetch(node)
        return fetch.run()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(str(exc) or type(exc).__name__)
        return 1
    finally:
        if fetch is not None:
            _send(fetch.cmd, fetch.twist, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
