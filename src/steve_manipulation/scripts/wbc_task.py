"""Phases of the whole-body pick, without ROS.

The plan turns the cube estimate and the robot state into one Task for
the whole-body MPC at every control cycle:

APPROACH  The base drives to the standoff in front of the stand. At the
          same time the arm unfolds from home into the pregrasp posture.
          Once the base is within RAMP_FAR of the standoff, the pregrasp
          pose in odom fades in as the hand task, so the arm absorbs the
          last base error instead of waiting for the base to stop.
DESCEND   As soon as the base is near the standoff and the hand is on the
          pregrasp, the hand goes straight down onto the cube. The base
          is still controlled and still settling.
GRASP     The hand holds on the cube while the fingers close.
LIFT      The hand rises straight up with the cube.
CARRY     The base backs off to where it started while the arm folds
          home with the cube.

A miss at the bottom of the descent (the cube is not between the pads)
goes through RISE and back to APPROACH with the corrected cube position.
"""

import math
from enum import Enum

import numpy as np

from wbc_solver import Task

ARM_JOINTS = (
    "ur5eshoulder_pan_joint",
    "ur5eshoulder_lift_joint",
    "ur5eelbow_joint",
    "ur5ewrist_1_joint",
    "ur5ewrist_2_joint",
    "ur5ewrist_3_joint",
)
HOME = np.array([-0.9423, -0.8950, 1.8669, -1.1554, -0.9506, 0.1067])
IK_SEEDS = (
    HOME,
    np.array([1.57, -1.5709, 2.530, -0.960, 1.57, 0.0]),
    np.array([0.614, -0.994, 1.530, -0.515, 0.169, -0.357]),
    np.array([-0.4, -1.01, 2.2, -1.29, 1.2, 0.0]),
)
KEEPOUT_LINKS = ("ur5eforearm_link", "ur5ewrist_1_link", "ur5ewrist_2_link", "ur5ewrist_3_link")

# Same geometry as fetch_cube.py.
BODY_FRONT = 0.34
STAND_HALF = 0.125
STAND_TOP = 0.80
GAP_TARGET = 0.10
GAP_MIN = 0.05
STANDOFF_X = BODY_FRONT + GAP_TARGET + STAND_HALF
PREGRASP = 0.12
GRASP_ABOVE_CENTER = 0.005
LIFT = 0.15
GRASP_Z_MIN = 0.82
GRASP_Z_MAX = 0.83

# Hand task fades in between these base distances to the standoff.
RAMP_FAR = 0.9
RAMP_NEAR = 0.25
# On the approach the base speed limit shrinks in proportion to the
# distance to the standoff inside SLOW_DISTANCE, down to SLOW_MIN of the
# full limit. The MPC applies it to every predicted step, so the base
# brakes into the stand however long a plan runs.
SLOW_DISTANCE = 0.6
SLOW_MIN = 0.15
# Descent starts once the base is this close and the hand is on the pregrasp.
DESCEND_BASE = 0.06
DESCEND_YAW = 0.06
ON_POSE = 0.015
ON_ROTATION = 0.06
AT_GRASP = 0.006
DESCEND_SPEED = 0.05
LIFT_SPEED = 0.06
SETTLE = 0.4
STEP_TIMEOUT = 12.0
APPROACH_TIMEOUT = 120.0
CARRY_DONE = 0.05
RETRIES = 3

W_HAND = 400.0
W_HAND_ROTATION = 60.0
W_BASE = 30.0
W_BASE_YAW = 30.0
W_POSTURE = 8.0
W_POSTURE_LOW = 0.3


class Phase(Enum):
    APPROACH = "approach"
    DESCEND = "descend"
    AT_GRASP = "at grasp"
    GRASP = "grasp"
    LIFT = "lift"
    RISE = "rise"
    CARRY = "carry"
    DONE = "done"
    FAILED = "failed"


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_near(q, reference, lower, upper):
    """Each joint on the 2π turn closest to `reference`, inside its limits."""
    out = np.array(q, dtype=float)
    for i, (value, ref) in enumerate(zip(q, reference)):
        best = None
        for turns in range(-3, 4):
            candidate = value + 2.0 * math.pi * turns
            if lower[i] <= candidate <= upper[i] and (best is None or abs(candidate - ref) < abs(best - ref)):
                best = candidate
        if best is not None:
            out[i] = best
    return out


def rotation_angle(a, b):
    cosine = (np.trace(a.T @ b) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, cosine)))


class LineReference:
    """A point that slides from `start` to `goal` at `speed`."""

    def __init__(self, start, goal, speed, now):
        self.start = np.asarray(start, dtype=float)
        self.goal = np.asarray(goal, dtype=float)
        self.started = now
        length = float(np.linalg.norm(self.goal - self.start))
        self.duration = length / speed if speed > 0.0 else 0.0

    def at(self, now):
        if self.duration <= 1e-6:
            return self.goal.copy()
        fraction = min(1.0, max(0.0, (now - self.started) / self.duration))
        return self.start + fraction * (self.goal - self.start)

    def finished(self, now):
        return now - self.started >= self.duration


class PickPlan:
    def __init__(self, chain, start_base, cube_xyz, cube_yaw=0.0, now=0.0, log=print):
        self.chain = chain
        self.log = log
        self.start_base = np.asarray(start_base, dtype=float)
        self.cube_yaw = cube_yaw
        self.retries = 0
        self.phase = Phase.APPROACH
        self.phase_started = now
        self.reference = None
        self.settled_since = None
        self.reason = ""
        self.carry_posture = HOME.copy()
        bx, by, _ = self.start_base
        self.heading = math.atan2(cube_xyz[1] - by, cube_xyz[0] - bx)
        self.set_cube(cube_xyz)

    # ------------------------------------------------------------ geometry
    def set_cube(self, cube_xyz):
        x, y, z = (float(value) for value in cube_xyz)
        z = min(max(z, GRASP_Z_MIN), GRASP_Z_MAX)
        self.cube = np.array([x, y, z])
        self.stand_xy = self.cube[:2].copy()
        self.grasp = self.cube + np.array([0.0, 0.0, GRASP_ABOVE_CENTER])
        self.pregrasp = self.grasp + np.array([0.0, 0.0, PREGRASP])
        self.lift = self.grasp + np.array([0.0, 0.0, LIFT])
        direction = np.array([math.cos(self.heading), math.sin(self.heading)])
        self.base_goal = self.stand_xy - STANDOFF_X * direction
        self._choose_grasp()

    def _choose_grasp(self):
        """Top-down grasp with the pads on two cube faces, and its arm posture.

        The pads close along gripper X. Of the four face directions, the two
        closest to the base's sideways axis are tried, since the gripper is
        symmetric. The posture is the IK solution closest to home with the
        elbow up, for the pregrasp seen from the standoff.
        """
        across = self.heading + math.pi / 2.0
        face = self.cube_yaw + round((across - self.cube_yaw) / (math.pi / 2.0)) * (math.pi / 2.0)
        base_rotation = _rz(self.heading)
        target = np.array([STANDOFF_X, 0.0, self.pregrasp[2]])
        best = None
        for flip in (0.0, math.pi):
            yaw = face + flip
            x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            z_axis = np.array([0.0, 0.0, -1.0])
            world = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])
            local = base_rotation.T @ world
            for seed in IK_SEEDS:
                q, converged = self.chain.solve_ik(target, local, seed)
                if not converged:
                    continue
                q = wrap_near(q, HOME, self.chain.lower, self.chain.upper)
                _hand, points = self.chain.evaluate(q)
                elbow_up = points["ur5eforearm_link"][0][2] > points["ur5ewrist_1_link"][0][2]
                score = float(np.sum(np.abs(q - HOME))) + (0.0 if elbow_up else 100.0)
                if best is None or score < best[0]:
                    best = (score, q, world)
        if best is None:
            raise RuntimeError("No IK solution for the pregrasp at the standoff")
        _score, self.pregrasp_posture, self.hand_rotation = best
        self.log(
            "Pregrasp posture "
            + ", ".join(f"{value:.2f}" for value in self.pregrasp_posture)
            + f" for the cube at odom ({self.cube[0]:.3f}, {self.cube[1]:.3f}, {self.cube[2]:.3f})"
        )

    # --------------------------------------------------------------- state
    def hand(self, base, q):
        p_arm, r_arm = self.chain.tip_pose(q)
        rz = _rz(base[2])
        return np.array([base[0], base[1], 0.0]) + rz @ p_arm, rz @ r_arm

    def base_error(self, base):
        return float(np.linalg.norm(np.asarray(base[:2]) - self.base_goal)), abs(_wrap(base[2] - self.heading))

    def _enter(self, phase, now, reason=""):
        if phase != self.phase:
            self.log(f"WBC phase: {self.phase.value} -> {phase.value}" + (f" ({reason})" if reason else ""))
        self.phase = phase
        self.phase_started = now
        self.settled_since = None
        if phase in (Phase.DONE, Phase.FAILED):
            self.reason = reason

    def _settled(self, condition, now):
        if not condition:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = now
        return now - self.settled_since >= SETTLE

    def update(self, base, q, now):
        """Advance the phase from the measured state."""
        base = np.asarray(base, dtype=float)
        hand_p, hand_r = self.hand(base, q)
        elapsed = now - self.phase_started
        base_distance, yaw_error = self.base_error(base)

        if self.phase == Phase.APPROACH:
            on_pregrasp = (
                np.linalg.norm(hand_p - self.pregrasp) < ON_POSE
                and rotation_angle(hand_r, self.hand_rotation) < ON_ROTATION
            )
            near = base_distance < DESCEND_BASE and yaw_error < DESCEND_YAW
            if near and on_pregrasp:
                self.reference = LineReference(self.pregrasp, self.grasp, DESCEND_SPEED, now)
                self._enter(
                    Phase.DESCEND,
                    now,
                    f"base {base_distance * 100:.1f} cm from the standoff, hand on the pregrasp",
                )
            elif elapsed > APPROACH_TIMEOUT:
                self._enter(Phase.FAILED, now, "the approach timed out")
        elif self.phase == Phase.DESCEND:
            target = self.reference.at(now)
            close = np.linalg.norm(hand_p - self.grasp) < AT_GRASP
            if self.reference.finished(now) and self._settled(close, now):
                self._enter(Phase.AT_GRASP, now, "hand is on the grasp pose")
            elif self.reference.finished(now) and elapsed - self.reference.duration > STEP_TIMEOUT:
                self._enter(Phase.AT_GRASP, now, f"hand stopped {np.linalg.norm(hand_p - target) * 100:.1f} cm short")
        elif self.phase == Phase.RISE:
            if self.reference.finished(now) and hand_p[2] > self.pregrasp[2] - 0.02:
                self._enter(Phase.APPROACH, now, "back above the stand")
            elif elapsed > STEP_TIMEOUT + self.reference.duration:
                self._enter(Phase.APPROACH, now, "rise timed out")
        elif self.phase == Phase.LIFT:
            if self.reference.finished(now) and self._settled(hand_p[2] > self.lift[2] - 0.02, now):
                self._enter(Phase.CARRY, now, "cube lifted off the stand")
            elif elapsed > STEP_TIMEOUT + self.reference.duration:
                self._enter(Phase.CARRY, now, "lift timed out")
        elif self.phase == Phase.CARRY:
            distance = float(np.linalg.norm(base[:2] - self.start_base[:2]))
            yaw = abs(_wrap(base[2] - self.start_base[2]))
            folded = float(np.max(np.abs(np.asarray(q) - self.carry_posture))) < 0.08
            if self._settled(distance < CARRY_DONE and yaw < 0.05 and folded, now):
                self._enter(Phase.DONE, now, "back at the start with the cube")
            elif elapsed > APPROACH_TIMEOUT:
                self._enter(Phase.FAILED, now, "the carry back timed out")
        return self.phase

    # ---------------------------------------------------------- transitions
    def missed(self, offset, now, base, q):
        """Cube is `offset` (odom) away from the pads. Go up and try again."""
        self.retries += 1
        if self.retries > RETRIES:
            self._enter(Phase.FAILED, now, "the cube never ended up between the pads")
            return
        hand_p, _ = self.hand(base, q)
        corrected = hand_p + np.asarray(offset, dtype=float)
        self.log(
            f"Cube is {np.linalg.norm(offset) * 100:.1f} cm from the pads. "
            f"Retry {self.retries}/{RETRIES} at the corrected position."
        )
        self.set_cube(corrected)
        up = np.array([hand_p[0], hand_p[1], self.pregrasp[2]])
        self.reference = LineReference(hand_p, up, DESCEND_SPEED, now)
        self._enter(Phase.RISE, now, "the cube is not between the pads")

    def start_grasp(self, now):
        self._enter(Phase.GRASP, now, "cube is between the pads")

    def start_lift(self, now, base, q):
        hand_p, _ = self.hand(base, q)
        self.lift = hand_p + np.array([0.0, 0.0, LIFT])
        self.reference = LineReference(hand_p, self.lift, LIFT_SPEED, now)
        self._enter(Phase.LIFT, now, "fingers closed on the cube")

    # ----------------------------------------------------------------- task
    def task(self, base, now) -> Task:
        base = np.asarray(base, dtype=float)
        task = Task(
            stand_xy=self.stand_xy,
            stand_half=STAND_HALF,
            stand_top=STAND_TOP,
            base_clearance=BODY_FRONT + STAND_HALF + GAP_MIN,
            hand_rotation=self.hand_rotation,
            base_yaw=self.heading,
            base_yaw_weight=W_BASE_YAW,
        )
        hand_clear = {name: 0.06 for name in KEEPOUT_LINKS}
        if self.phase == Phase.APPROACH:
            base_distance, _ = self.base_error(base)
            ramp = min(1.0, max(0.0, (RAMP_FAR - base_distance) / (RAMP_FAR - RAMP_NEAR)))
            if self.retries:
                ramp = 1.0
            task.base_xy = self.base_goal
            task.base_weight = W_BASE
            task.slow_xy = self.base_goal
            task.slow_distance = SLOW_DISTANCE
            task.slow_min = SLOW_MIN
            task.posture = self.pregrasp_posture
            task.posture_weight = W_POSTURE * (1.0 - ramp) + W_POSTURE_LOW * ramp
            task.hand_position = self.pregrasp
            task.hand_weight = W_HAND * ramp
            task.hand_rotation_weight = W_HAND_ROTATION * ramp
            task.stand_clearance = dict(hand_clear, tcp=0.10)
        elif self.phase in (Phase.DESCEND, Phase.AT_GRASP, Phase.GRASP, Phase.RISE, Phase.LIFT):
            if self.phase in (Phase.DESCEND, Phase.RISE, Phase.LIFT):
                target = self.reference.at(now)
            else:
                target = self.grasp
            task.base_xy = self.base_goal
            task.base_weight = W_BASE * 3.0
            task.base_speed_scale = 0.3
            task.posture = self.pregrasp_posture
            task.posture_weight = W_POSTURE_LOW
            task.hand_position = target
            task.hand_weight = W_HAND
            task.hand_rotation_weight = W_HAND_ROTATION
            # The hand is meant to go down to the cube, so only the links
            # above the hand keep their distance from the stand top.
            task.stand_clearance = dict(hand_clear)
            task.terminal_scale = 1.0
        elif self.phase == Phase.CARRY:
            task.base_xy = self.start_base[:2]
            task.base_yaw = self.start_base[2]
            task.base_weight = W_BASE
            task.posture = self.carry_posture
            task.posture_weight = W_POSTURE
            task.hand_rotation_weight = 0.0
            # The cube hangs 2 cm under gripper_tcp.
            task.stand_clearance = dict(hand_clear, tcp=0.08)
        else:
            task.base_xy = np.asarray(base[:2])
            task.base_yaw = float(base[2])
            task.base_weight = W_BASE
            task.hand_rotation_weight = 0.0
        return task
