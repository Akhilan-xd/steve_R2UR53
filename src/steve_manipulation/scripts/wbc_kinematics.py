"""Forward kinematics and Jacobians of one URDF chain, in numpy.

The whole-body controller needs the hand pose and its Jacobian many times
per control cycle, at every step of its prediction horizon. Reading TF is
too slow for that and only gives the current state, so the chain is built
once from the same robot_description Gazebo and robot_state_publisher use.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_rotation(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return np.array(
        [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ]
    )


def _origin(joint):
    transform = np.eye(4)
    origin = joint.find("origin")
    if origin is None:
        return transform
    xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    transform[:3, :3] = rpy_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


class Chain:
    """Serial chain from `root` to `tip` with revolute joints `joint_names`.

    `points` are links on that chain whose origins the controller keeps
    out of obstacles. Every query returns their positions and Jacobians.
    """

    def __init__(self, urdf_xml: str, root: str, tip: str, joint_names, points=()):
        robot = ET.fromstring(urdf_xml)
        by_child = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
        path = []
        link = tip
        while link != root:
            if link not in by_child:
                raise ValueError(f"{tip} is not below {root} in the URDF")
            joint = by_child[link]
            path.append(joint)
            link = joint.find("parent").get("link")
        path.reverse()

        self.joint_names = list(joint_names)
        self.lower = np.full(len(self.joint_names), -2.0 * math.pi)
        self.upper = np.full(len(self.joint_names), 2.0 * math.pi)
        # Each step is a fixed transform, then an optional joint rotation.
        # Runs of fixed joints are folded into the transform before them.
        self.steps = []
        fixed = np.eye(4)
        point_names = set(points)
        self.point_names = []
        for joint in path:
            name = joint.get("name")
            child = joint.find("child").get("link")
            fixed = fixed @ _origin(joint)
            if joint.get("type") in ("revolute", "continuous"):
                if name not in self.joint_names:
                    raise ValueError(f"Joint {name} is on the chain but not in joint_names")
                index = self.joint_names.index(name)
                axis = joint.find("axis")
                direction = np.array(
                    [float(value) for value in (axis.get("xyz") if axis is not None else "1 0 0").split()]
                )
                direction /= np.linalg.norm(direction)
                limit = joint.find("limit")
                if limit is not None and joint.get("type") == "revolute":
                    self.lower[index] = float(limit.get("lower", -2.0 * math.pi))
                    self.upper[index] = float(limit.get("upper", 2.0 * math.pi))
                self.steps.append((fixed, index, direction, child))
                fixed = np.eye(4)
            elif child in point_names:
                self.steps.append((fixed, None, None, child))
                fixed = np.eye(4)
        self.tip_offset = fixed
        if tip in point_names:
            self.tip_offset = np.eye(4)
            self.steps.append((fixed, None, None, tip))
        self.tip = tip
        self.point_names = [step[3] for step in self.steps if step[3] in point_names]
        missing = point_names - set(self.point_names)
        if missing:
            raise ValueError(f"Links {sorted(missing)} are not on the chain to {tip}")
        found = {step[1] for step in self.steps if step[1] is not None}
        if found != set(range(len(self.joint_names))):
            raise ValueError("joint_names has joints that are not on the chain")

    @property
    def dof(self):
        return len(self.joint_names)

    def evaluate(self, q, jacobians=True):
        """Tip pose and Jacobian, and every keep-out point with its Jacobian.

        Returns tip (p, R, Jp, Jw) and a dict link -> (p, Jp), all in `root`.
        Jacobian columns follow joint_names. Without `jacobians`, the
        Jacobians are None.
        """
        q = np.asarray(q, dtype=float)
        n = self.dof
        transform = np.eye(4)
        origins = np.zeros((n, 3))
        axes = np.zeros((n, 3))
        seen = []
        points = {}

        def linear(p):
            if not jacobians:
                return None
            jp = np.zeros((3, n))
            if seen:
                jp[:, seen] = _cross(axes[seen], p - origins[seen]).T
            return jp

        for fixed, index, direction, child in self.steps:
            transform = transform @ fixed
            if index is not None:
                origins[index] = transform[:3, 3]
                axes[index] = transform[:3, :3] @ direction
                rotation = np.eye(4)
                rotation[:3, :3] = axis_rotation(direction, q[index])
                transform = transform @ rotation
                seen.append(index)
            if child in self.point_names:
                p = transform[:3, 3].copy()
                points[child] = (p, linear(p))
        tip = transform @ self.tip_offset
        p = tip[:3, 3].copy()
        jw = None
        if jacobians:
            jw = np.zeros((3, n))
            jw[:, seen] = axes[seen].T
        return (p, tip[:3, :3].copy(), linear(p), jw), points

    def tip_pose(self, q):
        (p, rotation, _jp, _jw), _points = self.evaluate(q)
        return p, rotation

    def solve_ik(self, position, rotation, seed, iterations=300, tolerance=1e-4):
        """Damped least-squares IK for a tip pose. Returns (q, converged)."""
        q = np.array(seed, dtype=float)
        for _ in range(iterations):
            (p, current, jp, jw), _points = self.evaluate(q)
            error = np.concatenate([position - p, orientation_error(current, rotation)])
            if np.linalg.norm(error) < tolerance:
                return q, True
            jacobian = np.vstack([jp, jw])
            damping = 1e-3
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
            q = np.clip(q + np.clip(step, -0.2, 0.2), self.lower, self.upper)
        return q, False


def _cross(a, b):
    """Row-wise cross product. np.cross is several times slower on small arrays."""
    return np.stack(
        [
            a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1],
            a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
            a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0],
        ],
        axis=-1,
    )


def orientation_error(current, desired):
    """Rotation vector that turns `current` into `desired`, for small errors.

    0.5 * sum of column cross products. Near the goal it equals the
    axis-angle error, and it stays well defined far from it.
    """
    return 0.5 * _cross(current.T, desired.T).sum(axis=0)


def orientation_error_jacobian(current, desired):
    """d(orientation_error) / d(world rotation of `current`)."""
    return 0.5 * (current @ desired.T - float(np.trace(current.T @ desired)) * np.eye(3))


def rotation_residual(current, desired):
    """Columns of current - desired, scaled so its norm is the angle for small errors.

    Unlike orientation_error it is largest, not zero, at a half turn, so a
    least-squares cost on it has no false minimum with the hand flipped.
    """
    return (current - desired).T.ravel() / math.sqrt(2.0)


def rotation_residual_jacobian(current):
    """d(rotation_residual) / d(world rotation of `current`): each column c moves by w x c."""
    rows = []
    for c in current.T:
        rows.append(np.array([[0.0, c[2], -c[1]], [-c[2], 0.0, c[0]], [c[1], -c[0], 0.0]]))
    return np.vstack(rows) / math.sqrt(2.0)
