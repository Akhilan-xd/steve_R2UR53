"""Whole-body model predictive control for Steve, solved by SQP.

One optimisation moves the omnidirectional base and the UR5e together.

State  x = [bx, by, yaw, q1..q6]   base pose in odom, arm joints
Input  u = [vx, vy, wz, dq1..dq6]  base twist in base_link, joint speeds

The base integrates its body twist through its yaw, so the hand pose in
odom depends on base and arm at once:

    p_hand = [bx, by, 0] + Rz(yaw) p_arm(q)
    R_hand = Rz(yaw) R_arm(q)

Over a horizon of N steps the controller minimises the hand, base, and
posture errors plus the effort and jerk of every input. The hard
constraints are the base and joint speed and acceleration limits, the
joint range, and a minimum distance between the base and the stand.
Obstacles for the arm (the stand top, the pan-tilt tower, the body, the
floor) are hinge penalties, so a bad start never makes the problem
infeasible.

Each SQP iteration rolls the model out, linearises it with forward
sensitivities, takes the Gauss-Newton Hessian of the least-squares cost,
and solves the resulting QP with ADMM (the OSQP iteration, dense). The
previous solution, shifted by one step, warm-starts the next cycle, so
one or two iterations per cycle are enough (real-time iteration).
"""

import math
import multiprocessing as mp
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import cho_factor, cho_solve

from wbc_kinematics import Chain, rotation_residual, rotation_residual_jacobian

NB = 3
NA = 6
NX = NB + NA
NU = NB + NA


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Limits:
    base_speed: tuple = (0.35, 0.35, 0.4)
    base_accel: tuple = (0.6, 0.6, 1.0)
    joint_speed: float = 0.8
    joint_accel: float = 2.0


@dataclass
class Box:
    """Axis-aligned keep-out in base_link: points inside must stay above `top`."""

    x: tuple
    y: tuple
    top: float


@dataclass
class Task:
    """What the whole body should do. Everything is in odom unless noted."""

    hand_position: np.ndarray = None
    hand_rotation: np.ndarray = None
    hand_weight: float = 0.0
    hand_rotation_weight: float = 0.0
    base_xy: np.ndarray = None
    base_yaw: float = None
    base_weight: float = 0.0
    base_yaw_weight: float = 0.0
    posture: np.ndarray = None
    posture_weight: float = 0.0
    # Stand: center xy in odom, half width, top height.
    stand_xy: np.ndarray = None
    stand_half: float = 0.125
    stand_top: float = 0.80
    # How far each keep-out point stays above the stand top. A missing
    # link is not checked against the stand (the hand during the descent).
    stand_clearance: dict = field(default_factory=dict)
    base_clearance: float = 0.0
    base_speed_scale: float = 1.0
    # Within slow_distance of slow_xy the base speed limit of each step
    # shrinks with the distance its predicted state has left, down to
    # slow_min of the limit, so a plan brakes into the goal however long
    # it runs.
    slow_xy: np.ndarray = None
    slow_distance: float = 0.0
    slow_min: float = 1.0
    terminal_scale: float = 3.0


class AdmmQP:
    """min 0.5 x'Px + q'x  s.t.  l <= Ax <= u. Dense OSQP iteration."""

    def __init__(self, sigma=1e-6, alpha=1.6, rho=0.1, max_iter=400, eps=1e-4):
        self.sigma = sigma
        self.alpha = alpha
        self.rho = rho
        self.max_iter = max_iter
        self.eps = eps
        self.iterations = 0

    def solve(self, P, q, A, l, u, warm=None):
        n = P.shape[0]
        m = A.shape[0]
        equality = np.abs(u - l) < 1e-9
        rho = self.rho
        if warm is not None and warm[0].shape == (n,) and warm[2].shape == (m,):
            x, y = warm[0].copy(), warm[2].copy()
        else:
            x, y = np.zeros(n), np.zeros(m)
        z = np.clip(A @ x, l, u)

        def factor(rho_scalar):
            rho_vec = np.where(equality, 1e3 * rho_scalar, rho_scalar)
            kkt = P + self.sigma * np.eye(n) + A.T @ (rho_vec[:, None] * A)
            return rho_vec, cho_factor(kkt)

        rho_vec, chol = factor(rho)
        for iteration in range(1, self.max_iter + 1):
            rhs = self.sigma * x - q + A.T @ (rho_vec * z - y)
            x_tilde = cho_solve(chol, rhs)
            z_tilde = A @ x_tilde
            x = self.alpha * x_tilde + (1.0 - self.alpha) * x
            z_relaxed = self.alpha * z_tilde + (1.0 - self.alpha) * z
            z_next = np.clip(z_relaxed + y / rho_vec, l, u)
            y = y + rho_vec * (z_relaxed - z_next)
            z = z_next
            if iteration % 10:
                continue
            ax = A @ x
            primal = np.max(np.abs(ax - z), initial=0.0)
            dual_vec = P @ x + q + A.T @ y
            dual = np.max(np.abs(dual_vec), initial=0.0)
            primal_tol = self.eps * (1.0 + max(np.max(np.abs(ax), initial=0.0), np.max(np.abs(z), initial=0.0)))
            dual_tol = self.eps * (1.0 + max(np.max(np.abs(P @ x), initial=0.0), np.max(np.abs(q), initial=0.0)))
            if primal < primal_tol and dual < dual_tol:
                break
            # Re-balance rho as OSQP does when the residuals drift apart.
            if iteration % 50 == 0:
                scale = math.sqrt((primal / primal_tol + 1e-12) / (dual / dual_tol + 1e-12))
                if scale > 5.0 or scale < 0.2:
                    rho = float(np.clip(rho * scale, 1e-4, 1e4))
                    rho_vec, chol = factor(rho)
        self.iterations = iteration
        return x, z, y


class WholeBodyMPC:
    # Points on the arm that obstacles act on, and whether they count for
    # the robot's own body and the tower.
    POINTS = ("ur5eforearm_link", "ur5ewrist_1_link", "ur5ewrist_2_link", "ur5ewrist_3_link")

    def __init__(
        self,
        chain: Chain,
        horizon: int = 10,
        dt: float = 0.1,
        limits: Limits = None,
        sqp_iterations: int = 1,
    ):
        self.chain = chain
        self.N = horizon
        self.dt = dt
        self.limits = limits or Limits()
        self.sqp_iterations = sqp_iterations
        self.qp = AdmmQP()
        self.U = np.zeros((self.N, NU))
        self._warm = None
        # Input effort and change-of-input (jerk) weights.
        self.effort = np.array([0.02, 0.02, 0.02] + [0.01] * NA)
        self.smooth = np.array([0.3, 0.3, 0.3] + [0.2] * NA)
        # Body-fixed keep-outs in base_link.
        self.tower_xy = np.array([-0.15, 0.0])
        self.tower_z = (1.05, 1.70)
        self.tower_radius = 0.17
        self.body = Box(x=(-0.45, 0.45), y=(-0.38, 0.38), top=0.80)
        self.floor = 0.30
        self.keepout_weight = 4000.0
        self.last_cost = None
        self.last_qp_iterations = 0

    # ------------------------------------------------------------------ model
    def rollout(self, x0, U, sensitivities=True):
        """States x_1..x_N, and dx_k/dU (NX x N*NU) if asked."""
        N, dt = self.N, self.dt
        X = np.zeros((N + 1, NX))
        X[0] = x0
        S = np.zeros((N + 1, NX, N * NU)) if sensitivities else None
        for k in range(N):
            bx, by, yaw = X[k, :3]
            vx, vy, wz = U[k, :3]
            c, s = math.cos(yaw), math.sin(yaw)
            X[k + 1, 0] = bx + (c * vx - s * vy) * dt
            X[k + 1, 1] = by + (s * vx + c * vy) * dt
            X[k + 1, 2] = yaw + wz * dt
            X[k + 1, 3:] = X[k, 3:] + U[k, 3:] * dt
            if not sensitivities:
                continue
            S[k + 1] = S[k]
            # Yaw now moves the base along the current body velocity.
            S[k + 1, 0] += (-s * vx - c * vy) * dt * S[k, 2]
            S[k + 1, 1] += (c * vx - s * vy) * dt * S[k, 2]
            cols = slice(k * NU, (k + 1) * NU)
            S[k + 1, 0, cols][:2] += np.array([c, -s]) * dt
            S[k + 1, 1, cols][:2] += np.array([s, c]) * dt
            S[k + 1, 2, k * NU + 2] += dt
            S[k + 1, 3:, k * NU + 3:(k + 1) * NU] += np.eye(NA) * dt
        return X, S

    def _stage(self, x, task: Task, scale: float, with_jacobian=True):
        """Residuals of one predicted state and their d/dx (rows x NX)."""
        bx, by, yaw = x[:3]
        q = x[3:]
        (p_arm, r_arm, jp_arm, jw_arm), points = self.chain.evaluate(q, jacobians=with_jacobian)
        rz = _rz(yaw)
        base3 = np.array([bx, by, 0.0])
        residuals = []
        jacobians = []

        def world_point(p_local, jp_local):
            rotated = rz @ p_local
            if not with_jacobian:
                return base3 + rotated, None
            jac = np.zeros((3, NX))
            jac[0, 0] = 1.0
            jac[1, 1] = 1.0
            jac[:, 2] = [-rotated[1], rotated[0], 0.0]
            jac[:, 3:] = rz @ jp_local
            return base3 + rotated, jac

        def add(residual, weight, jacobian=None):
            w = math.sqrt(weight * scale)
            residuals.append(w * np.atleast_1d(residual))
            if with_jacobian:
                jacobians.append(w * np.atleast_2d(jacobian))

        def arm_rows(rows):
            """d/dx of a base_link quantity that only the arm moves."""
            if not with_jacobian:
                return None
            jac = np.zeros((rows.shape[0], NX))
            jac[:, 3:] = rows
            return jac

        hand, hand_jac = world_point(p_arm, jp_arm)
        if task.hand_position is not None and task.hand_weight > 0.0:
            add(hand - task.hand_position, task.hand_weight, hand_jac)
        if task.hand_rotation is not None and task.hand_rotation_weight > 0.0:
            rotation = rz @ r_arm
            error = rotation_residual(rotation, task.hand_rotation)
            jac = None
            if with_jacobian:
                angular = np.zeros((3, NX))
                angular[2, 2] = 1.0
                angular[:, 3:] = rz @ jw_arm
                jac = rotation_residual_jacobian(rotation) @ angular
            add(error, task.hand_rotation_weight, jac)
        if task.base_xy is not None and task.base_weight > 0.0:
            jac = np.zeros((2, NX))
            jac[0, 0] = jac[1, 1] = 1.0
            add(np.array([bx, by]) - task.base_xy, task.base_weight, jac)
        if task.base_yaw is not None and task.base_yaw_weight > 0.0:
            jac = np.zeros((1, NX))
            jac[0, 2] = 1.0
            add(_wrap(yaw - task.base_yaw), task.base_yaw_weight, jac)
        if task.posture is not None and task.posture_weight > 0.0:
            add(q - task.posture, task.posture_weight, arm_rows(np.eye(NA)))

        # Hinge keep-outs. Inactive ones contribute a zero row, so the
        # residual count stays fixed and the line search compares like
        # with like.
        everything = dict(points)
        everything["tcp"] = (p_arm, jp_arm)
        zero = np.zeros((1, NX))
        for name, (p_local, jp_local) in everything.items():
            # Body and floor, in base_link.
            inside_body = self.body.x[0] < p_local[0] < self.body.x[1] and self.body.y[0] < p_local[1] < self.body.y[1]
            floor = self.body.top if inside_body else self.floor
            amount = floor - p_local[2]
            if amount > 0.0:
                add(amount, self.keepout_weight, arm_rows(-jp_local[2:3]) if with_jacobian else None)
            else:
                add(0.0, self.keepout_weight, zero)
            # Pan-tilt tower: a vertical column behind the arm.
            if name != "tcp":
                amount = 0.0
                if self.tower_z[0] - 0.1 < p_local[2] < self.tower_z[1]:
                    offset = p_local[:2] - self.tower_xy
                    distance = float(np.linalg.norm(offset))
                    amount = self.tower_radius - distance
                if amount > 0.0:
                    jac = None
                    if with_jacobian:
                        normal = offset / max(distance, 1e-6)
                        jac = arm_rows(-(normal @ jp_local[:2])[None, :])
                    add(amount, self.keepout_weight, jac)
                else:
                    add(0.0, self.keepout_weight, zero)
            # Stand top, in odom.
            if task.stand_xy is not None and name in task.stand_clearance:
                p_world, jac_world = world_point(p_local, jp_local)
                margin = task.stand_half + 0.10
                dx, dy = p_world[:2] - task.stand_xy
                amount = 0.0
                if abs(dx) < margin and abs(dy) < margin:
                    amount = task.stand_top + task.stand_clearance[name] - p_world[2]
                if amount > 0.0:
                    add(amount, self.keepout_weight, -jac_world[2:3] if with_jacobian else None)
                else:
                    add(0.0, self.keepout_weight, zero)
        if not with_jacobian:
            return np.concatenate(residuals), None
        return np.concatenate(residuals), np.vstack(jacobians)

    # ------------------------------------------------------------------- cost
    def _input_residuals(self, U, u_prev):
        """Effort and smoothness rows, linear in U."""
        n = self.N * NU
        effort = np.sqrt(self.effort)
        smooth = np.sqrt(self.smooth)
        rows = [np.tile(effort, self.N) * U.ravel()]
        jac = [np.diag(np.tile(effort, self.N))]
        diff = np.vstack([U[:1] - u_prev, U[1:] - U[:-1]])
        rows.append(np.tile(smooth, self.N) * diff.ravel())
        d = np.eye(n)
        d[NU:, :-NU] -= np.eye(n - NU)
        jac.append(np.tile(smooth, self.N)[:, None] * d)
        return np.concatenate(rows), np.vstack(jac)

    def cost(self, x0, U, u_prev, task):
        X, _ = self.rollout(x0, U, sensitivities=False)
        total = 0.0
        for k in range(1, self.N + 1):
            scale = task.terminal_scale if k == self.N else 1.0
            r, _ = self._stage(X[k], task, scale, with_jacobian=False)
            total += float(r @ r)
        r, _ = self._input_residuals(U, u_prev)
        return total + float(r @ r)

    # ------------------------------------------------------------ constraints
    def _constraints(self, x0, U, X, S, u_prev, task):
        """Linearised l <= A dU <= u."""
        N, dt, n = self.N, self.dt, self.N * NU
        lim = self.limits
        speeds = self.speed_bounds(X, task)
        speed = speeds[0]
        accel = np.array(list(lim.base_accel) + [lim.joint_accel] * NA) * dt
        rows, lower, upper = [], [], []

        # Speed bounds.
        rows.append(np.eye(n))
        lower.append(-speeds.ravel() - U.ravel())
        upper.append(speeds.ravel() - U.ravel())
        # The diagonals of the vx-vy box too, so driving at 45 degrees is
        # not 41 % faster than straight.
        planar = np.minimum(speeds[:, 0], speeds[:, 1])
        for direction in ((1.0, 1.0), (1.0, -1.0)):
            d_row = np.zeros((N, n))
            for k in range(N):
                d_row[k, k * NU:k * NU + 2] = np.array(direction) / math.sqrt(2.0)
            value = d_row @ U.ravel()
            rows.append(d_row)
            lower.append(-planar - value)
            upper.append(planar - value)

        # Acceleration bounds between consecutive inputs, and from the one
        # being applied now. Kinodynamic: the plan can only change as fast
        # as the motors and the base can.
        d = np.eye(n)
        d[NU:, :-NU] -= np.eye(n - NU)
        diff = np.vstack([U[:1] - u_prev, U[1:] - U[:-1]]).ravel()
        # A previous input already outside the new speed bound must still
        # be able to come back inside, so the first step is never tighter
        # than that.
        first = np.maximum(np.abs(np.clip(u_prev, -speed, speed) - u_prev), accel)
        step = np.tile(accel, N)
        step[:NU] = first
        rows.append(d)
        lower.append(-step - diff)
        upper.append(step - diff)

        # Joint range on every predicted state.
        for k in range(1, N + 1):
            rows.append(S[k, 3:])
            lower.append(self.chain.lower + 0.05 - X[k, 3:])
            upper.append(self.chain.upper - 0.05 - X[k, 3:])

        # The base keeps its front away from the stand.
        if task.stand_xy is not None and task.base_clearance > 0.0:
            for k in range(1, N + 1):
                offset = X[k, :2] - task.stand_xy
                distance = float(np.linalg.norm(offset))
                normal = offset / max(distance, 1e-6)
                rows.append((normal @ S[k, :2])[None, :])
                lower.append(np.array([task.base_clearance - distance]))
                upper.append(np.array([np.inf]))
        A = np.vstack(rows)
        l = np.concatenate(lower)
        u = np.concatenate(upper)
        # ADMM wants finite bounds.
        return A, np.maximum(l, -1e6), np.minimum(u, 1e6)

    # ------------------------------------------------------------------ solve
    def solve(self, base_pose, q, u_prev, task: Task, shift=1, dt=None, iterations=None):
        """One receding-horizon step. Returns the first input and the plan.

        `shift` is how many steps of the previous plan have elapsed by the
        time this one applies. It only moves the warm start. `dt` sets the
        step length from now on, so a slow solve can plan further ahead
        with the same number of steps. `iterations` overrides
        sqp_iterations for this solve.
        """
        if dt is not None:
            self.dt = float(dt)
        x0 = np.concatenate([np.asarray(base_pose, dtype=float), np.asarray(q, dtype=float)])
        u_prev = np.asarray(u_prev, dtype=float)
        shift = int(min(max(shift, 0), self.N - 1))
        U = np.vstack([self.U[shift:], np.repeat(self.U[-1:], shift, axis=0)])
        for _ in range(self.sqp_iterations if iterations is None else int(iterations)):
            X, S = self.rollout(x0, U)
            residuals, jacobians = [], []
            for k in range(1, self.N + 1):
                scale = task.terminal_scale if k == self.N else 1.0
                r, jx = self._stage(X[k], task, scale)
                residuals.append(r)
                jacobians.append(jx @ S[k])
            r_in, j_in = self._input_residuals(U, u_prev)
            residuals.append(r_in)
            jacobians.append(j_in)
            r = np.concatenate(residuals)
            J = np.vstack(jacobians)
            hessian = J.T @ J + 1e-6 * np.eye(J.shape[1])
            gradient = J.T @ r
            A, l, u = self._constraints(x0, U, X, S, u_prev, task)
            step, _z, dual = self.qp.solve(hessian, gradient, A, l, u, warm=self._warm)
            self._warm = (np.zeros_like(step), None, dual)
            self.last_qp_iterations = self.qp.iterations
            # Backtracking on the true cost. Linear constraints stay
            # satisfied for any fraction of the step.
            base_cost = float(r @ r)
            accepted = False
            for fraction in (1.0, 0.5, 0.25):
                trial = U + fraction * step.reshape(self.N, NU)
                trial_cost = self.cost(x0, trial, u_prev, task)
                if trial_cost <= base_cost:
                    U = trial
                    base_cost = trial_cost
                    accepted = True
                    break
            if not accepted:
                U = U + 0.1 * step.reshape(self.N, NU)
            self.last_cost = base_cost
        # The rollout clips nothing, so clip once more for the motors.
        X, _ = self.rollout(x0, U, sensitivities=False)
        speeds = self.speed_bounds(X, task)
        U = np.clip(U, -speeds, speeds)
        U[0] = self.limit_input(U[0], u_prev, self.dt, task.base_speed_scale)
        self.U = U
        X, _ = self.rollout(x0, U, sensitivities=False)
        return U[0].copy(), X, U

    def speed_bounds(self, X, task):
        """|u_k| limits (N x NU). Step k drives slower with the distance X[k] has left to slow_xy.

        Only vx and vy slow down: the yaw still has to settle near the
        goal, and the arm would otherwise absorb it at full reach.
        """
        bounds = np.tile(self._speed(task.base_speed_scale), (self.N, 1))
        if task.slow_xy is not None and task.slow_distance > 0.0:
            distance = np.linalg.norm(X[: self.N, :2] - np.asarray(task.slow_xy), axis=1)
            scale = np.clip(distance / task.slow_distance, task.slow_min, 1.0)
            bounds[:, :2] *= scale[:, None]
        return bounds

    def _speed(self, scale=1.0):
        return np.array(
            [value * scale for value in self.limits.base_speed] + [self.limits.joint_speed] * NA
        )

    def limit_input(self, u, u_prev, dt=None, speed_scale=1.0):
        """Clip to speed and acceleration so a dt mismatch cannot jerk the base."""
        dt = self.dt if dt is None else float(dt)
        dt = max(dt, 1e-3)
        accel = np.array(list(self.limits.base_accel) + [self.limits.joint_accel] * NA) * dt
        speed = self._speed(speed_scale)
        u = np.clip(np.asarray(u, dtype=float), -speed, speed)
        planar = float(np.linalg.norm(u[:2]))
        if planar > min(speed[0], speed[1]):
            u[:2] *= min(speed[0], speed[1]) / planar
        return np.asarray(u_prev, dtype=float) + np.clip(u - u_prev, -accel, accel)

    def reset(self):
        self.U = np.zeros((self.N, NU))
        self._warm = None


def _solver_loop(conn, urdf_xml, root, tip, joints, points, limits):
    mpc = WholeBodyMPC(Chain(urdf_xml, root, tip, joints, points=points), limits=limits)
    conn.send(("ready", None))
    while True:
        request = conn.recv()
        if request is None:
            break
        request, task = request
        started = time.perf_counter()
        _u, X, U = mpc.solve(
            request.base,
            request.q,
            request.u_prev,
            task,
            shift=request.shift,
            dt=request.dt,
            iterations=request.iterations,
        )
        conn.send(("plan", (U, X, time.perf_counter() - started, mpc.last_qp_iterations)))


class SolverProcess:
    """WholeBodyMPC in its own process, so a slow solve never blocks /cmd_vel.

    One request at a time: submit(), then poll() until the plan is back.
    """

    def __init__(self, urdf_xml, root, tip, joints, points=(), limits=None, timeout=30.0):
        context = mp.get_context("spawn")
        self._conn, child = context.Pipe()
        self._process = context.Process(
            target=_solver_loop,
            args=(child, urdf_xml, root, tip, tuple(joints), tuple(points), limits or Limits()),
            daemon=True,
        )
        self._process.start()
        child.close()
        self.busy = False
        if not self._conn.poll(timeout) or self._conn.recv()[0] != "ready":
            self.close()
            raise RuntimeError("The whole-body solver process did not start")

    def submit(self, request, task):
        """Start solving `request` (a wbc_estimator.SolveRequest) for `task`."""
        if self.busy:
            raise RuntimeError("A solve is already running")
        self._conn.send((request, task))
        self.busy = True

    def poll(self):
        """(U, X, solve seconds, QP iterations) once the plan is back, else None."""
        if not self.busy or not self._conn.poll():
            if self.busy and not self._process.is_alive():
                raise RuntimeError("The whole-body solver process died")
            return None
        kind, payload = self._conn.recv()
        self.busy = False
        return payload if kind == "plan" else None

    def close(self):
        try:
            self._conn.send(None)
        except (BrokenPipeError, OSError):
            pass
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
