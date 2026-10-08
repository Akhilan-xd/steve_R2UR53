"""Base state for the whole-body controller, without ROS.

The SQP solve takes longer than one control step, much longer next to a
busy Gazebo. If the solver plans from the pose measured when it starts,
the plan is late by a whole solve: the base keeps driving on the old
command meanwhile, brakes too late, and each new plan jumps the speed.

So the controller works on one clock:

BaseEKF     Pose and body twist of the base, fused from /odom and the
            twist the node commanded. It answers "where is the base now"
            and "where will it be at t", through the commands already
            sent and the ones the active plan will send.
TimedPlan   The inputs of one solve, stamped with the time the first one
            applies. The node streams it against the clock, so the base
            follows the previous plan's braking while the next is solved.

A solve that starts at `now` plans from the EKF state at
`now + expected solve time`, which is when its first input takes over.
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

NS = 6
SUBSTEP = 0.01


@dataclass
class SolveRequest:
    """Where a solve starts from, and how it should plan."""

    applies: float
    base: np.ndarray
    q: np.ndarray
    u_prev: np.ndarray
    shift: int
    dt: float
    iterations: int


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TimedPlan:
    """Inputs U[k] of one solve. U[k] applies on [t0 + k dt, t0 + (k+1) dt)."""

    def __init__(self, U, t0, dt):
        self.U = np.asarray(U, dtype=float)
        self.t0 = float(t0)
        self.dt = float(dt)

    @property
    def end(self):
        return self.t0 + self.U.shape[0] * self.dt

    def at(self, t):
        """Planned input at t. Before t0 the first input, past the end zero."""
        k = int(math.floor((t - self.t0) / self.dt + 1e-9))
        if k >= self.U.shape[0]:
            return np.zeros(self.U.shape[1])
        return self.U[max(k, 0)].copy()


class PlanStreamer:
    """Streams timed plans against the clock and times the next solve.

    step() gives the input to send now, rate limited by `limit(u, u_prev,
    dt, speed_scale)`. request() gives the state a new solve starts from:
    the EKF and the joints carried forward to when its plan will apply.
    receive() installs that plan, which takes over at its own t0.

    The step length grows with the solve time, so the plan in use always
    outlasts the next solve by `spare` of its `steps`: a slow solve plans
    further ahead instead of letting the base run off the end of a plan.
    Each plan also runs open loop for a whole solve, so past
    `refine_latency` it gets a second SQP iteration.
    """

    def __init__(
        self,
        dt,
        limit,
        nu,
        latency_min=0.1,
        latency_max=2.0,
        steps=10,
        spare=4,
        dt_max=0.3,
        exec_dt=0.02,
        refine_latency=0.3,
    ):
        self.exec_dt = float(exec_dt)
        self.refine_latency = float(refine_latency)
        self.dt_min = float(dt)
        self.dt_max = float(dt_max)
        self.dt = self.dt_min
        self.cover = max(1, int(steps) - int(spare))
        self.limit = limit
        self.latency_min = float(latency_min)
        self.latency_max = float(latency_max)
        self.latency = self.latency_min
        self.u = np.zeros(nu)
        self.active = None
        self.pending = None
        self.speed_scale = 1.0
        self.requested = None

    @property
    def waiting(self):
        return self.requested is not None

    def planned(self, t):
        plan = self.pending if self.pending is not None and t >= self.pending.t0 else self.active
        return plan.at(t) if plan is not None else np.zeros_like(self.u)

    def step(self, now, dt):
        if self.pending is not None and now >= self.pending.t0:
            self.active, self.pending = self.pending, None
        self.u = self.limit(self.planned(now), self.u, dt, self.speed_scale)
        return self.u.copy()

    def forecast(self, now, until):
        """What step() will send from now to `until`, as input_at(t)."""
        times = [now]
        inputs = [self.u.copy()]
        t, u = now, self.u.copy()
        while t < until:
            t += self.exec_dt
            u = self.limit(self.planned(t), u, self.exec_dt, self.speed_scale)
            times.append(t)
            inputs.append(u)
        times = np.array(times)

        def input_at(t):
            return inputs[min(int(np.searchsorted(times, t, side="right")) - 1, len(inputs) - 1)]

        return input_at

    def request(self, now, ekf, q):
        """The SolveRequest for a solve started now."""
        applies = now + self.latency
        sent = self.forecast(now, applies)
        base, _twist = ekf.state_at(applies, future=sent)
        latest = self.pending or self.active
        self.dt = min(max(self.latency / self.cover, self.dt_min), self.dt_max)
        self.requested = (now, applies, self.dt)
        return SolveRequest(
            applies=applies,
            base=base,
            q=joints_at(q, sent, now, applies),
            u_prev=sent(applies),
            shift=int(round((applies - latest.t0) / latest.dt)) if latest is not None else 1,
            dt=self.dt,
            iterations=1 if self.latency <= self.refine_latency else 2,
        )

    def receive(self, now, U, speed_scale=1.0):
        """Install the plan of the last request. Returns (solve latency, how late it is)."""
        requested, applies, dt = self.requested
        self.requested = None
        took = now - requested
        latency = max(took, 0.7 * self.latency + 0.3 * took)
        self.latency = min(max(latency, self.latency_min), self.latency_max)
        self.pending = TimedPlan(U, applies, dt)
        self.speed_scale = float(speed_scale)
        return took, now - applies


def joints_at(q, input_at, t_from, t_to):
    """Joints at t_to if they follow the speeds input_at(t)[3:] from q at t_from."""
    q = np.array(q, dtype=float)
    t = t_from
    while t < t_to - 1e-9:
        h = min(SUBSTEP, t_to - t)
        q = q + np.asarray(input_at(t + 0.5 * h))[3:] * h
        t += h
    return q


class BaseEKF:
    """Planar EKF. State [px, py, yaw, vx, vy, wz]: pose in odom, twist in base_link.

    Process: the body twist follows the command with a first-order lag
    `tau` after a dead time `delay`, and the pose integrates the twist
    through yaw. The defaults fit Gazebo's planar-move plugin, which sets
    the twist on its next update; a real base needs its own measured lag.
    Measurements: pose and twist from /odom, or a pose alone
    (TF) when odometry is missing. Every measurement carries its stamp,
    so a late message is fused at the time it was taken, not when it
    arrived.
    """

    def __init__(
        self,
        tau=0.02,
        delay=0.01,
        accel_std=(0.6, 0.6, 1.0),
        pose_drift_std=(0.002, 0.002, 0.003),
        pose_std=(0.005, 0.005, 0.005),
        twist_std=(0.02, 0.02, 0.03),
        jump=0.3,
    ):
        self.tau = float(tau)
        self.delay = float(delay)
        self.accel_var = np.square(accel_std)
        self.drift_var = np.square(pose_drift_std)
        self.pose_var = np.square(pose_std)
        self.twist_var = np.square(twist_std)
        self.jump = float(jump)
        self.x = None
        self.P = None
        self.t = None
        self.commands = deque()

    @property
    def ready(self):
        return self.x is not None

    def reset(self, stamp, pose, twist=None):
        self.x = np.zeros(NS)
        self.x[:3] = pose
        if twist is not None:
            self.x[3:] = twist
        self.x[2] = _wrap(self.x[2])
        self.P = np.diag(np.concatenate([self.pose_var, self.twist_var if twist is not None else [0.1, 0.1, 0.1]]))
        self.t = float(stamp)

    # ------------------------------------------------------------ inputs
    def command(self, stamp, twist):
        """The node sent `twist` (vx, vy, wz in base_link) at `stamp`."""
        stamp = float(stamp)
        twist = np.asarray(twist, dtype=float)[:3].copy()
        while self.commands and self.commands[-1][0] >= stamp:
            self.commands.pop()
        self.commands.append((stamp, twist))
        if self.t is not None:
            horizon = self.t - self.delay - 1.0
            while len(self.commands) > 1 and self.commands[1][0] <= horizon:
                self.commands.popleft()

    def _command_at(self, t, future=None):
        """Twist acting on the base at t: what was sent `delay` earlier."""
        sent = t - self.delay
        if future is not None and (not self.commands or sent > self.commands[-1][0]):
            return np.asarray(future(sent), dtype=float)[:3]
        active = np.zeros(3)
        for stamp, twist in self.commands:
            if stamp > sent:
                break
            active = twist
        return active

    # ------------------------------------------------------------ model
    def _step(self, x, P, h, u):
        px, py, yaw, vx, vy, wz = x
        c, s = math.cos(yaw), math.sin(yaw)
        a = 1.0 - math.exp(-h / self.tau) if self.tau > 1e-6 else 1.0
        x_next = np.array(
            [
                px + (c * vx - s * vy) * h,
                py + (s * vx + c * vy) * h,
                _wrap(yaw + wz * h),
                vx + (u[0] - vx) * a,
                vy + (u[1] - vy) * a,
                wz + (u[2] - wz) * a,
            ]
        )
        if P is None:
            return x_next, None
        F = np.eye(NS)
        F[0, 2] = (-s * vx - c * vy) * h
        F[1, 2] = (c * vx - s * vy) * h
        F[0, 3], F[0, 4] = c * h, -s * h
        F[1, 3], F[1, 4] = s * h, c * h
        F[2, 5] = h
        F[3, 3] = F[4, 4] = F[5, 5] = 1.0 - a
        Q = np.diag(np.concatenate([self.drift_var * h, self.accel_var * h]))
        return x_next, F @ P @ F.T + Q

    def _propagate(self, x, P, t_from, t_to, future=None):
        t = t_from
        while t < t_to - 1e-9:
            h = min(SUBSTEP, t_to - t)
            x, P = self._step(x, P, h, self._command_at(t + 0.5 * h, future))
            t += h
        return x, P

    def predict(self, stamp):
        if self.x is None or stamp <= self.t:
            return
        self.x, self.P = self._propagate(self.x, self.P, self.t, float(stamp))
        self.t = float(stamp)

    # ------------------------------------------------------- measurement
    def update(self, stamp, pose, twist=None):
        """Fuse a pose (and twist) measured at `stamp`. Returns False if it was dropped."""
        stamp = float(stamp)
        pose = np.asarray(pose, dtype=float)
        if self.x is None:
            self.reset(stamp, pose, twist)
            return True
        if stamp < self.t - 1e-6:
            return False
        self.predict(stamp)
        if np.linalg.norm(pose[:2] - self.x[:2]) > self.jump:
            self.reset(stamp, pose, twist)
            return True
        rows = [0, 1, 2] + ([3, 4, 5] if twist is not None else [])
        z = pose if twist is None else np.concatenate([pose, np.asarray(twist, dtype=float)])
        R = np.diag(self.pose_var if twist is None else np.concatenate([self.pose_var, self.twist_var]))
        H = np.eye(NS)[rows]
        innovation = z - self.x[rows]
        innovation[2] = _wrap(innovation[2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2] = _wrap(self.x[2])
        I_KH = np.eye(NS) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True

    # ------------------------------------------------------------ query
    def state_at(self, t, future=None):
        """Pose and twist at t, through the sent commands and then `future(t)`."""
        if self.x is None:
            return None, None
        if t <= self.t:
            return self.x[:3].copy(), self.x[3:].copy()
        x, _P = self._propagate(self.x.copy(), None, self.t, float(t), future)
        return x[:3], x[3:]
