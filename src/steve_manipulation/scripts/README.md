# Whole-body controller

This note is for the optimization behind `wbc_pick.py`: what is minimized, which constraints are hard, and how model predictive control (MPC), sequential quadratic programming (SQP), and the quadratic-program solver fit together in this repository.

It assumes a robotics master's background: rigid-body kinematics, Jacobians, least squares, and the idea of a cost plus constraints. The algorithms are written out with the formulas this code actually evaluates.

How to launch the node is in the [package README](../README.md).

## Where each piece lives

| File | Role |
| --- | --- |
| `wbc_pick.py` | ROS node. Reads the cube, odometry, and joint state. Streams `/cmd_vel` and the arm trajectory. |
| `wbc_task.py` | Turns the current phase (approach, descend, grasp, lift, carry) into one `Task`: targets and weights. |
| `wbc_solver.py` | The optimal-control problem, the SQP loop, and the QP solver. |
| `wbc_kinematics.py` | Forward kinematics, geometric Jacobians, and the IK seed for the pregrasp. |
| `wbc_estimator.py` | Base extended Kalman filter, and the clock that says when a plan applies. |

One solve is one finite-horizon optimal-control problem. The node solves it again and again and applies only the beginning of each plan. That outer loop is MPC. Each solve is itself an SQP: a sequence of quadratic programs built from a linear model of the robot and a quadratic model of the cost.

## Decision variables

The robot state at a knot of the horizon stacks the planar base pose in `odom` and the six UR5e joint positions:

$$
x = \begin{bmatrix} b_x & b_y & \theta & q_1 & \cdots & q_6 \end{bmatrix}^{\mathsf T}
\in \mathbb{R}^{9}.
$$

The input stacks the body twist of the omnidirectional base, expressed in `base_link`, and the six joint speeds:

$$
u = \begin{bmatrix} v_x & v_y & \omega & \dot q_1 & \cdots & \dot q_6 \end{bmatrix}^{\mathsf T}
\in \mathbb{R}^{9}.
$$

A plan is the sequence $U = (u_0,\ldots,u_{N-1})$ with $N = 10$. That is the decision variable: 90 numbers. The predicted states $x_1,\ldots,x_N$ are determined by $x_0$ and $U$ through the model below, so they are not free variables.

The first input $u_0$ is what the robot is asked to do. The rest of $U$ is a forecast that makes the first input account for limits and the goal a few tenths of a second ahead.

## Prediction model

The base is a planar rigid body. Its body velocity rotates into `odom` by the current yaw. The arm joints are integrators. One step of length $\Delta t$ is forward Euler, implemented in `WholeBodyMPC.rollout`:

$$
\begin{aligned}
b_x^{+} &= b_x + \big(\cos\theta\, v_x - \sin\theta\, v_y\big)\, \Delta t, \\
b_y^{+} &= b_y + \big(\sin\theta\, v_x + \cos\theta\, v_y\big)\, \Delta t, \\
\theta^{+} &= \theta + \omega\, \Delta t, \\
q^{+} &= q + \dot q\, \Delta t.
\end{aligned}
$$

The hand pose in `odom` couples the two subsystems. If $p_{\mathrm{arm}}(q)$ and $R_{\mathrm{arm}}(q)$ are the tool pose in `base_link`,

$$
p_{\mathrm{hand}} = \begin{bmatrix} b_x \\ b_y \\ 0 \end{bmatrix} + R_z(\theta)\, p_{\mathrm{arm}}(q),
\qquad
R_{\mathrm{hand}} = R_z(\theta)\, R_{\mathrm{arm}}(q).
$$

A base motion moves the hand even when every joint speed is zero, and an arm motion moves the hand in a direction that depends on yaw. That product is the kinodynamic coupling: one cost on $p_{\mathrm{hand}}$ produces commands for both $u_{\mathrm{base}}$ and $\dot q$.

$\Delta t$ is 0.1 s when a solve returns quickly. When a solve is slow, the same ten steps are stretched, up to 0.3 s each, so the forecast still covers the time until the next plan arrives. See [Timing](#timing-the-plan-is-late).

## The optimal-control problem

At every solve the controller answers

$$
\begin{aligned}
\min_{U}\quad
& \sum_{k=1}^{N} \ell(x_k) \;+\; \sum_{k=0}^{N-1} \ell_u(u_k, u_{k-1}) \\
\text{s.t.}\quad
& x_{k+1} = f(x_k, u_k), \quad x_0 \text{ given}, \\
& u_k \in \mathcal{U}(x_k), \\
& |u_k - u_{k-1}| \le a\, \Delta t, \\
& q_{\min} \le q_k \le q_{\max}, \\
& \| p_k^{\mathrm{base}} - p_{\mathrm{stand}} \| \ge c_{\mathrm{base}}.
\end{aligned}
$$

$\ell$ is a sum of squared tracking and obstacle residuals. $\ell_u$ penalizes effort and the change in input. $\mathcal{U}(x_k)$ is the speed box, shrunk when the predicted base is close to the standoff. $u_{-1}$ is the input already being applied, so the new plan has to leave it within one acceleration step.

Arm collisions with the body, the floor, the pan-tilt tower, and the stand top are inside $\ell$, as hinge penalties. They pull the solution off the obstacle and they stay defined when the arm starts inside one, so a bad initial guess still produces a quadratic program the solver can accept.

## Least-squares cost

Every term is a residual vector $r$. The scalar stage cost is the squared Euclidean norm $\|r\|^2$, and the weights are folded into $r$ by multiplying with $\sqrt{w}$. A weight of 400 on a 1 cm hand error then contributes $400 \times (0.01)^2 = 0.04$ to the cost, on the same scale as a joint-posture error with a much smaller weight.

The last knot is multiplied by `terminal_scale` (3 during the approach and the carry, 1 during the descent). That asks the end of the horizon to be closer to the goal than the intermediate knots, which is the usual terminal cost of a short-horizon MPC.

### Tracking

Hand position, with weight $w_p$:

$$
r_p = \sqrt{w_p}\,\big(p_{\mathrm{hand}}(x) - p^{\star}\big).
$$

Hand orientation. Let $R$ be $R_{\mathrm{hand}}$ and $R^{\star}$ the desired rotation (fingers horizontal, tool $z$ pointing down). The code uses

$$
r_R = \frac{1}{\sqrt{2}}\, \mathrm{vec}(R - R^{\star}) \in \mathbb{R}^{9}.
$$

For a small rotation error with angle $\phi$, $\|R - R^{\star}\|_F = \sqrt{2}\,\phi$, so $\|r_R\| \approx \phi$ in radians. A half-turn makes this residual large. The compact orientation error $\tfrac12\sum_i c_i \times d_i$ is zero at a half-turn as well as at zero, and a least-squares cost on it can settle with the hand flipped. The matrix residual avoids that stationary point. It is used only in the MPC cost. The one-shot IK that seeds the pregrasp still uses the compact error, because that solver starts next to the solution.

Base position and yaw:

$$
r_b = \sqrt{w_b}\left(\begin{bmatrix}b_x\\ b_y\end{bmatrix} - p_b^{\star}\right),
\qquad
r_\theta = \sqrt{w_\theta}\; \mathrm{wrap}(\theta - \theta^{\star}),
$$

with $\mathrm{wrap}(\alpha) = \operatorname{atan2}(\sin\alpha, \cos\alpha)$.

Arm posture, used while the hand task is still faint and again while carrying:

$$
r_q = \sqrt{w_q}\,(q - q^{\star}).
$$

### Effort and smoothness

Both are linear in $U$. With diagonal weight matrices $W_u$ and $W_{\Delta u}$,

$$
r_u = W_u^{1/2} u_k,
\qquad
r_{\Delta u} = W_{\Delta u}^{1/2}\,(u_k - u_{k-1}).
$$

$W_u = \mathrm{diag}(0.02, 0.02, 0.02, 0.01,\ldots,0.01)$ on $(v_x,v_y,\omega,\dot q)$. $W_{\Delta u} = \mathrm{diag}(0.3, 0.3, 0.3, 0.2,\ldots,0.2)$. The second term is a penalty on input rate. Together with the hard acceleration constraint it keeps consecutive plans from jerking the base when a new solve replaces the old one.

### Hinge penalties

A hinge is $\max(0, s(x))$, where $s > 0$ means the point has entered a keep-out. The residual is $\sqrt{w_h}\, \max(0, s)$ with $w_h = 4000$. Inactive hinges contribute a zero residual of fixed length, so a line search always compares costs of the same dimension.

Checked points are the forearm, the three wrist links, and the tool center point. In `base_link`:

- Inside the body footprint $x \in [-0.45, 0.45]$, $y \in [-0.38, 0.38]$, the point must stay above the cabinet top at $0.80\,\mathrm{m}$. Outside that footprint the floor is $0.30\,\mathrm{m}$:

$$
s_{\mathrm{floor}} = z_{\mathrm{floor}}(p) - p_z.
$$

- The pan-tilt tower is a vertical cylinder of radius $0.17\,\mathrm{m}$ centered at $(-0.15, 0)$ in `base_link`, for $z$ in about $[0.95, 1.70]\,\mathrm{m}$:

$$
s_{\mathrm{tower}} = 0.17 - \| p_{xy} - c_{\mathrm{tower}} \|.
$$

- Above the stand, in `odom`, a link listed in the task's clearance map must stay above the stand top plus a margin (6 cm on the arm links, 10 cm on the tool during the approach). The tool is omitted from this map during the descent, because the hand is supposed to reach the cube:

$$
s_{\mathrm{stand}} = z_{\mathrm{stand}} + \delta_{\mathrm{link}} - p_z
\quad\text{when }p_{xy}\text{ lies over the stand}.
$$

## Hard constraints

These are enforced inside the quadratic program. A solution the QP returns satisfies them up to the linearization, and the final input is clipped to the speed and acceleration limits once more before it is stored.

### Speed

$$
|v_x| \le \bar v_x,\quad
|v_y| \le \bar v_y,\quad
|\omega| \le \bar\omega,\quad
|\dot q_i| \le \bar{\dot q}.
$$

Defaults in `Limits`: $0.35\,\mathrm{m/s}$, $0.35\,\mathrm{m/s}$, $0.4\,\mathrm{rad/s}$, and $0.8\,\mathrm{rad/s}$ per joint.

A square box on $(v_x, v_y)$ allows a diagonal command $\bar v_x$ and $\bar v_y$ at once, which is $\sqrt{2}$ times faster than driving along an axis (about 41 %). Two extra inequalities bound the components along the diagonals:

$$
\left| \frac{v_x + v_y}{\sqrt{2}} \right| \le \min(\bar v_x, \bar v_y),
\qquad
\left| \frac{v_x - v_y}{\sqrt{2}} \right| \le \min(\bar v_x, \bar v_y).
$$

Along a 45° heading the Euclidean speed is then limited to the same value as straight ahead.

Near the standoff the translational limits shrink with the distance the predicted base still has to go. If $d_k = \| p_k - p_{\mathrm{slow}} \|$ and $d_{\mathrm{slow}} = 0.6\,\mathrm{m}$,

$$
\bar v_{x,k} = \bar v_x \cdot \mathrm{clip}\!\left(\frac{d_k}{d_{\mathrm{slow}}},\, 0.15,\, 1\right),
$$

and the same factor multiplies $\bar v_y$. Yaw and the arm keep their full limits: the base still has to settle its heading, and the arm still has to finish the reach. During the descent an extra factor `base_speed_scale` $= 0.3$ multiplies the whole base speed limit, so the base only creeps while the hand goes down.

### Acceleration

Consecutive inputs, including the jump from the input applied right now, obey

$$
|u_k - u_{k-1}| \le a\, \Delta t,
$$

with $a = (0.6, 0.6, 1.0)$ on the base and $2.0\,\mathrm{rad/s}^2$ on each joint. If the previous input already lies outside a newly tightened speed bound, the first step is widened to the distance needed to come back inside, so that one constraint cannot make the quadratic program infeasible.

### Joint range

On every predicted knot, 0.05 rad inside the URDF limits:

$$
q_{\mathrm{lower}} + 0.05 \le q_k \le q_{\mathrm{upper}} - 0.05.
$$

### Base clearance

The base origin stays at least $c_{\mathrm{base}}$ from the stand center. With the body front at 0.34 m, the stand half-width 0.125 m, and a minimum gap of 0.05 m,

$$
c_{\mathrm{base}} = 0.34 + 0.125 + 0.05 = 0.515\,\mathrm{m}.
$$

The approach target itself is a little farther out: gap 0.10 m, so the standoff is 0.565 m in front of the stand center. The constraint is the safety floor; the cost is what parks the robot on the standoff.

## What each phase asks for

`PickPlan.task` fills the weights. Empty weights drop that residual. Numbers are from `wbc_task.py`.

| Phase | Base | Hand | Posture |
| --- | --- | --- | --- |
| Approach, far | drive to the standoff, weight 30, and match heading, weight 30 | off | unfold toward the pregrasp posture, weight 8 |
| Approach, within 0.9 m, fading in by 0.25 m | same, plus the braking speed law | pregrasp pose fades in, position weight up to 400, orientation up to 60 | posture weight fades from 8 down to 0.3 |
| Descend, grasp, lift | hold the standoff, weight 90, base speed scale 0.3 | a point sliding down or up the vertical line, weights 400 and 60 | posture weight 0.3 |
| Carry | return to the pose where the pick started | orientation off | fold to home, weight 8 |

The pregrasp posture is computed once, before the MPC starts. `Chain.solve_ik` runs damped least squares from a few seeds and keeps the solution closest to home with the elbow up. For a tip error $e \in \mathbb{R}^6$ and geometric Jacobian $J$,

$$
\Delta q = J^{\mathsf T}\big(J J^{\mathsf T} + \lambda I\big)^{-1} e,
\qquad \lambda = 10^{-3},
$$

with each component of $\Delta q$ clipped to 0.2 rad and $q$ clipped to the joint limits. After that, the MPC tracks the Cartesian pregrasp and that posture. It does not call IK again during the motion.

The descent reference is a constant-speed line from the pregrasp to the grasp at $0.05\,\mathrm{m/s}$, then from the grasp up by 0.15 m at $0.06\,\mathrm{m/s}$. The phase machine in `PickPlan.update` switches when the measured base and hand are inside the thresholds in `wbc_task.py` (a few centimeters and a few degrees). A miss at the cube raises the hand and starts another approach at the corrected cube position, up to three retries.

## Kinematics inside the cost

`Chain.evaluate` walks the URDF from `base_link` to the tool. For revolute joint $i$ with origin $o_i$ and unit axis $\omega_i$, both expressed in the root frame, the geometric Jacobian columns are

$$
J_{p,i} = \omega_i \times (p - o_i),
\qquad
J_{\omega,i} = \omega_i.
$$

The world-position Jacobian with respect to the full state $x$, used as $\partial r / \partial x$, follows by differentiating $p_{\mathrm{hand}}$:

$$
\frac{\partial p}{\partial b_x} = e_x,\quad
\frac{\partial p}{\partial b_y} = e_y,\quad
\frac{\partial p}{\partial \theta} = e_z \times \big(R_z(\theta)\, p_{\mathrm{arm}}\big),\quad
\frac{\partial p}{\partial q} = R_z(\theta)\, J_p.
$$

The orientation residual is differentiated the same way: yaw rate contributes $e_z$, and the arm contributes $R_z(\theta)\, J_\omega$.

## MPC

A single open-loop plan would be applied blindly and would drift. MPC solves the problem above from the current state, applies the input for a short time, measures the robot again, and solves once more. The horizon recedes: every plan looks $N$ steps ahead from *now*.

In textbooks the first input is applied for one $\Delta t$ and the solver runs at that same rate. Here a solve often takes longer than a control step, especially next to a busy Gazebo. The implementation splits the rates:

- The solver process computes a whole sequence $U$.
- `PlanStreamer` writes the input that the clock says is due, at $50\,\mathrm{Hz}$ (`EXEC_DT = 0.02`), to `/cmd_vel` and, a little more slowly, to the arm trajectory.
- A new solve may start every $0.1\,\mathrm{s}$, and it plans from the state predicted at the time its result will take over, not from the state at the moment the solve was requested.

That last point matters. If the plan were built from the pose at the start of a 0.4 s solve, the base would already have driven 0.4 s on the previous command, and the new plan would brake late and then jump. `request()` in `wbc_estimator.py` moves the initial condition forward by the expected solve time.

### Timing, the plan is late

Let $t$ be the time a solve is requested and $\tau$ the expected solve duration (a low-pass of the durations just measured, clamped to $[0.1, 2]\,\mathrm{s}$).

1. The initial base pose is the EKF state at $t + \tau$, propagated through the commands the streamer will send between $t$ and $t + \tau$.
2. The initial joint vector is the measured $q$, integrated with the joint speeds of those same future commands.
3. The step length grows with $\tau$ so six of the ten knots cover the next solve (`spare = 4` knots remain as margin):

$$
\Delta t = \mathrm{clip}\!\left(\frac{\tau}{6},\, 0.1,\, 0.3\right).
$$

4. The warm start is the previous $U$, shifted by the number of steps that will have elapsed by $t + \tau$. Inputs that have slid off the front of the horizon are replaced by copying the last input.
5. When $\tau \le 0.3\,\mathrm{s}$ the SQP runs one iteration. Above that it runs two. One iteration from a shifted warm start is the real-time-iteration scheme: the solver does not iterate the nonlinear problem to convergence on every cycle, because the next cycle will linearize again at a newer state.

The quadratic-program dual solution is kept and reused as the warm start of the next ADMM solve inside the same SQP.

## SQP

The problem is nonlinear because yaw rotates the base velocity and because the hand position and the obstacle hinges are nonlinear functions of $q$ and $\theta$. SQP replaces that problem, locally, by a quadratic program, steps toward its solution, and builds a new quadratic program there.

`WholeBodyMPC.solve` does the following.

**1. Roll out and differentiate.** From $x_0$ and the current guess $U$, integrate $f$ to get $x_1,\ldots,x_N$. Alongside the states, propagate the sensitivity

$$
S_k = \frac{\partial x_k}{\partial U} \in \mathbb{R}^{9 \times 90},
\qquad
S_0 = 0,
$$

by the chain rule through $f$. In blocks, if $A_k = \partial f/\partial x$ and $B_k = \partial f/\partial u$ at $(x_k, u_k)$,

$$
S_{k+1} = A_k S_k + B_k E_k,
$$

where $E_k$ writes $B_k$ into the columns that belong to $u_k$. For this model $A_k$ is identity except for the yaw column, which turns $(v_x, v_y)$ as $\theta$ changes, and $B_k = \Delta t$ times the yaw rotation on the base velocity and identity on $(\omega, \dot q)$. The code accumulates that product analytically in `rollout`. No finite differences.

**2. Stack the residuals.** At each knot, $r_k(x_k)$ is the tracking and hinge residual and $J_{x,k} = \partial r_k / \partial x$. The input residuals $r_u(U)$ are already linear. The residual of the whole horizon and its Jacobian with respect to $U$ are

$$
r(U) = \begin{bmatrix} r_1(x_1) \\ \vdots \\ r_N(x_N) \\ r_u(U) \end{bmatrix},
\qquad
J = \frac{\partial r}{\partial U}
= \begin{bmatrix} J_{x,1} S_1 \\ \vdots \\ J_{x,N} S_N \\ \partial r_u / \partial U \end{bmatrix}.
$$

**3. Gauss–Newton model.** The cost the node tracks is $\|r(U)\|^2$. Around the current guess, $r(U + \Delta) \approx r + J\Delta$, and the quadratic model used by the QP is

$$
m(\Delta)
= \frac12 \Delta^{\mathsf T} (J^{\mathsf T} J) \Delta
+ (J^{\mathsf T} r)^{\mathsf T} \Delta.
$$

$J^{\mathsf T} J$ is the Gauss–Newton approximation of the Hessian of $\tfrac12\|r\|^2$: it keeps the curvature that comes from $J$ and drops the second derivatives of $r$ itself. Those second-derivative terms are small when the residuals are small, and $J^{\mathsf T} J$ is positive semidefinite by construction, which a true Hessian of a nonlinear least-squares cost need not be. A ridge $10^{-6} I$ makes the quadratic program strictly convex:

$$
P = J^{\mathsf T} J + 10^{-6} I,
\qquad
g = J^{\mathsf T} r.
$$

The Newton step of $\|r\|^2$ and of $\tfrac12\|r\|^2$ is the same vector. The factor one half is the convention the QP solver uses.

**4. Linearize the constraints** in the step $\Delta$, at the current $(U, x)$. Speed and acceleration are already linear in $U$, so they are exact in $\Delta$:

$$
-u_{\max} - u_k \le \Delta_k \le u_{\max} - u_k.
$$

Joint limits and base clearance are linearized with the same sensitivities. For the joints,

$$
q_{\min} - q_k \le S_k^{\,q}\, \Delta \le q_{\max} - q_k.
$$

For the base, with unit vector $n_k$ pointing from the stand to the predicted base,

$$
n_k^{\mathsf T} S_k^{\,xy}\, \Delta \;\ge\; c_{\mathrm{base}} - \|p_k - p_{\mathrm{stand}}\|.
$$

Collected, the QP is

$$
\begin{aligned}
\min_{\Delta}\quad
& \tfrac12 \Delta^{\mathsf T} P \Delta + g^{\mathsf T} \Delta \\
\text{s.t.}\quad
& \ell \le A \Delta \le u.
\end{aligned}
$$

**5. Line search.** The QP step $\Delta$ is a proposal. The code tries fractions $1$, $\tfrac12$, and $\tfrac14$, rolls the nonlinear model out, and keeps the first fraction whose true cost $\|r\|^2$ does not increase. Any fraction of a feasible step stays feasible for these linearized inequalities, because they describe a convex set and the current guess is the origin $\Delta = 0$, which is feasible when the guess itself satisfies the linear constraints. If every fraction increases the cost, the step is still taken at fraction $0.1$, so a later iteration can move.

**6. Clip** the accepted $U$ to the speed bounds and the first input to the acceleration limit. That clip is what the motors see if the linearization and the nonlinear rollout disagree by a small amount.

Then `self.U` is stored for the next warm start, and $u_0$ together with the full sequences $X$ and $U$ go back to the node.

## The quadratic program (ADMM)

Each SQP iteration needs a solution of the inequality-constrained QP above. `AdmmQP` runs the operator-splitting iteration used by OSQP, on a dense system. There is no external QP library.

Split the constraint with an auxiliary variable $z = A\Delta$. The QP is then an unconstrained quadratic in $\Delta$, plus the indicator function of the box $\ell \le z \le u$, plus the equality $A\Delta - z = 0$. ADMM alternates a linear solve for $\Delta$ with a projection for $z$.

With penalty $\rho > 0$, proximal weight $\sigma = 10^{-6}$, dual variable $y$, and relaxation $\alpha = 1.6$:

$$
\begin{aligned}
\big(P + \sigma I + \rho A^{\mathsf T} A\big)\, \tilde\Delta
&= \sigma \Delta - g + A^{\mathsf T}(\rho z - y), \\[4pt]
z_{\mathrm{rel}}
&= \alpha A\tilde\Delta + (1 - \alpha)\, z, \\[4pt]
z^{+}
&= \Pi_{[\ell, u]}\!\left(z_{\mathrm{rel}} + y / \rho\right), \\[4pt]
y^{+}
&= y + \rho\,(z_{\mathrm{rel}} - z^{+}), \\[4pt]
\Delta^{+}
&= \alpha \tilde\Delta + (1 - \alpha)\, \Delta.
\end{aligned}
$$

$\Pi_{[\ell, u]}$ clips each component into its bound. That clip is the whole inequality handling: bounds never become active-set logic.

Equality rows (lower bound equal to upper bound) use a penalty $10^{3}\rho$ so they are satisfied tightly. Every 50 iterations, if the primal residual $\|A\Delta - z\|$ and the dual residual $\|P\Delta + g + A^{\mathsf T} y\|$ differ by a large factor, $\rho$ is rescaled and the KKT matrix is factored again. The iteration stops when both residuals are small relative to the size of the vectors involved, or at 400 iterations.

The matrix on the left is factorized once with a Cholesky decomposition and reused for every right-hand side at that $\rho$. The previous dual $y$ warm-starts the next SQP iteration, which is why one or two SQP passes are enough once the robot is moving smoothly.

## Base estimator

`BaseEKF` estimates

$$
\xi = \begin{bmatrix} p_x & p_y & \theta & v_x & v_y & \omega \end{bmatrix}^{\mathsf T},
$$

pose in `odom`, twist in `base_link`. The twist follows the commanded twist $u$ through a dead time $\delta = 0.01\,\mathrm{s}$ and a first-order lag $\tau = 0.02\,\mathrm{s}$, which matches Gazebo's planar-move plugin. Over a substep $h$,

$$
a = 1 - e^{-h/\tau},
\qquad
v^{+} = v + a\,(u(t - \delta) - v),
$$

and the pose integrates the *current* twist with the same yaw rotation as the MPC model. The discrete linearization $F = \partial \xi^{+}/\partial \xi$ of that step is the process Jacobian of the filter. Process noise grows with $h$: a few millimeters of pose drift and the base acceleration scales on the twist.

Measurements are pose and twist from `/odom`, applied at the stamp in the message. The update is the standard Kalman update with the Joseph form of the covariance,

$$
\begin{aligned}
K &= P H^{\mathsf T}(H P H^{\mathsf T} + R)^{-1}, \\
\xi &\leftarrow \xi + K(z - H\xi), \\
P &\leftarrow (I - KH)\, P\, (I - KH)^{\mathsf T} + K R K^{\mathsf T}.
\end{aligned}
$$

$H$ selects pose, or pose and twist. A position jump larger than 0.3 m resets the filter instead of being filtered in. Yaw innovations are wrapped.

The joints have no filter. Over the short gap between "solve requested" and "plan applies" they are open-loop integrated from the measured encoder values with the joint speeds the streamer is about to send.

## One control cycle

Putting the files in call order:

1. `wbc_pick.py` reads `/odom`, `/joint_states`, and `/cube_pose`.
2. `BaseEKF.update` fuses the odometry sample at its stamp.
3. `PickPlan.update` may change phase from the measured base and hand. `PickPlan.task` builds the `Task` for that phase.
4. If no solve is running, `PlanStreamer.request` builds a `SolveRequest`: predicted state at $t+\tau$, shifted warm-start index, $\Delta t$, and one or two SQP iterations.
5. `SolverProcess` runs `WholeBodyMPC.solve` in another process. Inside it: rollout, sensitivities, Gauss–Newton QP, ADMM, line search.
6. Meanwhile `PlanStreamer.step` keeps publishing the base part of the *previous* plan at 50 Hz, after the speed and acceleration clip. The arm is sent at 20 Hz.
7. When the new $U$ arrives, `receive` stamps it with the time it was supposed to start. From that time on, `step` reads the new sequence.

The arm command is the joint-speed part of the streamed input, integrated 0.2 s ahead of the measured joint position and sent as one `JointTrajectory` point so `joint_trajectory_controller` has a target to servo toward. The gripper is a separate action: it closes only after the grasp plugin reports the cube between the pads, which is the same check the fetch cycle uses.

## Defaults, collected

| Quantity | Value |
| --- | --- |
| Horizon | $N = 10$ |
| Step | $0.1$ to $0.3\,\mathrm{s}$, from the solve time |
| State / input | 9 / 9 |
| Base speed | $0.35$, $0.35\,\mathrm{m/s}$, $0.4\,\mathrm{rad/s}$ |
| Base acceleration | $0.6$, $0.6\,\mathrm{m/s}^2$, $1.0\,\mathrm{rad/s}^2$ |
| Joint speed / acceleration | $0.8\,\mathrm{rad/s}$, $2.0\,\mathrm{rad/s}^2$ |
| Hand / orientation / base / yaw / posture weights | 400 / 60 / 30 / 30 / 8 (0.3 while the hand task is on) |
| Hinge weight | 4000 |
| SQP iterations per solve | 1, or 2 if the solve is slower than $0.3\,\mathrm{s}$ |
| Stream rate | 50 Hz on `/cmd_vel` |
| ADMM | $\rho = 0.1$, $\alpha = 1.6$, at most 400 iterations |

## Further reading

The formulas above are the ones in `wbc_solver.py` and `wbc_estimator.py`. The surrounding methods, with the same notation, are standard:

- Rawlings, Mayne, Diehl, *Model Predictive Control: Theory, Computation, and Design*. Receding horizon, terminal cost, and the real-time-iteration idea (one SQP pass per cycle from a shifted warm start).
- Nocedal and Wright, *Numerical Optimization*, chapters on sequential quadratic programming and nonlinear least squares. Gauss–Newton is the least-squares chapter; the constrained step is the SQP chapter.
- Stellato, Banjac, Goulart, Bemporad, Boyd, *OSQP: an operator splitting solver for quadratic programs*, Mathematical Programming Computation, 2020. The iteration in `AdmmQP` is that splitting, including residual balancing of $\rho$ and the relaxation $\alpha$.
