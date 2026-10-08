"""Closed-loop check of the whole-body pick on a kinematic model of Steve.

No Gazebo: the base follows its twist exactly, the arm follows the
streamed joints with a first-order lag like the Gazebo PID, and the grasp
plugin's palm reading is the true cube minus gripper_tcp.

  python3 src/steve_manipulation/test/test_wbc_pick.py        # verbose run
  python3 -m pytest src/steve_manipulation/test/test_wbc_pick.py
"""

import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from wbc_estimator import BaseEKF, PlanStreamer, SolveRequest, TimedPlan  # noqa: E402
from wbc_kinematics import Chain  # noqa: E402
from wbc_solver import Limits, SolverProcess, WholeBodyMPC  # noqa: E402
from wbc_task import ARM_JOINTS, HOME, KEEPOUT_LINKS, STAND_TOP, Phase, PickPlan  # noqa: E402

START = (1.375, 1.56, math.pi)
CUBE = (-0.20, 1.56, 0.825)
CONTROL_DT = 0.1
ARM_LAG = 0.08


def _urdf():
    try:
        import xacro
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        pytest.skip("xacro / ament_index_python not available")
    try:
        share = get_package_share_directory("steve_simulation")
    except Exception:
        pytest.skip("steve_simulation is not built")
    path = os.path.join(share, "robots", "mmo_700", "mmo_700.urdf.xacro")
    return xacro.process_file(
        path,
        mappings={
            "use_gazebo": "true",
            "arm_type": "ur5e",
            "arm_tool": "robotiq_gripper",
            "use_docking_adapter": "False",
            "include_wrist_camera": "true",
            "include_depth_camera": "false",
            "include_pan_tilt": "true",
        },
    ).toxml()


def run(cube_error=(0.0, 0.0, 0.0), verbose=False, max_time=90.0):
    chain = Chain(_urdf(), "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS)
    log = print if verbose else (lambda *_: None)
    true_cube = np.array(CUBE)
    plan = PickPlan(chain, START, true_cube + np.asarray(cube_error), log=log)
    mpc = WholeBodyMPC(chain, limits=Limits())
    base = np.array(START, dtype=float)
    q = HOME.copy()
    q_cmd = HOME.copy()
    u_prev = np.zeros(9)
    held = False
    grasp_started = None
    record = {
        "t": [], "base": [], "hand": [], "u": [], "phase": [], "min_stand_gap": [],
        "solve": [], "arm_over_stand": [], "tower": [],
    }
    now = 0.0
    while now < max_time:
        phase = plan.update(base, q, now)
        if phase in (Phase.DONE, Phase.FAILED):
            break
        if phase == Phase.AT_GRASP:
            hand_p, _ = plan.hand(base, q)
            offset = true_cube - hand_p
            if np.linalg.norm(offset) <= 0.02:
                plan.start_grasp(now)
                grasp_started = now
            else:
                plan.missed(offset, now, base, q)
        if plan.phase == Phase.GRASP and now - grasp_started > 2.0:
            held = True
            plan.start_lift(now, base, q)

        task = plan.task(base, now)
        started = time.perf_counter()
        u, _X, _U = mpc.solve(base, q, u_prev, task)
        u = mpc.limit_input(u, u_prev, CONTROL_DT, task.base_speed_scale)
        record["solve"].append(time.perf_counter() - started)
        u_prev = u

        # Plant.
        c, s = math.cos(base[2]), math.sin(base[2])
        base = base + np.array([c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]]) * CONTROL_DT
        q_cmd = q_cmd + u[3:] * CONTROL_DT
        q = q + (q_cmd - q) * min(1.0, CONTROL_DT / ARM_LAG)
        now += CONTROL_DT

        hand_p, _ = plan.hand(base, q)
        if held:
            true_cube = hand_p.copy()
        (_tip, points) = chain.evaluate(q)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        over = []
        for name in KEEPOUT_LINKS:
            p = np.array([base[0], base[1], 0.0]) + rz @ points[name][0]
            if np.all(np.abs(p[:2] - plan.stand_xy) < 0.125 + 0.02):
                over.append(p[2] - STAND_TOP)
        record["arm_over_stand"].append(min(over) if over else np.inf)
        record["tower"].append(
            min(
                np.linalg.norm(points[name][0][:2] - np.array([-0.15, 0.0]))
                for name in KEEPOUT_LINKS
                if points[name][0][2] > 1.05
            )
            if any(points[name][0][2] > 1.05 for name in KEEPOUT_LINKS)
            else np.inf
        )
        record["t"].append(now)
        record["base"].append(base.copy())
        record["hand"].append(hand_p.copy())
        record["u"].append(u.copy())
        record["phase"].append(plan.phase)
        record["min_stand_gap"].append(float(np.linalg.norm(base[:2] - plan.stand_xy)))
        if verbose and len(record["t"]) % 20 == 0:
            log(
                f"t={now:5.1f} {plan.phase.value:9s} base=({base[0]:.3f},{base[1]:.3f},{base[2]:.2f}) "
                f"hand=({hand_p[0]:.3f},{hand_p[1]:.3f},{hand_p[2]:.3f}) "
                f"u=({u[0]:.2f},{u[1]:.2f},{u[2]:.2f}) |dq|={np.max(np.abs(u[3:])):.2f} "
                f"solve={record['solve'][-1] * 1000:.0f} ms qp={mpc.last_qp_iterations}"
            )
    return plan, record, held


def _check(plan, record, held):
    assert plan.phase == Phase.DONE, plan.reason
    assert held
    # The base never comes closer to the stand than its front plus GAP_MIN.
    assert min(record["min_stand_gap"]) > 0.34 + 0.125 + 0.05 - 0.01
    # Arm links over the stand stay above its top.
    assert min(record["arm_over_stand"]) > 0.02
    assert min(record["tower"]) > 0.12
    # Base and arm move at the same time during the approach.
    approach = [i for i, phase in enumerate(record["phase"]) if phase == Phase.APPROACH]
    u = np.array(record["u"])[approach]
    together = np.sum((np.linalg.norm(u[:, :2], axis=1) > 0.05) & (np.max(np.abs(u[:, 3:]), axis=1) > 0.05))
    assert together > 20
    # The descent starts before the base has fully stopped.
    first_descent = next(i for i, phase in enumerate(record["phase"]) if phase == Phase.DESCEND)
    assert first_descent > 0
    # Acceleration limits on the commands (kinodynamic coupling).
    du = np.diff(np.array(record["u"]), axis=0) / CONTROL_DT
    assert np.max(np.abs(du[:, :2])) < 0.6 + 1e-6


def run_with_latency(latency, synchronous=True, max_time=40.0, seed=0):
    """Approach with a solve that takes `latency` seconds, as next to Gazebo.

    The base follows /cmd_vel like the planar-move plugin and keeps the
    last command until a new one arrives. Odometry comes at 100 Hz with
    millimetre noise. `synchronous` is the node's scheme: EKF, plans that
    start where the base will be, streamed at EXEC_DT. Without it, every
    plan starts from the pose measured when the solve began and its first
    input is held until the next plan, as the node did before.
    """
    rng = np.random.default_rng(seed)
    chain = Chain(_urdf(), "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS)
    plan = PickPlan(chain, START, CUBE, log=lambda *_: None)
    mpc = WholeBodyMPC(chain, limits=Limits())
    ekf = BaseEKF()
    stream = PlanStreamer(mpc.dt, mpc.limit_input, 9, CONTROL_DT, 2.0, steps=mpc.N)
    h, exec_dt = 0.01, 0.02
    base = np.array(START, dtype=float)
    q = HOME.copy()
    q_cmd = HOME.copy()
    u = np.zeros(9)
    in_flight = None
    last_exec = -math.inf
    record = {"t": [], "speed": [], "distance": [], "u": [], "phase": [], "base_miss": [], "joint_miss": []}
    predicted = []
    now = 0.0
    while now < max_time and plan.phase == Phase.APPROACH:
        while predicted and now >= predicted[0][0] - 1e-9:
            _when, base_then, q_then = predicted.pop(0)
            record["base_miss"].append(float(np.linalg.norm(base[:2] - base_then[:2])))
            record["joint_miss"].append(float(np.max(np.abs(q - q_then))))
        ekf.update(now, base + rng.normal(0.0, 0.001, 3), u[:3] + rng.normal(0.0, 0.005, 3))
        if in_flight is not None and now >= in_flight[0] - 1e-9:
            _done, U, scale, u_first = in_flight
            in_flight = None
            if synchronous:
                stream.receive(now, U, scale)
            else:
                u = mpc.limit_input(u_first, u, latency, scale)
                ekf.command(now, u[:3])
        if synchronous and now - last_exec >= exec_dt - 1e-9:
            last_exec = now
            u = stream.step(now, exec_dt)
            ekf.command(now, u[:3])
        if in_flight is None:
            plan.update(base, q, now)
            if synchronous:
                request = stream.request(now, ekf, q)
                predicted.append((request.applies, request.base.copy(), request.q.copy()))
                task = plan.task(request.base, request.applies)
                u_first, _X, U = mpc.solve(
                    request.base,
                    request.q,
                    request.u_prev,
                    task,
                    shift=request.shift,
                    dt=request.dt,
                    iterations=request.iterations,
                )
            else:
                task = plan.task(base, now)
                u_first, _X, U = mpc.solve(base, q, u, task)
            in_flight = (now + latency, U.copy(), task.base_speed_scale, u_first)

        c, s = math.cos(base[2]), math.sin(base[2])
        base = base + np.array([c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]]) * h
        q_cmd = q_cmd + u[3:] * h
        q = q + (q_cmd - q) * min(1.0, h / ARM_LAG)
        now += h
        record["t"].append(now)
        record["speed"].append(float(np.linalg.norm(u[:2])))
        record["distance"].append(plan.base_error(base)[0])
        record["u"].append(u.copy())
        record["phase"].append(plan.phase)
    return plan, {key: np.array(value) if key != "phase" else value for key, value in record.items()}


def _final_approach(record, start=0.5):
    """Base speed over the last `start` metres to the standoff, in order."""
    distance = record["distance"]
    first = int(np.argmax(distance < start))
    return distance[first:], record["speed"][first:]


def test_ekf_tracks_pose_and_twist():
    ekf = BaseEKF()
    rng = np.random.default_rng(1)
    pose = np.array([0.0, 0.0, 0.3])
    twist = np.array([0.3, -0.1, 0.2])
    h = 0.01
    for k in range(300):
        t = k * h
        ekf.command(t, twist)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        if k * h > ekf.delay:
            pose = pose + np.array([c * twist[0] - s * twist[1], s * twist[0] + c * twist[1], twist[2]]) * h
        ekf.update(t + h, pose + rng.normal(0.0, 0.003, 3), twist + rng.normal(0.0, 0.01, 3))
    estimate, rate = ekf.state_at(ekf.t)
    assert np.linalg.norm(estimate[:2] - pose[:2]) < 0.005
    assert abs(estimate[2] - pose[2]) < 0.01
    assert np.max(np.abs(rate - twist)) < 0.02


def test_ekf_predicts_through_the_planned_inputs():
    """A pose one second ahead, through a plan that brakes the base to rest."""
    ekf = BaseEKF(tau=0.05, delay=0.0)
    ekf.reset(0.0, (0.0, 0.0, 0.0), (0.3, 0.0, 0.0))
    ekf.command(0.0, (0.3, 0.0, 0.0))
    U = np.zeros((10, 9))
    U[:, 0] = np.linspace(0.3, 0.0, 10)
    plan = TimedPlan(U, 0.0, 0.1)
    pose, twist = ekf.state_at(1.0, future=plan.at)
    expected = float(np.sum(U[:, 0]) * 0.1)
    assert abs(pose[0] - expected) < 0.02
    assert abs(twist[0]) < 0.03
    # The filter itself does not move.
    assert ekf.t == 0.0 and ekf.x[0] == 0.0


def test_solver_process_matches_an_inline_solve():
    xml = _urdf()
    chain = Chain(xml, "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS)
    plan = PickPlan(chain, START, CUBE, log=lambda *_: None)
    task = plan.task(np.array(START), 0.0)
    request = SolveRequest(
        applies=0.0, base=np.array(START), q=HOME.copy(), u_prev=np.zeros(9), shift=1, dt=0.12, iterations=2
    )
    inline = WholeBodyMPC(chain, limits=Limits())
    _u, _X, expected = inline.solve(request.base, request.q, request.u_prev, task, dt=0.12, iterations=2)
    solver = SolverProcess(xml, "base_link", "gripper_tcp", ARM_JOINTS, points=KEEPOUT_LINKS)
    try:
        solver.submit(request, task)
        result = None
        end = time.monotonic() + 30.0
        while result is None and time.monotonic() < end:
            result = solver.poll()
            time.sleep(0.01)
    finally:
        solver.close()
    assert result is not None
    np.testing.assert_allclose(result[0], expected, atol=1e-9)


@pytest.mark.parametrize("latency", [0.1, 0.8, 1.6])
def test_approach_brakes_into_the_stand_with_a_slow_solver(latency):
    plan, record = run_with_latency(latency)
    assert plan.phase == Phase.DESCEND, plan.reason
    distance, speed = _final_approach(record)
    # Over the last half metre the base slows down, and arrives slowly.
    assert np.max(speed - np.minimum.accumulate(speed)) < 0.06
    for near, limit in ((0.3, 0.25), (0.1, 0.12)):
        assert speed[np.argmax(distance < near)] < limit
    # Each plan started where the base really was when it took over.
    assert max(record["base_miss"]) < 0.02
    # The commands stay inside the acceleration limit at the 20 ms executor tick.
    du = np.diff(record["u"][:, :2], axis=0)
    assert np.max(np.abs(du)) < 0.6 * 0.02 + 1e-6


def test_pick_with_exact_cube():
    _check(*run())


def test_pick_recovers_from_a_cube_estimate_error():
    plan, record, held = run(cube_error=(0.03, -0.025, 0.0))
    _check(plan, record, held)
    assert plan.retries >= 1


if __name__ == "__main__":
    for error in ((0.0, 0.0, 0.0), (0.03, -0.025, 0.0)):
        plan, record, held = run(cube_error=error, verbose=True)
        solve = np.array(record["solve"]) * 1000
        print(
            f"\nResult {plan.phase.value} ({plan.reason}), retries {plan.retries}, t={record['t'][-1]:.1f} s, "
            f"solve mean {solve.mean():.0f} ms, p95 {np.percentile(solve, 95):.0f} ms, max {solve.max():.0f} ms"
        )
        print(
            f"min base-stand {min(record['min_stand_gap']):.3f} m, "
            f"min arm over stand {min(record['arm_over_stand']):.3f} m, min tower {min(record['tower']):.3f} m\n"
        )
