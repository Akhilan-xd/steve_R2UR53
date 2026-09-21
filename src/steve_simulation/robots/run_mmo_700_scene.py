#!/usr/bin/env python3
"""
MMO-700 Table Approach Scene
==============================
Launches the MMO-700 robot in MuJoCo with:
  - A wooden table placed 2.0 m ahead of the robot
  - A red box sitting on the table
  - Wheel velocity actuators driving the robot forward toward the table

Controls (while viewer is open):
  Space  - pause / resume
  Esc    - quit
"""
import time
from pathlib import Path
import mujoco
from mujoco import viewer

SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "mmo_700.xml"

# Wheel actuator indices (0-indexed in actuator list)
WHEEL_FL = 8
WHEEL_FR = 9
WHEEL_BL = 10
WHEEL_BR = 11

# Approach speed (rad/s) – positive spins wheels forward (+x)
APPROACH_SPEED = 4.0

# Stop when robot front reaches this distance from table front edge
TABLE_X       = 2.0
STOP_DISTANCE = 0.65   # stop when robot x >= 1.35 m


def main() -> None:
    print(f"Loading MuJoCo model from: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data  = mujoco.MjData(model)

    # Load home keyframe (box on table, arm in home pose, wheels pre-set to approach speed)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        print("  Keyframe 'home' loaded.")
    else:
        mujoco.mj_resetData(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    print("\n=== Scene: MMO-700 approaching table with a red box ===")
    print(f"  Table at x = {TABLE_X:.1f} m   |  Robot starts at x = 0.0 m")
    print(f"  Wheels set to {APPROACH_SPEED} rad/s forward, stop at x ≈ {TABLE_X - STOP_DISTANCE:.2f} m")
    print("  Close the viewer window to exit.\n")

    approaching = True

    with viewer.launch_passive(model, data) as v:
        # Position camera: slightly elevated, angled view of the scene
        v.cam.lookat[:]  = [1.0, 0.0, 0.5]
        v.cam.distance   = 4.5
        v.cam.azimuth    = -135
        v.cam.elevation  = -20

        while v.is_running():
            step_start = time.time()

            # Live wheel control: drive forward until stop threshold
            mujoco.mj_kinematics(model, data)
            robot_x = data.xpos[base_id][0]

            if approaching:
                data.ctrl[WHEEL_FL] = APPROACH_SPEED
                data.ctrl[WHEEL_FR] = APPROACH_SPEED
                data.ctrl[WHEEL_BL] = APPROACH_SPEED
                data.ctrl[WHEEL_BR] = APPROACH_SPEED
                if robot_x >= TABLE_X - STOP_DISTANCE:
                    approaching = False
                    data.ctrl[WHEEL_FL] = 0.0
                    data.ctrl[WHEEL_FR] = 0.0
                    data.ctrl[WHEEL_BL] = 0.0
                    data.ctrl[WHEEL_BR] = 0.0
                    print(f"  → Robot reached table (x = {robot_x:.3f} m). Wheels stopped.")

            # Step simulation and sync viewer
            mujoco.mj_step(model, data)
            v.sync()

            # Real-time pacing
            elapsed = time.time() - step_start
            sleep_t = model.opt.timestep - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)


if __name__ == "__main__":
    main()
