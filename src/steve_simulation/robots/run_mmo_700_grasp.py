#!/usr/bin/env python3
"""
MMO-700 Grasping Scene
======================
Launches the MMO-700 robot in MuJoCo with a Robotiq 2F-85 gripper.
The robot will:
  1. Drive toward the table.
  2. Perform Inverse Kinematics to reach the red box.
  3. Close the gripper to grasp the box.
  4. Lift the arm to pick up the box.
"""
import time
from pathlib import Path
import mujoco
from mujoco import viewer
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "mmo_700.xml"

# Wheel actuator indices
WHEEL_FL = 8
WHEEL_FR = 9
WHEEL_BL = 10
WHEEL_BR = 11

APPROACH_SPEED = 4.0
STOP_DISTANCE = 0.85  # Stop when robot is farther from the table to account for lidars/camera

def main() -> None:
    print(f"Loading MuJoCo model from: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data  = mujoco.MjData(model)

    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_box")
    pinch_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pinch")
    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
    
    ARM_JOINTS = ['ur5eshoulder_pan_joint','ur5eshoulder_lift_joint','ur5eelbow_joint',
                  'ur5ewrist_1_joint','ur5ewrist_2_joint','ur5ewrist_3_joint']
    jids    = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn) for jn in ARM_JOINTS]
    q_ids   = [model.jnt_qposadr[jid] for jid in jids]
    dof_ids = [model.jnt_dofadr[jid] for jid in jids]

    table_x = 2.0
    state = "DRIVING"
    ik_iterations = 0

    # Start with retracted arm configuration so it doesn't hit the table
    data.ctrl[0:6] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    
    with viewer.launch_passive(model, data) as v:
        v.cam.lookat[:]  = [1.5, 0.0, 0.8]
        v.cam.distance   = 3.0
        v.cam.elevation  = -20

        while v.is_running():
            mujoco.mj_step(model, data)

            if state == "DRIVING":
                robot_x = data.xpos[base_id][0]
                if robot_x >= (table_x - STOP_DISTANCE):
                    # Stop driving
                    data.ctrl[WHEEL_FL] = 0.0
                    data.ctrl[WHEEL_FR] = 0.0
                    data.ctrl[WHEEL_BL] = 0.0
                    data.ctrl[WHEEL_BR] = 0.0
                    state = "PREPARE_REACH"
                    print("\nRobot stopped. Preparing arm for reach...")
                    
                    # Move arm to initial good config for reaching
                    data.ctrl[0:6] = [0.0, -1.0, 1.0, -1.57, -1.57, 0.0]
                    # Open gripper
                    data.ctrl[gripper_act_id] = 0.0
                    ik_iterations = 0
            
            elif state == "PREPARE_REACH":
                ik_iterations += 1
                if ik_iterations > 1000:  # wait 2 seconds for arm to move to pre-reach pose
                    state = "REACHING"
                    print("Arm ready. Reaching for the box...")
                    ik_iterations = 0
            
            elif state == "REACHING":
                # Compute IK exactly once to find the target joints
                print("Computing IK target for reaching...")
                d_ik = mujoco.MjData(model)
                d_ik.qpos[:] = data.qpos[:]
                
                target_pos = data.xpos[box_id].copy()
                target_pos[2] = data.xpos[box_id][2]
                
                # Run IK loop in virtual data
                for _ in range(1000):
                    mujoco.mj_forward(model, d_ik)
                    pos_err = target_pos - d_ik.site_xpos[pinch_id]
                    if np.linalg.norm(pos_err) < 0.01:
                        break
                    
                    J = np.zeros((3, model.nv))
                    mujoco.mj_jacSite(model, d_ik, J, None, pinch_id)
                    J = J[:, dof_ids]
                    dq = J.T @ np.linalg.solve(J @ J.T + 0.1**2 * np.eye(3), pos_err)
                    
                    for i, (qi, jid) in enumerate(zip(q_ids, jids)):
                        lo, hi = model.jnt_range[jid]
                        d_ik.qpos[qi] = np.clip(d_ik.qpos[qi] + 0.1 * dq[i], lo, hi)
                
                target_q = [d_ik.qpos[qi] for qi in q_ids]
                start_q = [data.ctrl[i] for i in range(6)]
                state = "EXECUTE_REACH"
                ik_iterations = 0
            
            elif state == "EXECUTE_REACH":
                # Smoothly interpolate data.ctrl to target_q
                ik_iterations += 1
                progress = min(1.0, ik_iterations / 1000.0)
                for i in range(6):
                    data.ctrl[i] = start_q[i] + progress * (target_q[i] - start_q[i])
                
                if ik_iterations >= 1000:
                    state = "GRASPING"
                    print("Reached target. Grasping...")
                    ik_iterations = 0
            
            elif state == "GRASPING":
                # Close the gripper (max control input is 255 for 2f85 actuator)
                data.ctrl[gripper_act_id] = 255.0
                ik_iterations += 1
                
                if ik_iterations > 300:  # wait ~0.6 seconds for grasp
                    state = "COMPUTE_LIFT"
                    print("Grasped. Computing Lift IK...")
                    ik_iterations = 0
            
            elif state == "COMPUTE_LIFT":
                d_ik = mujoco.MjData(model)
                d_ik.qpos[:] = data.qpos[:]
                
                target_pos = data.site_xpos[pinch_id].copy()
                target_pos[2] += 0.15  # lift 15 cm
                
                for _ in range(1000):
                    mujoco.mj_forward(model, d_ik)
                    pos_err = target_pos - d_ik.site_xpos[pinch_id]
                    if np.linalg.norm(pos_err) < 0.01:
                        break
                    
                    J = np.zeros((3, model.nv))
                    mujoco.mj_jacSite(model, d_ik, J, None, pinch_id)
                    J = J[:, dof_ids]
                    dq = J.T @ np.linalg.solve(J @ J.T + 0.1**2 * np.eye(3), pos_err)
                    
                    for i, (qi, jid) in enumerate(zip(q_ids, jids)):
                        lo, hi = model.jnt_range[jid]
                        d_ik.qpos[qi] = np.clip(d_ik.qpos[qi] + 0.1 * dq[i], lo, hi)
                
                target_lift_q = [d_ik.qpos[qi] for qi in q_ids]
                start_lift_q = [data.ctrl[i] for i in range(6)]
                state = "EXECUTE_LIFT"
                ik_iterations = 0

            elif state == "EXECUTE_LIFT":
                ik_iterations += 1
                progress = min(1.0, ik_iterations / 1000.0)
                for i in range(6):
                    data.ctrl[i] = start_lift_q[i] + progress * (target_lift_q[i] - start_lift_q[i])
                
                if ik_iterations >= 1000:
                    state = "DONE"
                    print(f"Task completed. Final Box Z: {data.xpos[box_id][2]:.3f}")

            v.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    main()
