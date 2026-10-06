# steve_hardware_bringup

**Hardware Initialization Package** for the Steve Butler robot.

This package manages the startup and coordination of all physical robot components. It relies on drivers provided by **`steve_essentials`**.

## Overview

The robot is designed for **hands-off startup**:
1. **Power On**: Turn the key switch to ON.
2. **Auto-Start**: The system automatically runs `ROS_AUTOSTART.sh` on boot.
3. **Initialization**:
   - **Base**: MMO-700 omnidirectional platform.
   - **Arm**: UR5e with Robotiq gripper.
   - **Sensors**: LiDARs, RealSense L515, IMU.
   - **Input**: Logitech joystick.

---

## Automatic Bringup (`ROS_AUTOSTART.sh`)

The robot is configured to automatically launch the hardware drivers safely inside a `screen` session on boot.

**To monitor the startup process:**
```bash
screen -r bringup
```

**To detach from the session:**
Press `Ctrl + A`, then `D`.

**Logs are available at:**
```bash
tail -f /home/neobotix/ros_logs/bringup.log
```

---

## Manual Launch (Development)

If you need to stop the automatic session and run drivers manually for debugging:

1. **Stop the auto-session:**
   ```bash
   screen -S bringup -X quit
   ```

2. **Launch drivers manually:**
   ```bash
   ros2 launch steve_hardware_bringup hardware_bringup.launch.py
   ```

**Optional Arguments:**
- `arm_type:=ur5e` (default: ur5e)
- `robot_ip:=192.168.1.102` (default: 192.168.1.102)
- `enable_base`, `enable_lidar`, `enable_teleop`, `enable_arm` (default: true)
- `enable_camera:=true` (default: true) - L515 on the pan-tilt tower
- `enable_wrist_camera:=false` (default: false) - D405; set `pan_tilt_camera_serial_no` and `wrist_camera_serial_no` when both cameras run
- `enable_pan_tilt:=false` (default: false) - needs `steve_pan_tilt_controller`, not in the workspace yet
- `enable_gripper:=false` (default: false) - UR tool communication only, no Robotiq driver yet

Joints without a running driver (pan, tilt, gripper) are held by `joint_state_publisher` and published with the arm on `/joint_states_complete`, which `robot_state_publisher` and MoveIt read.

---

## Hardware Dependencies

All hardware drivers are now consolidated in **`steve_essentials`**. This package (`steve_hardware_bringup`) orchestrates their launch.

- **Base Drivers**: `neo_relayboard_v2-2`, `neo_kinematics_omnidrive2`
- **Sensors**: `neo_sick_s300-2`, `realsense-ros`
- **Teleop**: `neo_teleop2`, `joy`

---

## Network Configuration

| Component | Value |
|---|---|
| Robot IP | `10.7.4.213` |
| Ethernet Port | `192.168.60.90` |
| Username | `neobotix` |

**SSH Access:**
```bash
ssh neobotix@10.7.4.213
```

For more details on network setup, see the main [workspace README](../../README.md).

---

### Acknowledgements
- **Rohit Menon** - For mentorship and technical guidance on Neobotix platforms.
- **Prof. Maren Bennewitz** - Head of the Humanoid Robots Lab, University of Bonn.