# steve_manipulation

MoveIt 2 bringup for **Steve**: plan collision-aware UR5e motions and open/close the Robotiq 2F-85 gripper.

This package is the arm/gripper counterpart of `steve_navigation`. Navigation drives the base with Nav2. Manipulation moves the arm with MoveIt.

## What MoveIt is doing

```text
RViz / pick_object.py  -->  move_group  -->  joint_trajectory_controller  -->  Gazebo UR5e
                         \-> GripperCommand  -->  robotiq_gripper_controller  -->  fingers
```

- **SRDF** (`config/mmo_700.srdf`) names the planning groups (`ur_manipulator`, `gripper`), named poses (`home`, `ready`), and which links may touch without counting as a collision.
- **Kinematics** (`config/kinematics.yaml`) is the IK solver (KDL).
- **OMPL** (`config/ompl_planning.yaml`) samples joint-space paths. Default planner is RRTConnect.
- **Controllers** (`config/moveit_controllers.yaml`) map those paths onto the controllers already started by `steve_simulation`.

## Prerequisite

MoveIt 2 is not bundled with the workspace. Install it once:

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

Then rebuild:

```bash
source /opt/ros/humble/setup.bash
cd ~/steve_ws
colcon build --symlink-install --packages-select steve_manipulation steve_simulation
source install/setup.bash
```

## Launch files

| Launch file | Purpose |
| --- | --- |
| `manipulation.launch.py` | Optional Gazebo, pick cube, `move_group`, MoveIt RViz |
| `move_group.launch.py` | Planning server only |
| `moveit_rviz.launch.py` | RViz with the MotionPlanning plugin |

## Try it

If simulation is already running:

```bash
ros2 launch steve_manipulation manipulation.launch.py \
  launch_simulation:=false \
  use_sim_time:=true
```

From scratch:

```bash
ros2 launch steve_manipulation manipulation.launch.py
```

In RViz:

1. Select the **MotionPlanning** panel
2. Planning Group: `ur_manipulator`
3. Goal State: `ready` (or drag the interactive marker)
4. **Plan** then **Execute**

From the command line:

```bash
ros2 run steve_manipulation pick_object.py --named ready
ros2 run steve_manipulation gripper_command.py open
ros2 run steve_manipulation gripper_command.py close
ros2 run steve_manipulation pick_object.py --pick
```

`--pick` uses a known pose for the red cube. The stand is spawned at world `(-0.20, 0.55)`, which is `base_link (0.20, -0.55)`: in line with the shoulder, on the arm's side, and far enough out that a top-down grasp keeps the elbow bent and `wrist_2` near 90°. Physics grasping in Gazebo Classic is sensitive; if the cube slips, first confirm the arm reaches the cube and the fingers close.

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)
