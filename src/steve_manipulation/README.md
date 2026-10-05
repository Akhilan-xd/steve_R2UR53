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
ros2 launch steve_manipulation manipulation.launch.py run_fetch:=true
```

The robot spawns at `(1.375, 1.56)` with yaw π, one meter behind the stand at `(-0.20, 1.56)`, arm folded. The pan camera looks straight ahead and is pitched down onto that cube. One RViz window shows the map, the local costmap, both lidar scans, and the pan and wrist color images. The robot model stays out of that window. `fetch_cube.py` drives the base from the camera measurement until the body is 10 cm from the stand, then aims the wrist camera, grasps that measurement, and folds the arm to `home`.

Manual arm motions still work:

```bash
ros2 run steve_manipulation pick_object.py --named look
ros2 run steve_manipulation pick_object.py --named home
ros2 run steve_manipulation gripper_command.py open
ros2 run steve_manipulation gripper_command.py close
```

`--pick` still drives to the pose given by `--x/--y/--z`. The perceived grasp is `fetch_cube.py`.

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)
