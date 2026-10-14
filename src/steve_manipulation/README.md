# steve_manipulation

MoveIt 2 bringup for **Steve**: collision-aware UR5e motions, the Robotiq 2F-85 gripper, a scripted pick, the full fetch cycle, and a whole-body pick in which the base and the arm are one optimization.

Navigation drives the base with Nav2. The scripted pick and the fetch cycle move the arm with MoveIt and, in the fetch, drive the base from the camera measurement. The whole-body pick (`wbc_pick.py`) commands the base twist and the arm joint speeds from one receding-horizon problem. The formulation — model predictive control, sequential quadratic programming, and the constraints — is in [scripts/README.md](scripts/README.md).

The workspace overview is in the [repository README](../../README.md).

## Layout

```text
src/steve_manipulation/
├── launch/
│   ├── manipulation.launch.py  # Optional sim + stand/cube + perception + move_group + RViz (+ fetch/WBC)
│   ├── move_group.launch.py    # MoveIt planning server
│   └── moveit_rviz.launch.py   # RViz with map, costmap, lidar, and both camera images
├── config/                     # SRDF, IK, OMPL, controller mapping, RViz layouts
├── models/                     # Pick stand and textured red cube
└── scripts/
    ├── pick_object.py          # Named poses and a scripted pick at a known pose
    ├── fetch_cube.py           # Full cycle: see, drive, grasp, carry, place on the table
    ├── wbc_pick.py             # ROS node: whole-body pick
    ├── wbc_solver.py           # Receding-horizon SQP (base + arm)
    ├── wbc_estimator.py        # Base EKF and plan timing
    ├── wbc_task.py             # Approach, descend, grasp, lift, carry
    ├── wbc_kinematics.py       # URDF chain, Jacobians, IK
    ├── publish_pick_scene.py   # Stand and cube collision objects for MoveIt
    ├── gripper_command.py      # Open / close the 2F-85
    └── README.md               # WBC optimization, MPC, SQP, constraints
```

## What MoveIt is doing

```text
RViz / pick_object.py  -->  move_group  -->  joint_trajectory_controller  -->  Gazebo UR5e
                         \-> GripperCommand  -->  robotiq_gripper_controller  -->  fingers
```

- **SRDF** (`config/mmo_700.srdf`) names the planning groups (`ur_manipulator`, `gripper`), named poses (`home`, `ready`), and which links may touch without counting as a collision.
- **Kinematics** (`config/kinematics.yaml`) is the IK solver (KDL).
- **OMPL** (`config/ompl_planning.yaml`) samples joint-space paths. Default planner is RRTConnect.
- **Controllers** (`config/moveit_controllers.yaml`) map those paths onto the controllers already started by `steve_simulation`.

The whole-body controller does not call `move_group`. It integrates its own URDF chain (`wbc_kinematics.py`) and sends `/cmd_vel` plus a joint trajectory.

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
| `manipulation.launch.py` | Optional Gazebo, stand and cube, perception, `move_group`, RViz, and optionally fetch or WBC |
| `move_group.launch.py` | Planning server only |
| `moveit_rviz.launch.py` | RViz with the MotionPlanning plugin |

If Gazebo is already running, add `launch_simulation:=false`. In RViz you can plan by hand: **MotionPlanning** panel, group `ur_manipulator`, pick a goal state, then **Plan** and **Execute**.

Named arm poses used by the scripts: `home` (parked beside the pan view), `ready`, `see`, `look`.

## Arm pick, base still

This picks the cube with the arm while the base stays still. The stand has to be spawned inside the arm's reach.

Terminal 1, Gazebo, the stand and cube 10 cm in front of the body, perception, MoveIt, and RViz:

```bash
ros2 launch steve_manipulation manipulation.launch.py cube_x:=0.81 cube_y:=1.56
```

Terminal 2, once MoveIt is up. With the robot at `(1.375, 1.56)` and yaw π, world `(0.81, 1.56)` is `base_link (0.565, 0.0)`:

```bash
ros2 run steve_manipulation pick_object.py --pick --x 0.565 --y 0.0 --z 0.82
```

The pick goes to `ready`, opens the gripper, moves above the cube, descends straight down, closes, lifts, and returns to `ready`.

Single motions:

```bash
ros2 run steve_manipulation pick_object.py --named home
ros2 run steve_manipulation pick_object.py --named ready
ros2 run steve_manipulation pick_object.py --named look
ros2 run steve_manipulation gripper_command.py open
ros2 run steve_manipulation gripper_command.py close
```

## Full fetch cycle

See the cube, drive to it, grasp it, carry it, and place it on the kitchen table in `small_house`.

```bash
ros2 launch steve_manipulation manipulation.launch.py run_fetch:=true
```

This starts Gazebo, spawns the stand and cube one meter in front of the robot, starts perception and MoveIt, and launches `fetch_cube.py` 20 seconds later. `fetch_cube.py` then:

1. Folds the arm to `home`, beside the pan camera's view, and opens the gripper.
2. Waits until the pan or wrist depth camera publishes the cube pose.
3. Turns the base to face the cube and drives until the stand is 10 cm in front of the body. The pan-tilt head tracks the cube the whole way.
4. Re-measures the cube and tries grasps in order: top-down with the fingers across the robot, top-down along it, then a side grasp. If all fail, the base backs off to 0.80 m and tries again.
5. Closes only once the grasp plugin reports the cube between the pads, lifts, and folds home.
6. Drives back to the start pose, then to the kitchen table, lowers the cube onto it, and releases it.

The log ends with `TASK COMPLETED SUCCESSFULLY` or `TASK FAILED`.

If the cube is still on the stand (for example after a failed run), start the fetch again without restarting Gazebo:

```bash
ros2 run steve_manipulation fetch_cube.py
```

`fetch_cube.py` drives the base on `/cmd_vel`. Leave Nav2 stopped.

## Whole-body pick

The base and the arm are one optimal-control problem. While the base drives to the stand, the arm unfolds toward the pregrasp. When the base is close, the hand descends and the gripper closes on the red cube. The math is in [scripts/README.md](scripts/README.md).

```bash
ros2 launch steve_manipulation manipulation.launch.py run_wbc:=true
```

That starts the same Gazebo scene as the fetch cycle, then runs `wbc_pick.py` 20 seconds later. The node:

1. Reads the cube from the pan or wrist depth camera.
2. Solves a short-horizon problem over the omnidirectional base twist and the six UR5e joint speeds, with speed and acceleration limits on both. Near the stand the base speed limit shrinks with the distance left, so the base brakes into the standoff.
3. Fuses `/odom` and the sent twist in an EKF. The solve runs in its own process and can take longer than a control step, so each plan starts from the state the EKF predicts for the moment that plan takes over. Plans are streamed to `/cmd_vel` at 50 Hz and to the arm at 20 Hz in the meantime.
4. Starts the descent once the base is near the standoff and the hand is on the pregrasp, then welds and closes the gripper when the cube is between the pads.
5. Lifts and carries the cube back to the start pose.

Leave Nav2 and `fetch_cube.py` stopped: `wbc_pick.py` owns `/cmd_vel`. If Gazebo is already up with the stand and cube:

```bash
ros2 run steve_manipulation wbc_pick.py
```

The kinematic closed-loop check (no Gazebo) is:

```bash
python3 -m pytest src/steve_manipulation/test/test_wbc_pick.py
```

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)

## License

Apache-2.0
