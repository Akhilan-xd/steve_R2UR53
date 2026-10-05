# Steve

ROS 2 Humble workspace for **Steve**, an indoor mobile manipulator used to simulate mapping, localization, Nav2 navigation, and a perceived cube pick-and-place before running on hardware.

The robot is based on a **Neobotix MMO-700** omnidirectional Mecanum base with a **Universal Robots UR5e** arm. In this workspace the default simulation also mounts a Robotiq 2F-85 gripper, a wrist camera, and a pan-tilt camera tower.

## Status

| Package | In this repository | Notes |
| --- | --- | --- |
| `steve_simulation` | Yes | Gazebo Classic simulation, robot description, worlds, grasp plugin |
| `steve_navigation` | Yes | Mapping, AMCL localization, Nav2 |
| `steve_manipulation` | Yes | MoveIt 2, gripper, scripted pick, full fetch cycle |
| `steve_perception` | Yes | Red cube detection from the pan and wrist depth cameras |
| `steve_essentials` | **Not yet** | Neobotix drivers, SICK S300, relayboard, teleop, RealSense. To be added |
| `steve_hardware_bringup` | **Not yet** | Real-robot bringup (base, lidars, UR5e, pan-tilt). To be added |

Everything below runs in **simulation only**. Running on the real robot needs `steve_essentials` and `steve_hardware_bringup`, which are not pushed to this repository yet.

## Robot overview

Steve is a service-style mobile manipulator:

- **Base** — MMO-700 omnidirectional platform (Mecanum wheels) with simulated odometry and lidar
- **Arm** — UR5e on a cabinet, spawned in a compact home pose
- **End effector** — Robotiq 2F-85 gripper
- **Perception** — SICK lidar(s), Intel RealSense wrist D405, L515 on a pan-tilt tower, optional front depth camera

Default simulation arguments:

| Argument | Default | Role |
| --- | --- | --- |
| `my_robot` | `mmo_700` | Robot model to spawn |
| `world` | `small_house` | Gazebo world |
| `arm_type` | `ur5e` | Manipulator |
| `include_wrist_camera` | `true` | Wrist D405 |
| `include_pan_tilt` | `true` | Pan-tilt camera tower |
| `include_depth_camera` | `false` | Front depth camera |
| `map` | `~/steve_ws/maps/my_house.yaml` | Map for the map server |
| `launch_map_server` | `true` | Map server plus a static `map -> odom` transform |
| `use_rviz` | `true` | RViz window |
| `enable_teleop` | `false` | Keyboard teleop |

The robot spawns at world `(1.375, 1.56)` with yaw π, facing the pick stand one meter away. The pan-tilt head starts tilted down (0.41 rad) so the pan camera sees the cube from there.

Supported built-in worlds: `small_house`, `steve_house`, `neo_workshop`, `neo_track1`.

## Workspace layout

```text
steve_ws/
├── maps/                      # Local occupancy maps used for Nav2 (my_house)
├── src/
│   ├── steve_simulation/      # Gazebo simulation, robot description, worlds
│   ├── steve_navigation/      # Nav2 mapping, AMCL localization, path planning
│   ├── steve_manipulation/    # MoveIt 2 arm/gripper planning, pick and fetch
│   ├── steve_perception/      # Red cube pose from the pan and wrist cameras
│   ├── steve_essentials/      # (not yet in the repo) hardware drivers
│   └── steve_hardware_bringup/# (not yet in the repo) real-robot bringup
└── README.md
```

### `steve_simulation`

Gazebo Classic simulation package. It describes Steve, loads worlds and meshes, and starts the robot in simulation.

```text
src/steve_simulation/
├── launch/
│   ├── simulation.launch.py   # Main Gazebo + robot spawn entry point
│   └── mapping.launch.py      # Thin wrapper that starts Nav2 mapping
├── robots/mmo_700/            # URDF/xacro, meshes, and macros for Steve
├── components/                # Shared arm, camera, lidar, and pan-tilt xacro
├── worlds/                    # Gazebo worlds
├── models/                    # Gazebo models used by the house/workshop worlds
├── maps/                      # Occupancy maps matching the built-in worlds
├── configs/                   # Per-robot mapping/nav params and ros2_control
├── src/grasp_attach_plugin.cpp# Gazebo plugin that holds the cube in the gripper
└── rviz/                      # Simulation RViz config
```

What it does:

- Starts Gazebo and spawns the MMO-700
- Publishes `robot_description` and TF through `robot_state_publisher`
- Loads `ros2_control` for the UR5e, gripper, and pan-tilt joints
- Optionally starts a map server (with a static `map -> odom`), RViz, and keyboard teleop
- Holds the cube in the gripper through the grasp plugin (`/grasp_cube/...`), and publishes the cube position in the palm frame on `/grasp_cube/cube_in_palm`

### `steve_navigation`

Nav2 bringup package for mapping, localization, and autonomous driving.

```text
src/steve_navigation/
├── launch/
│   ├── mapping.launch.py                 # slam_toolbox mapping
│   ├── localization_amcl.launch.py       # map_server + AMCL
│   ├── navigation_neo.launch.py          # Nav2 planner, controller, BT
│   ├── localization_navigation.launch.py # AMCL + Nav2 (optional sim)
│   └── rviz_launch.py                    # Nav2 RViz
├── config/
│   ├── mapping.yaml                      # slam_toolbox params
│   ├── navigation_sim.yaml               # Nav2 params for simulation (used by default)
│   └── navigation.yaml                   # Alternate Nav2 params
├── rviz/                                 # RViz layouts
└── scripts/navigate_to_pose.py           # CLI client for NavigateToPose
```

What it does:

- Builds a map with **slam_toolbox** from `/lidar_1/scan`
- Localizes on a saved map with **AMCL** (default map: `~/steve_ws/maps/my_house.yaml`)
- Plans and follows paths with **Nav2** (`controller_server`, `planner_server`, `bt_navigator`, …)
- Sends a 2D pose goal from RViz or from `navigate_to_pose.py`

### `steve_manipulation`

MoveIt 2 bringup for the UR5e and Robotiq gripper, plus the pick and fetch scripts.

```text
src/steve_manipulation/
├── launch/
│   ├── manipulation.launch.py  # Optional sim + stand/cube + perception + move_group + RViz (+ fetch)
│   ├── move_group.launch.py    # MoveIt planning server
│   └── moveit_rviz.launch.py   # RViz with map, costmap, lidar, and both camera images
├── config/                     # SRDF, IK, OMPL, controller mapping, RViz layouts
├── models/                     # Pick stand and textured red cube
└── scripts/
    ├── pick_object.py          # Named poses and a scripted pick at a known pose
    ├── fetch_cube.py           # Full cycle: see, drive, grasp, carry, place on the table
    ├── publish_pick_scene.py   # Stand and cube collision objects for MoveIt
    └── gripper_command.py      # Open / close the 2F-85
```

What it does:

- Starts **move_group** against the same `robot_description` Gazebo uses
- Plans collision-aware UR5e trajectories and executes them on `joint_trajectory_controller`
- Opens and closes the gripper through `robotiq_gripper_controller`
- Spawns a stand and a 40 mm red cube one meter in front of the robot
- Named arm poses: `home` (parked beside the pan view), `ready`, `see`, `look`

### `steve_perception`

```text
src/steve_perception/
├── launch/cube_perception.launch.py
└── scripts/cube_perception.py
```

`cube_perception.py` finds the red cube in the pan (L515) or wrist (D405) color image, takes its range from the matching depth image, and publishes:

| Topic | Content |
| --- | --- |
| `/cube_pose` | Cube center in `map`, for the base |
| `/cube_pose_base` | Cube center in `base_link`, for the arm |
| `/cube_source` | `pan depth` or `wrist depth` |

`manipulation.launch.py` starts it automatically. To run it on its own:

```bash
ros2 launch steve_perception cube_perception.launch.py
```

## Build

MoveIt 2 is not bundled with the workspace. Install it once:

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

Then build:

```bash
source /opt/ros/humble/setup.bash
cd ~/steve_ws
colcon build --symlink-install --packages-select \
  steve_simulation steve_navigation steve_manipulation steve_perception
source install/setup.bash
```

Every new terminal below needs:

```bash
source /opt/ros/humble/setup.bash
source ~/steve_ws/install/setup.bash
cd ~/steve_ws
```

## Simulation only

```bash
ros2 launch steve_simulation simulation.launch.py
```

Useful arguments:

```bash
ros2 launch steve_simulation simulation.launch.py \
  world:=steve_house \
  arm_type:=ur5e \
  include_pan_tilt:=true \
  enable_teleop:=true \
  use_rviz:=true
```

## Navigation only

Navigation does not need MoveIt, the stand, or the cube.

### Mapping

Terminal 1, the robot without a map server:

```bash
ros2 launch steve_simulation simulation.launch.py launch_map_server:=false use_rviz:=false
```

Terminal 2, slam_toolbox:

```bash
ros2 launch steve_navigation mapping.launch.py use_sim_time:=true
```

Save the map when you are done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/steve_ws/maps/my_house
```

### Localization and navigation

One command starts Gazebo, AMCL, and Nav2 together:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Or in two terminals. Terminal 1:

```bash
ros2 launch steve_simulation simulation.launch.py launch_map_server:=false use_rviz:=false
```

Terminal 2:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  launch_simulation:=false \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Keep `launch_map_server:=false` on the simulation when Nav2 runs. With the map server on, the simulation publishes a static `map -> odom`, which fights the transform AMCL publishes.

Send a goal from RViz (**Nav2 Goal**) or from the command line:

```bash
ros2 run steve_navigation navigate_to_pose.py --x 1.0 --y 0.5 --yaw 0.0
```

Speed limits and costmap inflation are in `src/steve_navigation/config/navigation_sim.yaml` (`controller_server -> FollowPath` and `local_costmap`).

## Manipulation only: cube pick

This picks the cube with the arm while the base stays still. The base does not drive, so the stand has to be spawned inside the arm's reach instead of one meter away.

Terminal 1, Gazebo, the stand and cube 10 cm in front of the body, perception, MoveIt, and RViz:

```bash
ros2 launch steve_manipulation manipulation.launch.py cube_x:=0.81 cube_y:=1.56
```

Terminal 2, once MoveIt is up, run the scripted pick at the cube's pose in `base_link`. With the robot at `(1.375, 1.56)` and yaw π, world `(0.81, 1.56)` is `base_link (0.565, 0.0)`:

```bash
ros2 run steve_manipulation pick_object.py --pick --x 0.565 --y 0.0 --z 0.82
```

The pick goes to `ready`, opens the gripper, moves above the cube, descends straight down, closes, lifts, and returns to `ready`.

Single motions for testing the arm and gripper:

```bash
ros2 run steve_manipulation pick_object.py --named home
ros2 run steve_manipulation pick_object.py --named ready
ros2 run steve_manipulation pick_object.py --named look
ros2 run steve_manipulation gripper_command.py open
ros2 run steve_manipulation gripper_command.py close
```

If Gazebo is already running, add `launch_simulation:=false` to the launch command. In RViz you can also plan by hand: **MotionPlanning** panel, group `ur_manipulator`, pick a goal state, then **Plan** and **Execute**.

## Full fetch cycle

This is the whole task: see the cube, drive to it, grasp it, carry it, and place it on the kitchen table in `small_house`.

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

`fetch_cube.py` drives the base directly on `/cmd_vel`. Do not run Nav2 at the same time.

## Maintainer

This workspace is maintained by:

| | |
| --- | --- |
| **Name** | Akhilan Ashokan |
| **Email** | [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de) |
| **Affiliation** | Hochschule Bonn-Rhein-Sieg (H-BRS) |
| **GitHub** | [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53) |

The simulation and navigation packages started from Neobotix ROS 2 bringup (`neo_simulation2` / `neo_nav2_bringup`) by Pradheep Padmanabhan, with later simulation work by Shrikar Nakhye (ItsShriks) and contributors.

## License

- `steve_simulation` — see `src/steve_simulation/LICENSE`
- `steve_navigation` — Apache-2.0
- `steve_manipulation` — Apache-2.0
- `steve_perception` — Apache-2.0
