# Steve

ROS 2 Humble workspace for **Steve**, an indoor mobile manipulator used to simulate mapping, localization, and Nav2 navigation before running on hardware.

The robot is based on a **Neobotix MMO-700** omnidirectional Mecanum base with a **Universal Robots UR5e** arm. In this workspace the default simulation also mounts a Robotiq gripper, a wrist camera, and a pan-tilt camera tower.

## Robot overview

Steve is a service-style mobile manipulator:

- **Base** — MMO-700 omnidirectional platform (Mecanum wheels) with simulated odometry and lidar
- **Arm** — UR5e on a cabinet, spawned in a compact home pose
- **End effector** — Robotiq gripper
- **Perception** — SICK lidar(s), optional Intel RealSense wrist D405, optional pan-tilt tower, optional front depth camera

Default simulation arguments:

| Argument | Default | Role |
| --- | --- | --- |
| `my_robot` | `mmo_700` | Robot model to spawn |
| `world` | `small_house` | Gazebo world |
| `arm_type` | `ur5e` | Manipulator |
| `include_wrist_camera` | `true` | Wrist D405 |
| `include_pan_tilt` | `true` | Pan-tilt camera tower |
| `include_depth_camera` | `false` | Front depth camera |

Supported built-in worlds: `small_house`, `steve_house`, `neo_workshop`, `neo_track1`.

## Workspace layout

```text
steve_ws/
├── maps/                      # Local occupancy maps used for Nav2 (e.g. my_house)
├── src/
│   ├── steve_simulation/      # Gazebo simulation, robot description, worlds
│   ├── steve_navigation/      # Nav2 mapping, AMCL localization, path planning
│   └── steve_manipulation/    # MoveIt 2 arm/gripper planning and pick helpers
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
└── rviz/                      # Simulation RViz config
```

What it does:

- Starts Gazebo and spawns the MMO-700
- Publishes `robot_description` and TF through `robot_state_publisher`
- Loads `ros2_control` for the UR5e and pan-tilt joints
- Optionally starts a map server, RViz, and keyboard teleop

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
│   ├── navigation_sim.yaml               # Nav2 params for simulation
│   └── navigation.yaml                   # Alternate Nav2 params
├── rviz/                                 # RViz layouts
└── scripts/navigate_to_pose.py           # CLI client for NavigateToPose
```

What it does:

- Builds a map with **slam_toolbox** from `/lidar_1/scan`
- Localizes on a saved map with **AMCL**
- Plans and follows paths with **Nav2** (`controller_server`, `planner_server`, `bt_navigator`, …)
- Sends a 2D pose goal from RViz or from `navigate_to_pose.py`

### `steve_manipulation`

MoveIt 2 bringup for the UR5e and Robotiq gripper.

```text
src/steve_manipulation/
├── launch/
│   ├── manipulation.launch.py  # Optional sim + cube + move_group + RViz
│   ├── move_group.launch.py    # MoveIt planning server
│   └── moveit_rviz.launch.py   # MotionPlanning RViz
├── config/                     # SRDF, IK, OMPL, controller mapping
├── models/                     # Pick stand and cube spawned for the demo
└── scripts/
    ├── pick_object.py          # Named poses and a scripted pick
    └── gripper_command.py      # Open / close the 2F-85
```

What it does:

- Starts **move_group** against the same `robot_description` Gazebo uses
- Plans collision-aware UR5e trajectories and executes them on `joint_trajectory_controller`
- Opens and closes the gripper through `robotiq_gripper_controller`
- Optionally spawns a stand and red cube in front of the robot for a first pick

Install MoveIt 2 once (`sudo apt install ros-humble-moveit`), then:

```bash
ros2 launch steve_manipulation manipulation.launch.py
```

If Gazebo is already running:

```bash
ros2 launch steve_manipulation manipulation.launch.py launch_simulation:=false
```

Move the arm:

```bash
ros2 run steve_manipulation pick_object.py --named ready
ros2 run steve_manipulation gripper_command.py open
ros2 run steve_manipulation pick_object.py --pick
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/steve_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick start

### Simulation only

```bash
ros2 launch steve_simulation simulation.launch.py
```

Useful arguments:

```bash
ros2 launch steve_simulation simulation.launch.py \
  world:=steve_house \
  arm_type:=ur5e \
  include_pan_tilt:=true \
  use_rviz:=true
```

### Mapping

In one terminal, start the robot without a map server:

```bash
ros2 launch steve_simulation simulation.launch.py launch_map_server:=false use_rviz:=false
```

In another terminal, start slam_toolbox:

```bash
ros2 launch steve_navigation mapping.launch.py use_sim_time:=true
```

Save the map when you are done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/steve_ws/maps/my_house
```

### Localization and navigation

If simulation is already running:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  launch_simulation:=false \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Or start simulation, AMCL, and Nav2 together:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Send a goal from the command line:

```bash
ros2 run steve_navigation navigate_to_pose.py --x 1.0 --y 0.5 --yaw 0.0
```

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
