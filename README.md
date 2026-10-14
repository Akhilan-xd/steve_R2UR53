# Steve

ROS 2 Humble workspace for **Steve**, an indoor mobile manipulator used to simulate mapping, localization, Nav2 navigation, and a perceived cube pick-and-place before running on hardware.

The robot is based on a **Neobotix MMO-700** omnidirectional Mecanum base with a **Universal Robots UR5e** arm. In this workspace the default simulation also mounts a Robotiq 2F-85 gripper, a wrist camera, and a pan-tilt camera tower.

## Status

| Package | In this repository | Notes |
| --- | --- | --- |
| `steve_simulation` | Yes | Gazebo Classic simulation, robot description, worlds, grasp plugin |
| `steve_navigation` | Yes | Mapping, AMCL localization, Nav2 |
| `steve_manipulation` | Yes | MoveIt 2, gripper, scripted pick, fetch cycle, whole-body pick |
| `steve_perception` | Yes | Red cube detection from the pan and wrist depth cameras |
| `steve_isaac` | Yes | Isaac Sim and cuRobo. Does not start or modify Gazebo |
| `steve_essentials` | **Not yet** | Neobotix drivers, SICK S300, relayboard, teleop, RealSense. To be added |
| `steve_hardware_bringup` | **Not yet** | Real-robot bringup (base, lidars, UR5e, pan-tilt). To be added |

Everything below runs in **simulation only**. Running on the real robot needs `steve_essentials` and `steve_hardware_bringup`, which are not pushed to this repository yet.

## Robot overview

Steve is a service-style mobile manipulator:

- **Base** — MMO-700 omnidirectional platform (Mecanum wheels) with simulated odometry and lidar
- **Arm** — UR5e on a cabinet, spawned in a compact home pose
- **End effector** — Robotiq 2F-85 gripper
- **Perception** — SICK lidar(s), Intel RealSense wrist D405, L515 on a pan-tilt tower, optional front depth camera

The robot spawns at world `(1.375, 1.56)` with yaw π, facing the pick stand one meter away. The pan-tilt head starts tilted down (0.41 rad) so the pan camera sees the cube from there.

Supported built-in worlds: `small_house`, `steve_house`, `neo_workshop`, `neo_track1`. Launch arguments and the simulation layout are in [src/steve_simulation/README.md](src/steve_simulation/README.md).

## Workspace layout

```text
steve_ws/
├── docker/                    # ROS dependency image and Compose helpers
├── docker-compose.yml         # One Compose profile per stack
├── maps/                      # Local occupancy maps used for Nav2 (my_house)
├── src/
│   ├── steve_simulation/      # Gazebo simulation, robot description, worlds
│   ├── steve_navigation/      # Nav2 mapping, AMCL localization, path planning
│   ├── steve_manipulation/    # MoveIt 2, pick, fetch, whole-body controller
│   ├── steve_perception/      # Red cube pose from the pan and wrist cameras
│   ├── steve_isaac/           # Isaac Sim scene, cuRobo planner, same ROS interfaces
│   ├── steve_essentials/      # (not yet in the repo) hardware drivers
│   └── steve_hardware_bringup/# (not yet in the repo) real-robot bringup
└── README.md
```

| Package | What to read |
| --- | --- |
| `steve_simulation` | [src/steve_simulation/README.md](src/steve_simulation/README.md) |
| `steve_navigation` | [src/steve_navigation/README.md](src/steve_navigation/README.md) |
| `steve_perception` | [src/steve_perception/README.md](src/steve_perception/README.md) |
| `steve_manipulation` | [src/steve_manipulation/README.md](src/steve_manipulation/README.md). The whole-body optimization is in [src/steve_manipulation/scripts/README.md](src/steve_manipulation/scripts/README.md) |
| `steve_isaac` | [src/steve_isaac/README.md](src/steve_isaac/README.md) |

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
  steve_simulation steve_navigation steve_manipulation steve_perception steve_isaac
source install/setup.bash
```

Every new terminal below needs:

```bash
source /opt/ros/humble/setup.bash
source ~/steve_ws/install/setup.bash
cd ~/steve_ws
```

## Run

Start Gazebo:

```bash
ros2 launch steve_simulation simulation.launch.py
```

Then use one stack at a time. Each package README has the full commands.

| Task | Command | Details |
| --- | --- | --- |
| Mapping, localization, Nav2 | `ros2 launch steve_navigation localization_navigation.launch.py use_sim_time:=true map:=$HOME/steve_ws/maps/my_house.yaml` | [navigation](src/steve_navigation/README.md) |
| Cube pose | started by the manipulation launch, or `ros2 launch steve_perception cube_perception.launch.py` | [perception](src/steve_perception/README.md) |
| Arm pick, base still | `ros2 launch steve_manipulation manipulation.launch.py cube_x:=0.81 cube_y:=1.56` | [manipulation](src/steve_manipulation/README.md) |
| See, drive, grasp, carry, place | `ros2 launch steve_manipulation manipulation.launch.py run_fetch:=true` | [manipulation](src/steve_manipulation/README.md) |
| Base and arm in one controller | `ros2 launch steve_manipulation manipulation.launch.py run_wbc:=true` | [manipulation](src/steve_manipulation/README.md), [WBC math](src/steve_manipulation/scripts/README.md) |
| Isaac Sim and cuRobo | `ros2 launch steve_isaac isaac_bringup.launch.py` | [isaac](src/steve_isaac/README.md) |

`fetch_cube.py` and `wbc_pick.py` both publish `/cmd_vel`. Leave Nav2 stopped while either of them runs. Run one simulator at a time: Gazebo and Isaac both publish `/clock`.



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
