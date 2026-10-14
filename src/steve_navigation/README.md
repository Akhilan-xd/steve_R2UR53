# steve_navigation

Nav2 bringup for **Steve**: mapping with slam_toolbox, AMCL localization, and autonomous driving.

Navigation does not need MoveIt, the stand, or the cube. The workspace overview is in the [repository README](../../README.md).

## Layout

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
- Plans and follows paths with **Nav2** (`controller_server`, `planner_server`, `bt_navigator`, and the rest of the Nav2 stack)
- Sends a 2D pose goal from RViz or from `navigate_to_pose.py`

Speed limits and costmap inflation are in `config/navigation_sim.yaml` (`controller_server -> FollowPath` and `local_costmap`).

## Mapping

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

## Localization and navigation

One command starts Gazebo, AMCL, and Nav2 together:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Or in two terminals, if Gazebo is already running. Terminal 1:

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

Leave this stack stopped while `fetch_cube.py` or `wbc_pick.py` runs. Those nodes publish `/cmd_vel` themselves.

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)

This package started from Neobotix `neo_nav2_bringup` by Pradheep Padmanabhan.

## License

Apache-2.0
