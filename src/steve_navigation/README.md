# steve_navigation

Nav2 bringup package for **Steve**: mapping with slam_toolbox, AMCL localization, and autonomous navigation.

The workspace-level overview lives in the [repository README](../../README.md).

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)

This package started from Neobotix `neo_nav2_bringup` by Pradheep Padmanabhan.

## Launch files

| Launch file | Purpose |
| --- | --- |
| `mapping.launch.py` | slam_toolbox mapping (+ RViz) |
| `localization_amcl.launch.py` | map_server + AMCL |
| `navigation_neo.launch.py` | Nav2 planner, controller, and BT navigator |
| `localization_navigation.launch.py` | AMCL + Nav2, optionally starting Gazebo |

## Examples

Mapping while simulation is already running:

```bash
ros2 launch steve_navigation mapping.launch.py use_sim_time:=true
```

Localization and navigation on a saved map:

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  launch_simulation:=false \
  map:=$HOME/steve_ws/maps/my_house.yaml
```

Send a pose goal:

```bash
ros2 run steve_navigation navigate_to_pose.py --x 1.0 --y 0.5 --yaw 0.0
```
