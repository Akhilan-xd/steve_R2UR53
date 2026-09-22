# steve_simulation

Gazebo Classic simulation package for **Steve**, an MMO-700 mobile manipulator with a UR5e arm.

This package provides robot descriptions, worlds, and launch files so navigation, manipulation, and perception can be tested before deploying to hardware.

The workspace-level overview lives in the [repository README](../../README.md).

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)

## How to run the simulation

### 1. Basic launch

```bash
ros2 launch steve_simulation simulation.launch.py
```

### 2. Custom robot and world

```bash
ros2 launch steve_simulation simulation.launch.py \
    world:=neo_workshop \
    arm_type:=ur5e \
    include_pan_tilt:=true
```

### 3. Mapping, localization, and Nav2

Use the launch files in `steve_navigation`.

**Mapping (SLAM):**

```bash
ros2 launch steve_simulation simulation.launch.py launch_map_server:=false use_rviz:=false
ros2 launch steve_navigation mapping.launch.py use_sim_time:=true
```

**Localization and navigation (AMCL + Nav2):**

```bash
ros2 launch steve_navigation localization_navigation.launch.py \
  use_sim_time:=true \
  map:=/path/to/your/map.yaml
```

## Troubleshooting

### Models not loading ("white box" robot)

1. Source the workspace: `source install/setup.bash`
2. Make sure `GAZEBO_MODEL_PATH` includes this package's models:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/steve_simulation/models
```

### "Missing model.config" errors

Gazebo often prints this when it tries the online model database. It is usually harmless if the local models are present.

### RealSense camera not publishing

The simulation uses a plugin for the wrist / depth cameras.

- Check topics: `ros2 topic list | grep camera`

## Acknowledgements

- **Rohit Menon** — mentorship and technical guidance on Neobotix platforms
- **Prof. Maren Bennewitz** — Head of the Humanoid Robots Lab, University of Bonn
- Original Neobotix simulation bringup by **Pradheep Padmanabhan**, with later work by **Shrikar Nakhye (ItsShriks)**
