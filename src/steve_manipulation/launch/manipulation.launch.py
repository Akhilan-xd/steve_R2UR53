import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manip_dir = get_package_share_directory("steve_manipulation")
    sim_dir = get_package_share_directory("steve_simulation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_simulation = LaunchConfiguration("launch_simulation")
    spawn_pick_scene = LaunchConfiguration("spawn_pick_scene")
    cube_x = LaunchConfiguration("cube_x")
    cube_y = LaunchConfiguration("cube_y")
    cube_z = LaunchConfiguration("cube_z")

    spawn_script = os.path.join(
        get_package_prefix("gazebo_ros"), "lib", "gazebo_ros", "spawn_entity.py"
    )
    stand_sdf = os.path.join(manip_dir, "models", "pick_stand", "model.sdf")
    cube_sdf = os.path.join(manip_dir, "models", "pick_cube", "model.sdf")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "simulation.launch.py")
        ),
        condition=IfCondition(launch_simulation),
        launch_arguments={
            "launch_map_server": "false",
            "use_rviz": "false",
            "enable_teleop": "false",
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manip_dir, "launch", "move_group.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manip_dir, "launch", "moveit_rviz.launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "spawn_pick_scene": spawn_pick_scene,
            "pick_scene_x": cube_x,
            "pick_scene_y": cube_y,
            "pick_scene_z": cube_z,
        }.items(),
    )

    spawn_stand = Node(
        package=None,
        executable="/usr/bin/python3",
        arguments=[
            spawn_script,
            "-entity",
            "pick_stand",
            "-file",
            stand_sdf,
            "-x",
            cube_x,
            "-y",
            cube_y,
            "-z",
            "0.0",
        ],
        output="screen",
        condition=IfCondition(spawn_pick_scene),
    )

    spawn_cube = Node(
        package=None,
        executable="/usr/bin/python3",
        arguments=[
            spawn_script,
            "-entity",
            "pick_cube",
            "-file",
            cube_sdf,
            "-x",
            cube_x,
            "-y",
            cube_y,
            "-z",
            cube_z,
        ],
        output="screen",
        condition=IfCondition(spawn_pick_scene),
    )

    # Gazebo needs a few seconds before spawn_entity will succeed when we
    # start simulation from this same launch file.
    delayed_scene = TimerAction(period=8.0, actions=[spawn_stand, spawn_cube])
    delayed_moveit = TimerAction(period=5.0, actions=[move_group, rviz])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from Gazebo",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Open MoveIt RViz with the MotionPlanning plugin",
            ),
            DeclareLaunchArgument(
                "launch_simulation",
                default_value="true",
                description="Start Gazebo (set false if simulation is already running)",
            ),
            DeclareLaunchArgument(
                "spawn_pick_scene",
                default_value="true",
                description="Spawn a stand and cube in front of the robot",
            ),
            DeclareLaunchArgument(
                "cube_x",
                default_value="-0.20",
                description="Cube Gazebo-world x. Robot yaw is pi, so this is base_link x=+0.20",
            ),
            DeclareLaunchArgument(
                "cube_y",
                default_value="0.55",
                description="Cube Gazebo-world y. Robot yaw is pi, so this is base_link y=-0.55",
            ),
            DeclareLaunchArgument(
                "cube_z",
                default_value="0.82",
                description="Cube world z (stand top is 0.80 m)",
            ),
            simulation,
            delayed_moveit,
            delayed_scene,
        ]
    )
