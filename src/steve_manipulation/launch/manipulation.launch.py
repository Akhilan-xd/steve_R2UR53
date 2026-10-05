import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    run_fetch = LaunchConfiguration("run_fetch")

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
            "launch_map_server": "true",
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
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(manip_dir, "config", "scene.rviz"),
            "spawn_pick_scene": spawn_pick_scene,
            "pick_scene_x": cube_x,
            "pick_scene_y": cube_y,
            "pick_scene_z": cube_z,
        }.items(),
    )

    # Read use_rviz before the simulation include. That include sets
    # use_rviz to false for Gazebo's own window, and the value stays in
    # this launch context afterwards. A condition inside TimerAction is
    # also dropped, so the decision has to be made here, up front.
    def start_moveit(context):
        actions = [move_group]
        if IfCondition(use_rviz).evaluate(context):
            actions.append(rviz)
        return [TimerAction(period=5.0, actions=actions)]

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
    # start simulation from this same launch file. The cube spawns only once
    # the stand exists. Spawned first, it falls to the floor and the stand
    # appears around it.
    delayed_scene = TimerAction(period=8.0, actions=[spawn_stand])
    cube_after_stand = RegisterEventHandler(
        OnProcessExit(target_action=spawn_stand, on_exit=[spawn_cube])
    )

    perception = Node(
        package="steve_perception",
        executable="cube_perception.py",
        name="cube_perception",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    fetch = Node(
        package="steve_manipulation",
        executable="fetch_cube.py",
        name="fetch_cube",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    # A condition on TimerAction is ignored by this launch setup, same as
    # the MoveIt timer above. Decide here, then start the timer.
    def start_fetch(context):
        if not IfCondition(run_fetch).evaluate(context):
            return []
        return [TimerAction(period=20.0, actions=[fetch])]

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
                description="Open one RViz window with the map, local costmap, lidar, and both cameras",
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
                description="Cube Gazebo-world x. The robot spawns at x=1.375 facing this stand",
            ),
            DeclareLaunchArgument(
                "cube_y",
                default_value="1.56",
                description="Cube Gazebo-world y. Same y as the robot, so the level pan camera looks straight at the stand",
            ),
            DeclareLaunchArgument(
                "run_fetch",
                default_value="false",
                description="Pan or wrist depth starts the base. MoveIt solves the grasp while the base closes in",
            ),
            DeclareLaunchArgument(
                "cube_z",
                default_value="0.83",
                description="Cube center z. Stand top is 0.80 m, so 0.83 sits the 40 mm cube on it",
            ),
            OpaqueFunction(function=start_moveit),
            simulation,
            delayed_scene,
            cube_after_stand,
            perception,
            OpaqueFunction(function=start_fetch),
        ]
    )
