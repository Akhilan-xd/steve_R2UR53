import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context: LaunchContext, use_sim_time, rviz_config):
    share = get_package_share_directory("steve_manipulation")
    sys.path.insert(0, os.path.join(share, "launch"))
    from steve_moveit import get_moveit_config

    moveit_config = get_moveit_config()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("joint_states", "/joint_states_complete")],
    )

    # Gazebo models are not in the planning scene. Publish matching boxes so
    # the MotionPlanning plugin shows the stand/cube and plans around them.
    pick_scene = Node(
        package="steve_manipulation",
        executable="publish_pick_scene.py",
        name="publish_pick_scene",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "x": ParameterValue(
                    LaunchConfiguration("pick_scene_x"), value_type=float
                ),
                "y": ParameterValue(
                    LaunchConfiguration("pick_scene_y"), value_type=float
                ),
                "z": ParameterValue(
                    LaunchConfiguration("pick_scene_z"), value_type=float
                ),
                "frame": LaunchConfiguration("pick_scene_frame"),
                "spawn_stand": ParameterValue(
                    LaunchConfiguration("spawn_pick_stand"), value_type=bool
                ),
                "spawn_cube": ParameterValue(
                    LaunchConfiguration("spawn_pick_cube"), value_type=bool
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("spawn_pick_scene")),
    )
    return [rviz, pick_scene]


def generate_launch_description():
    share = get_package_share_directory("steve_manipulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(share, "config", "moveit.rviz"),
            ),
            DeclareLaunchArgument(
                "spawn_pick_scene",
                default_value="true",
                description="Add stand/cube boxes to the MoveIt planning scene (RViz)",
            ),
            DeclareLaunchArgument(
                "spawn_pick_stand",
                default_value="true",
                description="Include the stand collision object",
            ),
            DeclareLaunchArgument(
                "spawn_pick_cube",
                default_value="true",
                description="Include the cube collision object",
            ),
            DeclareLaunchArgument(
                "pick_scene_x",
                default_value="-0.20",
                description="Stand/cube world x [m]. Matches manipulation.launch.py cube_x",
            ),
            DeclareLaunchArgument(
                "pick_scene_y",
                default_value="0.55",
                description="Stand/cube world y [m]. Matches manipulation.launch.py cube_y",
            ),
            DeclareLaunchArgument(
                "pick_scene_z",
                default_value="0.82",
                description="Cube center z in pick_scene_frame [m]",
            ),
            DeclareLaunchArgument(
                "pick_scene_frame",
                default_value="world",
                description="Frame for the MoveIt stand/cube (SRDF world == base_link)",
            ),
            OpaqueFunction(function=launch_setup, args=[use_sim_time, rviz_config]),
        ]
    )
