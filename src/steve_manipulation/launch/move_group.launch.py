import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(
    context: LaunchContext,
    use_sim_time,
    arm_type,
    include_wrist_camera,
    include_pan_tilt,
    include_depth_camera,
):
    share = get_package_share_directory("steve_manipulation")
    sys.path.insert(0, os.path.join(share, "launch"))
    from steve_moveit import get_moveit_config

    moveit_config = get_moveit_config(
        arm_type=arm_type.perform(context),
        include_wrist_camera=include_wrist_camera.perform(context),
        include_pan_tilt=include_pan_tilt.perform(context),
        include_depth_camera=include_depth_camera.perform(context),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "capabilities": ParameterValue("", value_type=str),
                "disable_capabilities": ParameterValue("", value_type=str),
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
        remappings=[("joint_states", "/joint_states_complete")],
    )
    return [move_group]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_type = LaunchConfiguration("arm_type")
    include_wrist_camera = LaunchConfiguration("include_wrist_camera")
    include_pan_tilt = LaunchConfiguration("include_pan_tilt")
    include_depth_camera = LaunchConfiguration("include_depth_camera")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("arm_type", default_value="ur5e"),
            DeclareLaunchArgument("include_wrist_camera", default_value="true"),
            DeclareLaunchArgument("include_pan_tilt", default_value="true"),
            DeclareLaunchArgument("include_depth_camera", default_value="false"),
            OpaqueFunction(
                function=launch_setup,
                args=[
                    use_sim_time,
                    arm_type,
                    include_wrist_camera,
                    include_pan_tilt,
                    include_depth_camera,
                ],
            ),
        ]
    )
