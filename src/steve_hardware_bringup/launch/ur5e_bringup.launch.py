import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for UR5e robotic arm
    Uses the official Universal Robots ROS2 driver
    """

    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.102',  # TODO: Update with actual UR5e IP
        description='IP address of the UR5e robot'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing without real robot'
    )

    # UR5e bringup from ur_robot_driver
    ur5e_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
        }.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        ur5e_bringup
    ])
