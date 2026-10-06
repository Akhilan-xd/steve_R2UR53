#!/usr/bin/env python3
"""
Teleop Launch File
Launches the joystick driver and neo_teleop2 for the omnidirectional base
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('steve_hardware_bringup')

    robot_namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')

    teleop_config = os.path.join(pkg_share, 'config', 'teleop.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for teleop'
    )

    declare_joy_dev_cmd = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='steve_joy_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[{'dev': joy_dev, 'deadzone': 0.12}]
    )

    teleop_node = Node(
        package='neo_teleop2',
        executable='neo_teleop2_node',
        name='steve_teleop_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[teleop_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_joy_dev_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
