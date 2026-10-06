#!/usr/bin/env python3
"""
LiDAR Launch File
Launches dual SICK S300 LiDAR sensors with scan filtering
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory('steve_hardware_bringup')

    # Launch configurations
    robot_namespace = LaunchConfiguration('namespace', default='')
    context = LaunchContext()

    # Configuration files
    s300_1_config = os.path.join(pkg_share, 'config', 'lidar', 's300_1.yaml')
    s300_2_config = os.path.join(pkg_share, 'config', 'lidar', 's300_2.yaml')
    s300_filter_1_config = os.path.join(pkg_share, 'config', 'lidar', 's300_filter_1.yaml')
    s300_filter_2_config = os.path.join(pkg_share, 'config', 'lidar', 's300_filter_2.yaml')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for LiDAR sensors'
    )

    # LiDAR 1 nodes
    lidar_1_node = Node(
        package='neo_sick_s300-2',
        executable='neo_sick_s300_node',
        name='steve_sick_s300_node',
        namespace=robot_namespace.perform(context) + 'lidar_1',
        output='screen',
        parameters=[s300_1_config]
    )

    lidar_1_filter_node = Node(
        package='neo_sick_s300-2',
        executable='neo_scan_filter_node',
        name='steve_scan_filter_node',
        namespace=robot_namespace.perform(context) + 'lidar_1',
        output='screen',
        parameters=[s300_filter_1_config]
    )

    # LiDAR 2 nodes
    lidar_2_node = Node(
        package='neo_sick_s300-2',
        executable='neo_sick_s300_node',
        name='steve_sick_s300_node',
        namespace=robot_namespace.perform(context) + 'lidar_2',
        output='screen',
        parameters=[s300_2_config]
    )

    lidar_2_filter_node = Node(
        package='neo_sick_s300-2',
        executable='neo_scan_filter_node',
        name='steve_scan_filter_node',
        namespace=robot_namespace.perform(context) + 'lidar_2',
        output='screen',
        parameters=[s300_filter_2_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(GroupAction([
        PushRosNamespace(namespace=robot_namespace),
        lidar_1_node,
        lidar_1_filter_node,
        lidar_2_node,
        lidar_2_filter_node
    ]))

    return ld
