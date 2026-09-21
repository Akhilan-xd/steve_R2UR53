# Copyright (c) 2022 Neobotix GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('steve_navigation')
    default_params = os.path.join(bringup_dir, 'config', 'mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to slam_toolbox param file')

    declare_param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_params,
        description='Alias for params_file')

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz to watch the map being built')

    start_sync_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_output': 'screen',
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_param_file_arg)
    ld.add_action(declare_use_rviz_arg)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_rviz)
    return ld
