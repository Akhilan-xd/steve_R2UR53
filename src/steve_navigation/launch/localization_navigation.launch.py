import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('steve_navigation')
    sim_dir = get_package_share_directory('steve_simulation')
    default_map = os.path.join(sim_dir, 'maps', 'small_house.yaml')

    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_simulation = LaunchConfiguration('launch_simulation')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=default_map, description='Full path to map yaml')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'navigation_sim.yaml'),
        description='Full path to the ROS2 parameters file to use')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz with Nav2 tools')
    declare_launch_simulation_cmd = DeclareLaunchArgument(
        'launch_simulation', default_value='true',
        description='Start Gazebo and spawn the robot (set false if sim is already running)')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'simulation.launch.py')),
        condition=IfCondition(launch_simulation),
        launch_arguments={
            'launch_map_server': 'false',
            'use_rviz': 'false',
            'enable_teleop': 'false',
        }.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'localization_amcl.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_neo.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_launch_simulation_cmd)
    ld.add_action(simulation)
    ld.add_action(localization)
    ld.add_action(navigation)
    return ld
