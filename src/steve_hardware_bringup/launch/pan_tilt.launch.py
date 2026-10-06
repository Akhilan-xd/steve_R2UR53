#!/usr/bin/env python3
"""
Pan-Tilt Launch File
Launches pan-tilt Dynamixel motors and RealSense L515 camera
"""

import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def pan_tilt_motors(context):
    if LaunchConfiguration('enable_pan_tilt').perform(context).lower() != 'true':
        return []
    # TODO: steve_pan_tilt_controller is not in this workspace yet. It must
    # publish pan_tilt_pan_motor_joint and pan_tilt_tilt_motor_joint on
    # /joint_states.
    try:
        controller_share = get_package_share_directory('steve_pan_tilt_controller')
    except PackageNotFoundError:
        return [LogInfo(msg='enable_pan_tilt:=true but steve_pan_tilt_controller is not '
                            'installed; pan-tilt motors are not started.')]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'steve_pan_tilt_controller.launch.py')
        )
    )]


def generate_launch_description():
    # Launch configurations
    enable_camera = LaunchConfiguration('enable_camera')
    camera_serial_no = LaunchConfiguration('camera_serial_no')
    usb_port_id = LaunchConfiguration('usb_port_id')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for pan-tilt unit'
    )

    declare_pan_tilt_cmd = DeclareLaunchArgument(
        'enable_pan_tilt',
        default_value='false',
        description='Start the Dynamixel pan-tilt motors'
    )

    declare_camera_cmd = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense L515 camera'
    )

    declare_serial_no_cmd = DeclareLaunchArgument(
        'camera_serial_no',
        default_value="''",
        description='Serial number of the RealSense camera (leave empty to auto-detect)'
    )

    declare_usb_port_cmd = DeclareLaunchArgument(
        'usb_port_id',
        default_value="''",
        description='USB port ID of the RealSense camera (leave empty to auto-detect)'
    )

    # RealSense L515 camera
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'camera_name': 'pan_tilt_camera',
            'serial_no': camera_serial_no,
            'usb_port_id': usb_port_id,
            'device_type': 'l515',
            'initial_reset': 'true',
            'wait_for_device_timeout': '10.0',
            'reconnect_timeout': '6.0',
            # Color stream configuration
            'enable_color': 'true',
            'rgb_camera.color_profile': '640x480x30',
            # Depth stream configuration
            'enable_depth': 'true',
            'depth_module.depth_profile': '640x480x30',
            # Disable infrared streams (not needed for L515)
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # Alignment and point cloud
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'enable_sync': 'true',
            # The URDF already publishes pan_tilt_camera_link and its optical
            # frames; a second publisher would give them two parents.
            'publish_tf': 'false',
            'diagnostics_period': '1.0'
        }.items(),
        condition=IfCondition(enable_camera)
    )

    # Static transform to correct camera optical frame orientation
    # Camera is mounted -90° to the left, so we rotate +90° around Z to compensate
    camera_optical_correction_color = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_correction_color',
        arguments=[
            '0', '0', '0',  # No translation
            str(math.pi/2), '0', '0',  # +90° rotation around Z axis (yaw)
            'pan_tilt_camera_color_optical_frame',  # Parent (original frame from camera)
            'pan_tilt_camera_color_optical_frame_corrected'  # Child (corrected frame)
        ],
        condition=IfCondition(enable_camera)
    )

    camera_optical_correction_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_correction_depth',
        arguments=[
            '0', '0', '0',  # No translation
            str(math.pi/2), '0', '0',  # +90° rotation around Z axis (yaw)
            'pan_tilt_camera_depth_optical_frame',  # Parent (original frame from camera)
            'pan_tilt_camera_depth_optical_frame_corrected'  # Child (corrected frame)
        ],
        condition=IfCondition(enable_camera)
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_pan_tilt_cmd)
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_serial_no_cmd)
    ld.add_action(declare_usb_port_cmd)
    ld.add_action(OpaqueFunction(function=pan_tilt_motors))
    ld.add_action(realsense_camera)
    ld.add_action(camera_optical_correction_color)
    ld.add_action(camera_optical_correction_depth)

    return ld
