#!/usr/bin/env python3
"""
Main Hardware Bringup Launch File
Launches all hardware components for the Steve robot
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

UR_ARMS = ["ur5", "ur10", "ur5e", "ur10e"]

# Joints without a real driver yet. joint_state_publisher holds them here so
# the full TF tree (pan-tilt camera, gripper fingers) is always published.
# Keep in sync with jsp_zeros in steve_simulation/launch/simulation.launch.py.
HELD_JOINTS = {
    "pan_tilt_pan_motor_joint": 0.0,
    "pan_tilt_tilt_motor_joint": 0.41,
    "robotiq_85_left_knuckle_joint": 0.0,
}


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def execution_stage(context: LaunchContext):
    robot_namespace = LaunchConfiguration("robot_namespace")
    namespace = robot_namespace.perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)
    arm_tool = LaunchConfiguration("arm_tool").perform(context)

    rp_ns = ""
    if namespace not in ("", "/"):
        rp_ns = namespace.strip("/") + "/"

    launches = []
    pkg_share = get_package_share_directory("steve_hardware_bringup")
    sim_share = get_package_share_directory("steve_simulation")

    # Same description MoveIt plans with, minus the Gazebo plugins. The
    # pan-tilt tower and wrist camera are always mounted, so their links stay
    # in the model even when their drivers are off.
    urdf_file = os.path.join(sim_share, "robots", "mmo_700", "mmo_700.urdf.xacro")
    robot_description_content = xacro.process_file(
        urdf_file,
        mappings={
            "use_gazebo": "false",
            "arm_type": arm_type,
            "arm_tool": arm_tool,
            "include_wrist_camera": "true",
            "include_depth_camera": "false",
            "include_pan_tilt": "true",
        },
    ).toxml()

    # Joint states: drivers -> /joint_states, relayed reliably to
    # /joint_states_hw, completed with the held joints on
    # /joint_states_complete. RSP, MoveIt and the scripts all read the
    # complete topic, exactly as in simulation.
    complete_joint_states_topic = "/joint_states_complete"

    joint_state_relay = Node(
        package=None,
        executable="/usr/bin/python3",
        arguments=[os.path.join(sim_share, "launch", "relay_joint_states.py")],
        name="joint_state_relay",
        output="screen",
    )
    launches.append(joint_state_relay)

    held = dict(HELD_JOINTS)
    if _flag(context, "enable_pan_tilt"):
        held.pop("pan_tilt_pan_motor_joint")
        held.pop("pan_tilt_tilt_motor_joint")
    if _flag(context, "enable_gripper"):
        held.pop("robotiq_85_left_knuckle_joint")
    jsp_parameters = {
        "robot_description": robot_description_content,
        "source_list": ["/joint_states_hw"],
        "rate": 50,
        "use_mimic_tags": True,
    }
    for joint_name, value in held.items():
        jsp_parameters[f"zeros.{joint_name}"] = value

    launches.append(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="missing_joint_state_publisher",
            output="screen",
            parameters=[jsp_parameters],
            remappings=[("joint_states", complete_joint_states_topic)],
        )
    )

    launches.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="steve_robot_state_publisher",
            output="screen",
            namespace=robot_namespace,
            parameters=[
                {"robot_description": robot_description_content, "frame_prefix": rp_ns}
            ],
            remappings=[("joint_states", complete_joint_states_topic)],
        )
    )

    # 1. Robot Base (Relayboard + Kinematics)
    if _flag(context, "enable_base"):
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "robot_base.launch.py")
                ),
                launch_arguments={"namespace": robot_namespace}.items(),
            )
        )

    # 2. LiDAR Sensors (Nav2 reads lidar_1/scan_filtered and lidar_2/scan_filtered)
    if _flag(context, "enable_lidar"):
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "lidar.launch.py")
                ),
                launch_arguments={"namespace": robot_namespace}.items(),
            )
        )

    # 3. Joystick teleop
    if _flag(context, "enable_teleop"):
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "teleop.launch.py")
                ),
                launch_arguments={"namespace": robot_namespace}.items(),
            )
        )

    # 4. UR Arm
    if _flag(context, "enable_arm") and arm_type in UR_ARMS:
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "ur5e_arm.launch.py")
                ),
                launch_arguments={
                    "ur_type": arm_type,
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "tf_prefix": arm_type,
                    # The Robotiq gripper talks through the UR tool port.
                    "use_tool_communication": LaunchConfiguration("enable_gripper"),
                }.items(),
            )
        )

    # 5. Gripper
    if _flag(context, "enable_gripper"):
        # TODO: start a Robotiq 2F-85 driver on /tmp/ttyUR that offers
        # /robotiq_gripper_controller/gripper_cmd (control_msgs/GripperCommand)
        # and publishes robotiq_85_left_knuckle_joint on /joint_states.
        launches.append(
            LogInfo(
                msg="enable_gripper:=true starts UR tool communication only; "
                "no Robotiq driver is wired into this bringup yet."
            )
        )

    # 6. Pan-tilt motors and pan-tilt L515 camera
    if _flag(context, "enable_pan_tilt") or _flag(context, "enable_camera"):
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "pan_tilt.launch.py")
                ),
                launch_arguments={
                    "namespace": robot_namespace,
                    "enable_pan_tilt": LaunchConfiguration("enable_pan_tilt"),
                    "enable_camera": LaunchConfiguration("enable_camera"),
                    "camera_serial_no": LaunchConfiguration("pan_tilt_camera_serial_no"),
                }.items(),
            )
        )

    # 7. Wrist D405 camera
    if _flag(context, "enable_wrist_camera"):
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    )
                ),
                launch_arguments={
                    "camera_name": "wrist_camera",
                    "serial_no": LaunchConfiguration("wrist_camera_serial_no"),
                    "device_type": "d405",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "false",
                    # The URDF already publishes the camera frames.
                    "publish_tf": "false",
                }.items(),
            )
        )

    return launches


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Top-level namespace for the robot",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="ur5e",
            description="Arm type - Options: ur5, ur5e, ur10, ur10e",
        ),
        DeclareLaunchArgument(
            "arm_tool",
            default_value="robotiq_gripper",
            description="End effector in the robot description: none or robotiq_gripper",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.102",
            description="IP address of the UR arm",
        ),
        DeclareLaunchArgument(
            "enable_base",
            default_value="true",
            description="Start the relayboard and omnidrive kinematics",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="Start both SICK S300 scanners and their filters",
        ),
        DeclareLaunchArgument(
            "enable_teleop",
            default_value="true",
            description="Start the joystick and neo_teleop2",
        ),
        DeclareLaunchArgument(
            "enable_arm",
            default_value="true",
            description="Start the UR ros2_control driver",
        ),
        DeclareLaunchArgument(
            "enable_gripper",
            default_value="false",
            description="Robotiq 2F-85 over the UR tool port (driver not wired yet)",
        ),
        DeclareLaunchArgument(
            "enable_camera",
            default_value="true",
            description="Start the RealSense L515 on the pan-tilt tower",
        ),
        DeclareLaunchArgument(
            "enable_pan_tilt",
            default_value="false",
            description="Start the Dynamixel pan-tilt motors (needs steve_pan_tilt_controller)",
        ),
        DeclareLaunchArgument(
            "enable_wrist_camera",
            default_value="false",
            description="Start the RealSense D405 wrist camera",
        ),
        DeclareLaunchArgument(
            "pan_tilt_camera_serial_no",
            default_value="''",
            description="L515 serial number. Set both serials when two RealSense cameras run",
        ),
        DeclareLaunchArgument(
            "wrist_camera_serial_no",
            default_value="''",
            description="D405 serial number. Set both serials when two RealSense cameras run",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=execution_stage)])
