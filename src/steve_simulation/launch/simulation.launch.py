# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
import subprocess

import launch
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

"""
Description:

This launch file is used to start a ROS2 simulation for a Neobotix robot in a specified environment.
It sets up the Gazebo simulator with the chosen robot and environment,
optionally starts the robot state publisher, and enables keyboard teleoperation.

You can launch this file using the following terminal commands:

1. `ros2 launch steve_simulation simulation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch steve_simulation simulation.launch.py my_robot:=mpo_500 world:=neo_track1 arm_type:=ur5e`
   This command launches the simulation with sample values for the arguments.
   !(only mpo_700 and mpo_500 support arms)
"""


def cleanup_stale_gazebo_processes():
    """Remove stale Gazebo processes that can block a fresh launch."""
    for pattern in ["gzserver", "gzclient"]:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", pattern],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            print(f"[WARN] Could not run pkill for {pattern}; it may not be installed")


# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(
    context: LaunchContext,
    my_neo_robot_arg,
    my_neo_env_arg,
    robot_arm_arg,
    docking_adapter_arg,
    include_wrist_camera_arg,
    include_depth_camera_arg,
    include_pan_tilt_arg,
    enable_teleop_arg,
    use_rviz_arg,
    launch_map_server_arg,
    map_arg,
):
    cleanup_stale_gazebo_processes()

    # Create a list to hold all the nodes
    launch_actions = []
    # The perform method of a LaunchConfiguration is called to evaluate its value.
    my_neo_robot = my_neo_robot_arg.perform(context)
    my_neo_environment = my_neo_env_arg.perform(context)
    robot_arm_type = robot_arm_arg.perform(context)
    use_docking_adapter = docking_adapter_arg.perform(context)
    include_wrist_camera = include_wrist_camera_arg.perform(context)
    include_depth_camera = include_depth_camera_arg.perform(context)

    include_pan_tilt = include_pan_tilt_arg.perform(context)
    enable_teleop = enable_teleop_arg.perform(context)
    use_rviz = use_rviz_arg.perform(context)
    launch_map_server = launch_map_server_arg.perform(context)
    map_path = map_arg.perform(context)
    use_sim_time = True

    # Map auto-resolution logic
    if launch_map_server.lower() == "true":
        if map_path and not os.path.exists(map_path):
            try:
                sim_pkg_share = get_package_share_directory("steve_simulation")
                potential_map = os.path.join(sim_pkg_share, "maps", f"{map_path}.yaml")
                if os.path.exists(potential_map):
                    map_path = potential_map
                else:
                    potential_map_asis = os.path.join(sim_pkg_share, "maps", map_path)
                    if os.path.exists(potential_map_asis):
                        map_path = potential_map_asis
                    elif not map_path.endswith('.yaml'):
                        potential_map_yaml = os.path.join(sim_pkg_share, "maps", f"{map_path}.yaml")
                        if os.path.exists(potential_map_yaml):
                            map_path = potential_map_yaml
            except Exception as e:
                print(f"[WARN] Could not resolve map path for '{map_path}': {e}")

        if map_path == "":
            if my_neo_environment in ["neo_workshop", "neo_track1", "small_house"]:
                world_name = my_neo_environment
            else:
                world_name = os.path.splitext(os.path.basename(my_neo_environment))[0]
            
            try:
                map_path = os.path.join(
                    get_package_share_directory("steve_simulation"),
                    "maps",
                    f"{world_name}.yaml",
                )
                print(f"[INFO] Auto-detected map file: {map_path}")
            except Exception:
                print(f"[WARN] Could not auto-detect map for world: {world_name}")

    if launch_map_server.lower() == "true" and (not map_path or not os.path.exists(map_path)):
        print(f"[WARN] Map server requested but map file not found: {map_path}")
        print("[WARN] Disabling map server")
        launch_map_server = "false"

    print("\n" + "=" * 70)
    print("  Neobotix ROS2 Simulation Launch")
    print("=" * 70)
    print(f"[INFO] Requested Robot: {my_neo_robot}")
    print(f"[INFO] Requested World: {my_neo_environment}")
    print(f"[INFO] Requested Arm Type: {robot_arm_type if robot_arm_type else 'None'}")
    print(f"[INFO] Docking Adapter: {use_docking_adapter}")
    print("=" * 70 + "\n")

    robots = ["mpo_700", "mp_400", "mp_500", "mpo_500", "mmo_700"]

    # Checking if the user has selected a robot that is valid
    if my_neo_robot not in robots:
        # Incase of an invalid selection
        print(f"[WARNING] Invalid robot '{my_neo_robot}' selected!")
        print(f"[WARNING] Available robots: {', '.join(robots)}")
        print("[WARNING] Defaulting to 'mpo_700'")
        my_neo_robot = "mpo_700"
    else:
        print(f"[INFO] Robot validation successful: {my_neo_robot}")

    with open("robot_name.txt", "w") as file:
        file.write(my_neo_robot)

    # Remove arm_type if robot does not support it
    if robot_arm_type != "":
        if (
            my_neo_robot != "mpo_700"
            and my_neo_robot != "mpo_500"
            and my_neo_robot != "mmo_700"
        ):
            print(f"[WARNING] Robot '{my_neo_robot}' does not support arm integration")
            print(
                "[WARNING] Arm support is only available for: mpo_700, mpo_500, mmo_700"
            )
            print("[WARNING] Disabling arm_type")
            robot_arm_type = ""
        else:
            print(f"[INFO] Arm integration enabled: {robot_arm_type}")

    # Get the required paths for the world and robot robot_description_urdf
    if (
        my_neo_environment == "neo_workshop"
        or my_neo_environment == "neo_track1"
        or my_neo_environment == "small_house"
        or my_neo_environment == "steve_house"
    ):
        world_path = os.path.join(
            get_package_share_directory("steve_simulation"),
            "worlds",
            my_neo_environment + ".world",
        )
        print(f"[INFO] Using built-in world: {my_neo_environment}")
    else:
        world_path = my_neo_environment
        print(f"[INFO] Using custom world file: {world_path}")
    print(f"[INFO] World path: {world_path}")

    # Setting the world and starting the Gazebo
    pkg_share_path = os.path.join(get_package_prefix("steve_simulation"), "share")
    workspace_install_share = os.path.join(os.getcwd(), "install", "share")

    steve_simulation_models = os.path.join(
        get_package_share_directory("steve_simulation"), "models"
    )

    # Build GAZEBO_MODEL_PATH with only necessary directories
    model_paths = [workspace_install_share, pkg_share_path]
    try:
        robotiq_share = os.path.dirname(get_package_share_directory("robotiq_description"))
        if os.path.exists(robotiq_share):
            model_paths.append(robotiq_share)
    except Exception as exc:
        print(f"[WARN] robotiq_description not found on the ROS 2 index: {exc}")
    # Add AWS RoboMaker models for small house world
    if os.path.exists(steve_simulation_models):
        model_paths.append(steve_simulation_models)

    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + os.pathsep.join(model_paths)
    else:
        os.environ["GAZEBO_MODEL_PATH"] = os.pathsep.join(model_paths)

    print(f"[DEBUG] GAZEBO_MODEL_PATH set to: {os.environ['GAZEBO_MODEL_PATH']}")

    # Set GAZEBO_RESOURCE_PATH for shader libs and rendering resources
    if "GAZEBO_RESOURCE_PATH" not in os.environ:
        gazebo_resource_paths = ["/usr/share/gazebo-11", "/usr/share/gazebo"]
        os.environ["GAZEBO_RESOURCE_PATH"] = os.pathsep.join(gazebo_resource_paths)
        print(
            f"[DEBUG] GAZEBO_RESOURCE_PATH set to: {os.environ['GAZEBO_RESOURCE_PATH']}"
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "false",
        }.items(),
    )

    # Getting the robot description xacro
    robot_description_xacro = os.path.join(
        get_package_share_directory("steve_simulation"),
        "robots/" + my_neo_robot + "/",
        my_neo_robot + ".urdf.xacro",
    )
    print(f"[INFO] Robot URDF path: {robot_description_xacro}")

    # Docking adapter is only for MPO 700
    if my_neo_robot != "mpo_700":
        if use_docking_adapter == "True":
            print(f"[WARNING] Docking adapter only supported for mpo_700")
            print("[WARNING] Disabling docking adapter")
        use_docking_adapter = False

    # use_gazebo is set to True since this code launches the robot in simulation
    xacro_args = {
        "use_gazebo": "true",
        "arm_type": robot_arm_type,
        "arm_tool": "robotiq_gripper",
        "use_docking_adapter": use_docking_adapter,
        "include_wrist_camera": include_wrist_camera,
        "include_depth_camera": include_depth_camera,
        "include_pan_tilt": include_pan_tilt,
    }
    print("[INFO] Processing URDF with xacro...")
    print(f"[INFO] Xacro arguments: {xacro_args}")

    # Use xacro to process the file with the argunments above
    robot_description_file = xacro.process_file(
        robot_description_xacro, mappings=xacro_args
    ).toxml()
    print("[INFO] URDF processing complete")

    if robot_arm_type != "":
        initial_positions_path = os.path.join(
            get_package_share_directory("steve_simulation"),
            "configs",
            "ur_config",
            "initial_joint_positions.yaml",
        )
        try:
            with open(initial_positions_path, "r") as positions_file:
                arm_home = yaml.safe_load(positions_file)
            print(f"[INFO] Arm spawn (home) pose from {initial_positions_path}:")
            for joint_name, joint_value in arm_home.items():
                print(f"[INFO]   {joint_name}: {joint_value}")
        except (OSError, yaml.YAMLError) as exc:
            print(f"[WARN] Could not read arm home pose file: {exc}")

    # Spawning the robot
    # Using /usr/bin/python3 explicitly to avoid Anaconda conflicts
    spawn_entity = Node(
        package=None,
        executable="/usr/bin/python3",
        arguments=[
            os.path.join(
                get_package_prefix("gazebo_ros"), "lib", "gazebo_ros", "spawn_entity.py"
            ),
            "-entity",
            my_neo_robot,
            "-topic",
            "/robot_description",
            "-timeout",
            "300.0",
            "-z",
            "0.02",  # slight lift so wheel cylinders are not spawned in the floor
            "-Y",
            "3.14159",  # 180° rotation around Z-axis (pi radians)
        ],
        output="screen",
    )

    # Gazebo / ros2_control only publish actuated joints. The Robotiq fingers are
    # mimic joints, so they never appear on /joint_states. RViz follows TF from
    # robot_state_publisher and will hide those links unless we fill them in.
    complete_joint_states_topic = "/joint_states_complete"
    missing_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="missing_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_file,
                "source_list": ["/joint_states"],
                "rate": 50.0,
                "use_mimic_tags": True,
            }
        ],
        remappings=[("joint_states", complete_joint_states_topic)],
    )

    # Start the robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description_file}
        ],
        remappings=[("joint_states", complete_joint_states_topic)],
    )

    # Starting the teleop node
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        # prefix = 'xterm -e',
        name="teleop",
        parameters=[
            {"speed": 0.3},
            {"turn": 0.5},
        ],
    )

    # RViz for visualization and joint control
    rviz_config = os.path.join(
        get_package_share_directory("steve_simulation"),
        "rviz",
        "robot_description_rviz.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    pan_tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_controller", "-c", "/controller_manager"],
    )

    # See Issue: https://github.com/ros2/rclpy/issues/1287
    # Cannot delete the newly create file. The user has to delete it on his own
    # Refer documentation for more info
    # shutdown_event = RegisterEventHandler(
    #         OnShutdown(
    #             on_shutdown=[os.remove('robot_name.txt')]
    #         )
    #     )

    # The required nodes can just be appended to the launch_actions list
    print("\n[INFO] Launching nodes...")
    print("[INFO] - Robot State Publisher")
    launch_actions.append(start_robot_state_publisher_cmd)
    print("[INFO] - Joint State Publisher (fills gripper mimic joints for RViz)")
    launch_actions.append(missing_joint_state_publisher)

    # Collect controller spawners to delay them
    controller_spawners = []
    if robot_arm_type != "":
        print("[INFO] - Joint State Broadcaster (delayed 2s)")
        print("[INFO] - Joint Trajectory Controller (delayed 2s)")
        controller_spawners.append(joint_state_broadcaster_spawner)
        controller_spawners.append(initial_joint_controller_spawner_stopped)

    if include_pan_tilt == "true":
        print("[INFO] - Pan-Tilt Controller (delayed 2s)")
        controller_spawners.append(pan_tilt_controller_spawner)

    # Add 2-second delay to controller spawners to allow gazebo_ros2_control plugin to initialize
    # Reduced from 15s to prevent arm from falling due to gravity before controllers activate
    if controller_spawners:
        delayed_controllers = TimerAction(period=2.0, actions=controller_spawners)
        launch_actions.append(delayed_controllers)

    print("[INFO] - Gazebo Simulator")
    launch_actions.append(gazebo)
    print(f"[INFO] - Spawn Entity ({my_neo_robot})")
    launch_actions.append(spawn_entity)

    if use_rviz.lower() == "true":
        print("[INFO] - RViz2 (for visualization and joint control)")
        launch_actions.append(rviz)
    else:
        print("[INFO] - RViz2: Disabled (use use_rviz:=true to enable)")

    if enable_teleop == "true":
        print("[INFO] - Teleop Twist Keyboard")
        launch_actions.append(teleop)
    else:
        print(
            "[INFO] - Teleop Twist Keyboard: Disabled (use enable_teleop:=true to enable)"
        )

    if launch_map_server.lower() == "true":
        print(f"[INFO] - Map Server (map: {map_path})")
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path}, {'use_sim_time': use_sim_time}]
        )

        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
        launch_actions.append(map_server_node)
        launch_actions.append(lifecycle_manager_node)

    # launch_actions.append(shutdown_event)
    print("\n" + "=" * 70)
    print("  Launch Configuration Complete")
    print("=" * 70)
    print(f"  Robot: {my_neo_robot}")
    print(f"  World: {my_neo_environment}")
    print(f"  Arm: {robot_arm_type if robot_arm_type else 'Disabled'}")
    print(f"  RViz: {'Enabled' if use_rviz.lower() == 'true' else 'Disabled'}")
    print(f"  Nodes: {len(launch_actions)} total")
    print("=" * 70 + "\n")

    return launch_actions


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' and 'world' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        "my_robot",
        default_value="mmo_700",
        description='Only set to mmo_700 for this project',
    )

    declare_world_name_arg = DeclareLaunchArgument(
        "world",
        default_value="small_house",
        description='Available worlds: "neo_track1", "neo_workshop", "small_house"',
    )

    declare_arm_type_cmd = DeclareLaunchArgument(
        "arm_type",
        default_value="ur5e",
        description="Arm Types:\n"
        "\t Elite Arms: ec66, cs66\n"
        "\t Universal Robotics: ur5, ur10, ur5e, ur10e",
    )

    declare_docking_adapter_cmd = DeclareLaunchArgument(
        "use_docking_adapter",
        default_value="False",
        description="Set True to use the docking adapter for the robot\n"
        "\t Neobotix: docking_adapter",
    )

    declare_wrist_camera_cmd = DeclareLaunchArgument(
        "include_wrist_camera",
        default_value="true",
        description="Include wrist D405 camera on arm",
    )

    declare_depth_camera_cmd = DeclareLaunchArgument(
        "include_depth_camera",
        default_value="false",
        description="Include front depth camera",
    )

    declare_pan_tilt_cmd = DeclareLaunchArgument(
        "include_pan_tilt",
        default_value="true",
        description="Include pan-tilt camera tower",
    )

    declare_enable_teleop_cmd = DeclareLaunchArgument(
        "enable_teleop",
        default_value="false",
        description="Enable teleop_twist_keyboard (requires interactive terminal, disable for Docker)",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz for visualization"
    )

    declare_map_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file (empty = auto-detect from world)",
    )

    declare_launch_map_server_cmd = DeclareLaunchArgument(
        "launch_map_server",
        default_value="true",
        description="Launch Map Server and Lifecycle Manager",
    )

    # Create launch configuration variables for the robot and map name
    my_neo_robot_arg = LaunchConfiguration("my_robot")

    my_neo_env_arg = LaunchConfiguration("world")
    robot_arm_arg = LaunchConfiguration("arm_type")
    docking_adapter_arg = LaunchConfiguration("use_docking_adapter")
    include_wrist_camera_arg = LaunchConfiguration("include_wrist_camera")
    include_depth_camera_arg = LaunchConfiguration("include_depth_camera")
    map_arg = LaunchConfiguration("map")
    launch_map_server_arg = LaunchConfiguration("launch_map_server")

    include_pan_tilt_arg = LaunchConfiguration("include_pan_tilt")
    enable_teleop_arg = LaunchConfiguration("enable_teleop")
    use_rviz_arg = LaunchConfiguration("use_rviz")

    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_arm_type_cmd)
    ld.add_action(declare_docking_adapter_cmd)
    ld.add_action(declare_wrist_camera_cmd)
    ld.add_action(declare_depth_camera_cmd)

    ld.add_action(declare_pan_tilt_cmd)
    ld.add_action(declare_enable_teleop_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_launch_map_server_cmd)

    context_arguments = [
        my_neo_robot_arg,
        my_neo_env_arg,
        robot_arm_arg,
        docking_adapter_arg,
        include_wrist_camera_arg,
        include_depth_camera_arg,
        include_pan_tilt_arg,
        enable_teleop_arg,
        use_rviz_arg,
        launch_map_server_arg,
        map_arg,
    ]

    opq_function = OpaqueFunction(function=launch_setup, args=context_arguments)

    ld.add_action(opq_function)

    return ld
