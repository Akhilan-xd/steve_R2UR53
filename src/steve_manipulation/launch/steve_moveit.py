"""Shared MoveItConfigsBuilder for Steve.

Keep robot_description mappings in sync with steve_simulation/simulation.launch.py
so move_group sees the same URDF that Gazebo spawned.
"""

import os

from ament_index_python.packages import get_package_share_directory


def get_moveit_config(
    arm_type="ur5e",
    include_wrist_camera="true",
    include_pan_tilt="true",
    include_depth_camera="false",
    use_docking_adapter="False",
):
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ImportError as exc:
        raise ImportError(
            "MoveIt 2 is not installed. Install it with:\n"
            "  sudo apt install ros-humble-moveit"
        ) from exc

    sim_share = get_package_share_directory("steve_simulation")
    urdf_path = os.path.join(sim_share, "robots", "mmo_700", "mmo_700.urdf.xacro")

    return (
        MoveItConfigsBuilder("mmo_700", package_name="steve_manipulation")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "use_gazebo": "true",
                "arm_type": str(arm_type),
                "arm_tool": "robotiq_gripper",
                "use_docking_adapter": str(use_docking_adapter),
                "include_wrist_camera": str(include_wrist_camera),
                "include_depth_camera": str(include_depth_camera),
                "include_pan_tilt": str(include_pan_tilt),
            },
        )
        .robot_description_semantic(file_path="config/mmo_700.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
