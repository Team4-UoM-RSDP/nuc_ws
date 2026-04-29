"""
Launch file for the cobot controller task connecting to real hardware.
Loads MoveIt2 configuration without Gazebo simulation.
Connects to the controller node built from cobot_node.cpp.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Configure and launch MoveIt2 and pick_block for real hardware."""

    # Get package share directories
    moveit_config_dir = get_package_share_directory("cobot_moveit_config")
    robot_name_str = "mycobot_280pi"
    config_path = os.path.join(moveit_config_dir, "config")

    # Define config file paths
    srdf_model_path = os.path.join(config_path, f"{robot_name_str}.srdf")
    moveit_controllers_file_path = os.path.join(config_path, "moveit_controllers.yaml")
    joint_limits_file_path = os.path.join(config_path, "joint_limits.yaml")
    kinematics_file_path = os.path.join(config_path, "kinematics.yaml")

    # Create MoveIt configuration for real hardware (no simulation)
    moveit_config = (
        MoveItConfigsBuilder(robot_name_str, package_name="cobot_moveit_config")
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(pipelines=["stomp", "ompl"])
        .to_moveit_configs()
    )

    # Get the robot description to pass to pick_block
    moveit_config_dict = moveit_config.to_dict()

    # Move group node for MoveIt2 planning and execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            {"use_sim_time": False},  # Real hardware, not simulation
        ],
    )

    # Cobot controller node - needs robot_description parameter
    cobot_node = Node(
        package="cobot_controller",
        executable="cobot_control",
        name="cobot_control",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            moveit_config_dict,  # Pass robot_description and other MoveIt config
        ],
        remappings=[
            ("joint_states", "joint_states"),
            ("gripper_command", "gripper_command"),
        ],
    )

    # RViz for path planning visualization
    rviz_config_file = os.path.join(moveit_config_dir, "rviz", "move_group.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config_dict,
            {"use_sim_time": False},
        ],
    )

    # Return nodes - move_group, cobot controller, and rviz
    return [move_group_node, cobot_node, rviz_node]


def generate_launch_description():
    """Generate launch description."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="mycobot_280pi",
                description="Name of the robot",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
