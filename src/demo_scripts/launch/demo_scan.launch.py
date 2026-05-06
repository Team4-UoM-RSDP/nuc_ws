import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    robot_description_pkg = get_package_share_directory("robot_description")

    # Path to the URDF file
    urdf_file = os.path.join(
        robot_description_pkg, "urdf", "leo_cobot", "leo_cobot_sim.urdf.xacro"
    )

    # Declare launch arguments
    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns", default_value="leo", description="Namespace for the robot"
    )

    cobot_ns_arg = DeclareLaunchArgument(
        "cobot_ns", default_value="", description="Namespace for the cobot"
    )

    # Process the URDF with xacro
    robot_description_content = Command(
        [
            "xacro ",
            urdf_file,
            " robot_ns:=",
            LaunchConfiguration("robot_ns"),
            " cobot_ns:=",
            LaunchConfiguration("cobot_ns"),
        ]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            }
        ],
    )

    # Demo scan node
    demo_scan_node = Node(
        package="demo_scripts",
        executable="demo_scan",
        name="demo_scan",
        output="screen",
    )

    return LaunchDescription(
        [robot_ns_arg, cobot_ns_arg, robot_state_publisher_node, demo_scan_node]
    )
