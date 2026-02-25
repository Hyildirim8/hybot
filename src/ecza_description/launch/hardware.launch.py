"""
hardware.launch.py — Launch robot_state_publisher for the real ESP32 hardware stack.
RViz is launched separately on the host desktop.

In hardware mode:
  - robot_state_publisher reads the URDF and broadcasts TF.
  - joint_states are published by kinematics_node (integrates wheel velocities).
  - joint_state_publisher is NOT used to avoid publishing zeros that fight
    with the kinematics_node joint_states publisher.

Usage:
  ros2 launch ecza_description hardware.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ecza_description")
    default_urdf = os.path.join(pkg_share, "urdf", "rover.urdf.xacro")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", default_urdf]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
    ])
