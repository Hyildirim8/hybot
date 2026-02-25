"""
display.launch.py — Launch robot_state_publisher + RViz2 for the ecza rover.

Usage (inside container or with ROS2 sourced on host):
  ros2 launch ecza_description display.launch.py

Optional args:
  use_sim_time:=true      — use /clock topic (for rosbag playback)
  gui:=false              — suppress joint_state_publisher_gui (headless mode)
  rvizconfig:=<path>      — override default RViz config path
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ecza_description")
    default_urdf = os.path.join(pkg_share, "urdf", "rover.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "rviz", "rover.rviz")

    # ── Declare launch arguments ─────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (rosbag) clock if true",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch joint_state_publisher_gui (set false for headless)",
    )
    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz,
        description="Full path to the RViz config file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui          = LaunchConfiguration("gui")
    rvizconfig   = LaunchConfiguration("rvizconfig")

    # ── robot_state_publisher ────────────────────────────────────────────
    # Reads the URDF, publishes /robot_description, and broadcasts TF from
    # /joint_states (provided by ESP32 wheel encoders or joint_state_publisher).
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

    # ── joint_state_publisher_gui ────────────────────────────────────────
    # Provides a slider GUI to manually set joint positions when no hardware
    # joint states are published. Disabled when gui:=false (headless / HW mode).
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gui_arg,
            rvizconfig_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
