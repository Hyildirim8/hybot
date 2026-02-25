"""
hardware.launch.py — Launch ros2_control stack for the real ESP32 hardware.

Architecture:
  controller_manager
    └─ topic_based_ros2_control  (hardware interface → /wheel_velocities topics)
    └─ mecanum_drive_controller  (cmd_vel → wheel velocity commands + odometry)
    └─ joint_state_broadcaster   (joint states → /joint_states for TF/RViz)
  robot_state_publisher          (URDF + joint_states → TF tree)

Data flow:
  /cmd_vel (Twist)
    → mecanum_drive_controller (IK + odometry)
      → /wheel_velocities_cmd (Float32MultiArray, [fl fr rl rr] rad/s)
        → ESP32 velocity_subscriber.c → motors
  ESP32 → /wheel_velocities (Float32MultiArray)
    → topic_based_ros2_control hardware interface
      → joint_state_broadcaster → /joint_states → robot_state_publisher → TF
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ecza_description")
    default_urdf = os.path.join(pkg_share, "urdf", "rover.urdf.xacro")
    controllers_yaml = "/config/controllers.yaml"

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

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": use_sim_time,
        }],
    )

    # ── controller_manager ────────────────────────────────────────────────
    # Loads topic_based_ros2_control hardware plugin + manages controllers.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_yaml,
        ],
    )

    # ── Spawn joint_state_broadcaster (immediate) ─────────────────────────
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── Spawn mecanum_drive_controller (after jsb is up) ──────────────────
    spawn_mecanum = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "mecanum_drive_controller",
                "--controller-manager", "/controller_manager",
                "--param-file", controllers_yaml,
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_jsb,
        spawn_mecanum,
    ])
