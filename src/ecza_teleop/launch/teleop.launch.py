from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy_dev = LaunchConfiguration("joy_dev")
    joy_dev_name = LaunchConfiguration("joy_dev_name")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate")
    joy_coalesce_interval_ms = LaunchConfiguration("joy_coalesce_interval_ms")
    joy_respawn_delay = LaunchConfiguration("joy_respawn_delay")

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path used when joy_dev_name is empty.",
    )
    joy_dev_name_arg = DeclareLaunchArgument(
        "joy_dev_name",
        default_value="",
        description="Optional joystick device name. Prefer this for hotplug-stable matching.",
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        "joy_deadzone",
        default_value="0.05",
        description="Deadzone passed to joy_node.",
    )
    joy_autorepeat_rate_arg = DeclareLaunchArgument(
        "joy_autorepeat_rate",
        default_value="20.0",
        description="Autorepeat rate passed to joy_node.",
    )
    joy_coalesce_interval_ms_arg = DeclareLaunchArgument(
        "joy_coalesce_interval_ms",
        default_value="10",
        description="Coalesce interval in milliseconds passed to joy_node.",
    )
    joy_respawn_delay_arg = DeclareLaunchArgument(
        "joy_respawn_delay",
        default_value="2.0",
        description="Seconds to wait before respawning joy_node after device-open failure.",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        respawn=True,
        respawn_delay=joy_respawn_delay,
        parameters=[{
            "dev": joy_dev,
            "dev_name": joy_dev_name,
            "deadzone": joy_deadzone,
            "autorepeat_rate": joy_autorepeat_rate,
            "coalesce_interval_ms": joy_coalesce_interval_ms,
        }],
    )

    teleop_node = Node(
        package="ecza_teleop",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
    )

    slam_manager_node = Node(
        package="ecza_teleop",
        executable="slam_manager_node",
        name="slam_manager_node",
        output="screen",
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_dev_name_arg,
        joy_deadzone_arg,
        joy_autorepeat_rate_arg,
        joy_coalesce_interval_ms_arg,
        joy_respawn_delay_arg,
        joy_node,
        teleop_node,
        slam_manager_node,
    ])
