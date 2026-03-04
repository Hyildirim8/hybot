"""
nav2_navigation.launch.py — Full autonomous navigation stack for ecza-robotu.

Modes (set via 'mode' launch argument):
  slam      — Online SLAM + Nav2. Builds map while navigating.
              No pre-existing map needed. Good for first exploration.
              Publishes: /map (grows over time)

  nav       — Nav2 with AMCL localisation against a saved map.
              Requires: map:=<path-to-map.yaml>
              Good for repeatable navigation in a known environment.

Usage:
  # SLAM mode (explore + map simultaneously):
  ros2 launch ecza_navigation nav2_navigation.launch.py mode:=slam

  # Navigation mode (known map):
  ros2 launch ecza_navigation nav2_navigation.launch.py \\
      mode:=nav map:=/maps/my_room.yaml

Data flow:
  RPLidar → /scan
    → SLAM Toolbox (slam mode) → /map + odom→map TF
    → AMCL       (nav  mode)  → odom→map TF (localisation)
  /odom (from mecanum_drive_controller)
    → Nav2 costmaps, planners, controllers
  Nav2 → /cmd_vel → mecanum_drive_controller → ESP32 → motors
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_nav   = get_package_share_directory("ecza_navigation")
    pkg_nav2  = get_package_share_directory("nav2_bringup")

    nav2_params_file  = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    slam_params_file  = os.path.join(pkg_nav, "config", "slam_params.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        description="Navigation mode: 'slam' (mapping) or 'nav' (localisation)",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="[nav mode only] Path to map YAML file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_params_file,
        description="Nav2 params file path",
    )

    mode           = LaunchConfiguration("mode")
    map_yaml       = LaunchConfiguration("map")
    use_sim_time   = LaunchConfiguration("use_sim_time")
    params_file    = LaunchConfiguration("params_file")

    # ── SLAM Toolbox (slam mode only) ─────────────────────────────────────
    slam_node = Node(
        condition=IfCondition(EqualsSubstitution(mode, "slam")),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Nav2 Bringup ──────────────────────────────────────────────────────
    # slam mode: no map_server, no AMCL (SLAM publishes /map itself)
    nav2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "navigation_launch.py")
        ),
        condition=IfCondition(EqualsSubstitution(mode, "slam")),
        launch_arguments={
            "use_sim_time":  use_sim_time,
            "params_file":   params_file,
            "use_composition": "False",
        }.items(),
    )

    # nav mode: full bringup with map_server + AMCL
    nav2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        condition=IfCondition(EqualsSubstitution(mode, "nav")),
        launch_arguments={
            "use_sim_time":  use_sim_time,
            "params_file":   params_file,
            "map":           map_yaml,
            "use_composition": "False",
        }.items(),
    )

    # ── Logging ───────────────────────────────────────────────────────────
    log_slam = LogInfo(
        condition=IfCondition(EqualsSubstitution(mode, "slam")),
        msg="[ecza_navigation] SLAM mode — building map with SLAM Toolbox. "
            "Save map: ros2 run nav2_map_server map_saver_cli -f /maps/my_map",
    )
    log_nav = LogInfo(
        condition=IfCondition(EqualsSubstitution(mode, "nav")),
        msg="[ecza_navigation] Nav mode — localising with AMCL against saved map.",
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        use_sim_time_arg,
        params_file_arg,
        log_slam,
        log_nav,
        slam_node,
        nav2_slam,
        nav2_nav,
    ])
