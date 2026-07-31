"""
lidar.launch.py — Launch RPLidar A2M12 driver node.

Publishes:
  /scan  (sensor_msgs/LaserScan)  — 360° scan at ~10 Hz, 12 m range

The lidar is mounted on top of the chassis, centred, facing forward.
TF: base_link → laser_frame  (static, set in rover.urdf.xacro)

Device: /dev/ttyLIDAR (udev symlink for /dev/ttyUSBx)
  → add to /etc/udev/rules.d/99-esp32.rules:
     SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",
       SYMLINK+="ttyLIDAR", MODE="0666"
  SiLabs CP210x VCP: idVendor=10c4 idProduct=ea60
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyLIDAR",
        description="Serial port for the RPLidar",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="laser_frame",
        description="TF frame id published in the LaserScan header",
    )

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar",
        output="screen",
        parameters=[{
            "serial_port":     LaunchConfiguration("serial_port"),
            "serial_baudrate": 256000,   # A2M12 uses 256000, not 115200
            "frame_id":        LaunchConfiguration("frame_id"),
            "inverted":        False,
            "angle_compensate": True,
            "scan_mode":       "Standard",  # Standard mode: ~8000 samples/s; Boost overflows serial buffer on RPi
        }],
        remappings=[
            ("/scan", "/scan_raw"),
            ("scan", "/scan_raw"),
        ],
    )

    scan_restamper = Node(
        package="ecza_lidar",
        executable="scan_restamper.py",
        name="scan_restamper",
        output="screen",
        parameters=[{
            "input_topic": "/scan_raw",
            "output_topic": "/scan",
            "slam_output_topic": "/scan_slam",
            "frame_id": LaunchConfiguration("frame_id"),
            "max_publish_hz": 8.0,
            "angle_downsample": 2,
            "slam_publish_hz": 2.5,
            # 4 -> 2: lidar zaten tarama başına ~720 binin sadece ~%11'inde
            # geçerli ölçüm döndürüyor (~79 nokta). Bunun 4'te 3'ünü de atınca
            # slam_toolbox'a giden taramaların çoğu tamamen boş kalıyordu;
            # Karto boş taramayı reddettiği için pose graph'a hiç düğüm
            # eklenmiyor ve /map 0x0 olarak yayınlanıyordu.
            "slam_angle_downsample": 2,
            "max_slam_angular_z": 0.07,
            "odom_topic": "/odom",
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        scan_restamper,
    ])
