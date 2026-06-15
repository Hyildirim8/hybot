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
            "slam_publish_hz": 5.0,
            "slam_angle_downsample": 2,
            # Dönüş sırasında SLAM'i aç bırak: scan matcher dönüş hatasını ancak
            # tarama görürse düzeltebilir. 0.07 tüm dönüşleri susturuyordu →
            # dönüş kayması odometriden birikip duvarları çiftliyordu.
            # 0.65: otonom dönüşler (≤0.45 rad/s) ve yavaş manuel dönüşler geçer; çok hızlı spin (1.8) gate'lenir.
            # 0.50 düşüktü — manuel çevirme sırasında SLAM hiç tarama alamıyordu → dönüş sonrası büyük açısal boşluk.
            "max_slam_angular_z": 0.65,
            "odom_topic": "/odom",
            # A2M12 aralıklı olarak neredeyse boş tarama üretiyor (720'de 2-13
            # geçerli ışın). Bunlar SLAM/costmap/keşfi zehirliyor — at.
            "min_valid_fraction": 0.10,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        scan_restamper,
    ])
