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
            # 4 -> 2 -> 1. Downsampling is plain stride slicing
            # (ranges[::n] in scan_restamper), so it throws away that
            # fraction of the *valid* points too, and this lidar has none to
            # spare. Measured 2026-08-11: it spins at 12.7 Hz, not the 10 Hz
            # it reports, so at 4 kHz there are only ~315 samples for the 720
            # angle_compensate bins, and only ~100 of those come back with a
            # real return (~14% of the scan). Halving that again left SLAM
            # matching against ~50 points per scan, which is why the map
            # smeared into a radial fan instead of closing loops.
            "slam_angle_downsample": 1,
            # 0.07 -> 0.65: bu kadar düşük bir eşik robot ~4°/sn'den hızlı
            # döndüğü an SLAM'a scan gitmesini tamamen kesiyordu — dönüş
            # boyunca SLAM kör kalıyor, sadece dead-reckoning ile ilerliyor,
            # dönüş bitince biriken hata haritaya tek seferde işleniyor ve
            # düzeltilmiyordu ("harita kayması"). 0.65 daha önce doğrulanmış
            # çalışan değer.
            "max_slam_angular_z": 0.65,
            "odom_topic": "/odom",
            # Whole rotations carry 150-400 valid points, fragments carry
            # 0-25, and almost nothing lands in between — so 80 cleanly
            # separates them with room to spare on both sides. About 38% of
            # incoming scans pass, which is still ~5-6 whole scans/s, well
            # above the 2.5 Hz SLAM path and 8 Hz costmap path below.
            "min_valid_points": 80,
            "slam_min_valid_points": 80,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        scan_restamper,
    ])
