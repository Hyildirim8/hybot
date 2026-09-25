"""Projedeki GERÇEK topic / action / frame / servis isimleri — tek kaynak.

Buradaki her isim 2026-09-25'te çalışan sistemden `ros2 topic list`,
`ros2 action list`, `ros2 service list` ve `ros2 topic type` ile doğrulandı.
Hiçbiri tahmin değildir. İsim değişirse YALNIZCA burayı güncelleyin.

Doğrulama komutu (container içinde):
    ros2 topic list && ros2 action list
"""

from dataclasses import dataclass


# ── Poz / odometri ────────────────────────────────────────────────────────────
# /odometry/filtered  : robot_localization EKF çıkışı, odom->base_link TF sahibi.
#                       2026-09-24'ten beri saf ölü hesaplama (tekerlek vx/vy +
#                       gyro yaw hızı); rf2o mutlak pozu EKF'ten çıkarıldı.
# /odom               : mecanum_drive_controller ham tekerlek odometrisi.
# /odom_rf2o          : lazer odometrisi (kovaryans adaptöründen geçmiş). EKF
#                       bunu ARTIK kullanmıyor; karşılaştırma için dinlenebilir.
TOPIC_ODOM_FILTERED = "/odometry/filtered"        # nav_msgs/msg/Odometry
TOPIC_ODOM_WHEEL = "/odom"                        # nav_msgs/msg/Odometry
TOPIC_ODOM_RF2O = "/odom_rf2o"                    # nav_msgs/msg/Odometry

# ── Harita / costmap ──────────────────────────────────────────────────────────
TOPIC_MAP = "/map"                                # nav_msgs/msg/OccupancyGrid
TOPIC_MAP_METADATA = "/map_metadata"              # nav_msgs/msg/MapMetaData
TOPIC_GLOBAL_COSTMAP = "/global_costmap/costmap"  # nav_msgs/msg/OccupancyGrid
TOPIC_LOCAL_COSTMAP = "/local_costmap/costmap"    # nav_msgs/msg/OccupancyGrid

# ── Plan / rota ───────────────────────────────────────────────────────────────
# /plan            : planner_server'ın ürettiği global yol (yeniden planlama
#                    tespiti bunun üzerinden yapılır).
# /local_plan      : controller_server (MPPI) yerel yörüngesi.
# /plan_smoothed   : smoother_server çıkışı.
TOPIC_GLOBAL_PLAN = "/plan"                       # nav_msgs/msg/Path
TOPIC_LOCAL_PLAN = "/local_plan"                  # nav_msgs/msg/Path
TOPIC_PLAN_SMOOTHED = "/plan_smoothed"            # nav_msgs/msg/Path

# ── Lidar ─────────────────────────────────────────────────────────────────────
# /scan_raw : rplidar sürücüsünün ham çıkışı (720 ışın).
# /scan     : scan_restamper süzgecinden geçmiş, Nav2/rf2o girdisi (360 ışın).
# /scan_slam: slam_toolbox girdisi.
TOPIC_SCAN = "/scan"                              # sensor_msgs/msg/LaserScan
TOPIC_SCAN_RAW = "/scan_raw"                      # sensor_msgs/msg/LaserScan
TOPIC_SCAN_SLAM = "/scan_slam"                    # sensor_msgs/msg/LaserScan

# ── Mod (TELEOP / AUTONOMOUS) ─────────────────────────────────────────────────
# teleop_node latched (TRANSIENT_LOCAL) olarak yayınlar: True = AUTONOMOUS.
# Mod geçişini TETİKLEMEK için ayrı bir servis YOKTUR; operatör joystick'te
# Start'a basar veya /goal_pose'a hedef gelir (teleop_node bunu AUTO'ya geçiş
# olarak yorumlar). Bu yüzden mode-switch testi operatör tetiklemeli çalışır.
TOPIC_AUTONOMOUS_MODE = "/autonomous_mode"        # std_msgs/msg/Bool  (latched)
TOPIC_SLAM_MANAGER_STATUS = "/slam_manager/status"    # std_msgs/msg/String
TOPIC_SLAM_MANAGER_EXPLORING = "/slam_manager/exploring"  # std_msgs/msg/Bool

# ── Kamera ────────────────────────────────────────────────────────────────────
# Bu projede kamera SADECE sıkıştırılmış olarak yayınlanır; ham Image topic'i
# yoktur. rpicam_node -> CompressedImage.
TOPIC_CAMERA_COMPRESSED = "/camera_csi/image_raw/compressed"  # sensor_msgs/msg/CompressedImage

# ── Hız komutları (SALT OKUMA — benchmark bunlara yayın YAPMAZ) ───────────────
# /cmd_vel                                 : teşhis/bag için ayna.
# /cmd_vel_nav                             : Nav2 controller_server çıkışı.
# /cmd_vel_nav_smoothed                    : velocity_smoother çıkışı; teleop_node
#                                            AUTO modda bunu okur.
# /controller_manager/reference_unstamped  : mecanum kontrolcüye giden SON komut.
TOPIC_CMD_VEL = "/cmd_vel"                                        # geometry_msgs/msg/Twist
TOPIC_CMD_VEL_NAV = "/cmd_vel_nav"                                # geometry_msgs/msg/Twist
TOPIC_CMD_VEL_NAV_SMOOTHED = "/cmd_vel_nav_smoothed"              # geometry_msgs/msg/Twist
TOPIC_CONTROLLER_REFERENCE = "/controller_manager/reference_unstamped"  # geometry_msgs/msg/Twist

# ── Tekerlek geri beslemesi ───────────────────────────────────────────────────
# Duruş tespiti için ŞART: gyro/poz ölçümü alırken robotun gerçekten durduğunu
# yalnızca bu doğrular. (2026-09-24 dersi: /autonomous_mode false okusa bile
# robot hareket edebiliyor.)
TOPIC_WHEEL_VELOCITIES = "/wheel_velocities"              # std_msgs/msg/Float32MultiArray
TOPIC_WHEEL_VELOCITIES_CMD = "/wheel_velocities_cmd_f32"  # std_msgs/msg/Float32MultiArray

# ── IMU ───────────────────────────────────────────────────────────────────────
TOPIC_IMU = "/imu/data_raw"                       # sensor_msgs/msg/Imu

# ── Hedef / başlangıç pozu ────────────────────────────────────────────────────
TOPIC_GOAL_POSE = "/goal_pose"                    # geometry_msgs/msg/PoseStamped
TOPIC_INITIAL_POSE = "/initialpose"               # geometry_msgs/msg/PoseWithCovarianceStamped

# ── Nav2 davranış ağacı / teşhis ──────────────────────────────────────────────
# BehaviorTreeLog yeniden planlama ve recovery tespiti için en güvenilir kaynak:
# ComputePathToPose / ClearEntireCostmap / BackUp düğümlerinin durum geçişleri.
TOPIC_BEHAVIOR_TREE_LOG = "/behavior_tree_log"     # nav2_msgs/msg/BehaviorTreeLog
TOPIC_DIAGNOSTICS = "/diagnostics"                 # diagnostic_msgs/msg/DiagnosticArray
TOPIC_FIRMWARE_STATUS = "/firmware_status"          # std_msgs/msg/String

# ── Nav2 action'ları ──────────────────────────────────────────────────────────
ACTION_NAVIGATE_TO_POSE = "/navigate_to_pose"              # nav2_msgs/action/NavigateToPose
ACTION_NAVIGATE_THROUGH_POSES = "/navigate_through_poses"  # nav2_msgs/action/NavigateThroughPoses
ACTION_COMPUTE_PATH_TO_POSE = "/compute_path_to_pose"      # nav2_msgs/action/ComputePathToPose
ACTION_FOLLOW_PATH = "/follow_path"                        # nav2_msgs/action/FollowPath
ACTION_BACKUP = "/backup"                                  # nav2_msgs/action/BackUp

# ── Servisler ─────────────────────────────────────────────────────────────────
SERVICE_MAP_SAVER = "/map_saver/save_map"
SERVICE_SLAM_DYNAMIC_MAP = "/slam_toolbox/dynamic_map"
SERVICE_CLEAR_LOCAL_COSTMAP = "/local_costmap/clear_entirely_local_costmap"
SERVICE_CLEAR_GLOBAL_COSTMAP = "/global_costmap/clear_entirely_global_costmap"

# ── TF frame'leri ─────────────────────────────────────────────────────────────
# map -> odom     : slam_toolbox yayınlar (harita düzeltmesi)
# odom -> base_link: robot_localization EKF yayınlar
# base_link -> laser_frame: statik, yaw = 180° (lidar gövdeye ters monte)
FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE_LINK = "base_link"
FRAME_LASER = "laser_frame"
FRAME_IMU = "imu_link"

# ── Batarya ───────────────────────────────────────────────────────────────────
# Bu robotta batarya ölçüm donanımı ve topic'i YOKTUR (kod tabanında
# battery/voltage/vbat geçen hiçbir yayıncı bulunmuyor — 2026-09-25'te arandı).
# Bu yüzden batarya testi TAMAMEN operatör girişine dayanır ve kalan çalışma
# süresi TAHMİN EDİLMEZ (kapasite bilinmiyor).
TOPIC_BATTERY = None

# ── Mecanum geometrisi (config/rover_params.yaml + firmware/main/motor.h) ─────
WHEEL_RADIUS_M = 0.04
WHEEL_BASE_M = 0.38            # lx*2
WHEEL_SEPARATION_WIDTH_M = 0.26  # ly*2
FIRMWARE_MAX_WHEEL_RAD_S = 18.75

# Tekerlek duruş eşiği: dört tekerleğin |hız| toplamı bunun altındaysa robot
# gerçekten duruyor kabul edilir.
WHEEL_STILL_SUM_RAD_S = 0.05


@dataclass(frozen=True)
class MotionDirection:
    """Mecanum hareket yönü tanımı (mecanum testi için)."""

    key: str
    label_tr: str
    # Gövde çerçevesinde beklenen baskın bileşen:
    #   "ileri" (+x), "geri" (-x), "sol" (+y), "sag" (-y), "ccw" (+wz), "cw" (-wz)
    expect: str


# REP-103 gövde çerçevesi: +x ileri, +y SOL, +z yukarı, +wz saat yönünün TERSİ.
MECANUM_DIRECTIONS = (
    MotionDirection("ileri", "İleri (+x)", "ileri"),
    MotionDirection("geri", "Geri (-x)", "geri"),
    MotionDirection("sol", "Sola yanal (+y)", "sol"),
    MotionDirection("sag", "Sağa yanal (-y)", "sag"),
    MotionDirection("ccw", "Saat yönünün tersi (+wz)", "ccw"),
    MotionDirection("cw", "Saat yönü (-wz)", "cw"),
)


def max_wheel_rad_s(vx: float, vy: float, wz: float) -> float:
    """Mecanum ters kinematiğinde en yüksek teker hızı (rad/s).

    Firmware MAX_SPEED_RAD_S=18.75'i aşan komutlar kırpılır ve hareket bozulur;
    mecanum testinde operatöre uyarı vermek için kullanılır.
    """
    lx = WHEEL_BASE_M / 2.0
    ly = WHEEL_SEPARATION_WIDTH_M / 2.0
    k = lx + ly
    speeds = (
        (vx - vy - k * wz),
        (vx + vy + k * wz),
        (vx + vy - k * wz),
        (vx - vy + k * wz),
    )
    return max(abs(s) for s in speeds) / WHEEL_RADIUS_M
