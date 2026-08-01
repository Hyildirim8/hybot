#!/usr/bin/env python3
"""GY-85 IMU node for ecza-robotu.

Chips on the GY-85 module (all on I2C bus 1):
  ADXL345   0x53  ±2g  3-axis accelerometer  → m/s²
  ITG3205   0x68  ±2000°/s  3-axis gyroscope  → rad/s  (AD0=GND on this board)
  HMC5883L  0x1E  ~0.92 mGauss/LSB magnetometer → Tesla

Publishes:
  /imu/data_raw  sensor_msgs/Imu          (accel + gyro, no orientation estimate)
  /imu/mag       sensor_msgs/MagneticField

Parameters:
  i2c_bus    int    1        /dev/i2c-N to open
  frame_id   str   'imu_link'
  rate_hz    float  50.0    publish rate
"""

import struct
import time
import smbus                                 # python3-smbus (i2c-tools)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

# ── I2C addresses ──────────────────────────────────────────────────────────────
ADXL_ADDR  = 0x53
# ITG3205 address depends on the AD0 pin strap: 0x68 (AD0=GND) or 0x69 (AD0=VCC).
# This GY-85 board ties AD0 to GND — confirmed via `i2cdetect -y 1` (0x68 present,
# 0x69 absent).
ITG_ADDR   = 0x68
HMC_ADDR   = 0x1E

# ── ADXL345 register map ───────────────────────────────────────────────────────
ADXL_BW_RATE    = 0x2C   # output data rate
ADXL_POWER_CTL  = 0x2D   # power control (bit 3 = measure)
ADXL_DATA_FMT   = 0x31   # data format
ADXL_DATAX0     = 0x32   # first of 6 consecutive data bytes (little-endian)
# ±2g, 10-bit → 2/512 = 0.00390625 g/LSB
ADXL_SCALE      = 0.00390625 * 9.80665      # m/s² per LSB

# ── ITG3205 register map ───────────────────────────────────────────────────────
ITG_SMPLRT_DIV  = 0x15   # sample rate divider
ITG_DLPF_FS     = 0x16   # FS_SEL | DLPF config
ITG_INT_CFG     = 0x17   # interrupt config
ITG_GYRO_XOUT_H = 0x1D   # first of 6 data bytes (big-endian, XH XL YH YL ZH ZL)
# FS_SEL=3 → ±2000°/s, sensitivity = 14.375 LSB/(°/s)
ITG_SCALE       = (1.0 / 14.375) * (3.14159265358979 / 180.0)  # rad/s per LSB

# ── HMC5883L register map ──────────────────────────────────────────────────────
HMC_CFG_A  = 0x00   # Config A: samples/avg, data-rate, measurement mode
HMC_CFG_B  = 0x01   # Config B: gain
HMC_MODE   = 0x02   # operating mode
HMC_DATA   = 0x03   # first of 6 bytes: XH XL ZH ZL YH YL  ← note Z before Y
# Gain = 1090 LSB/Gauss, 1 Gauss = 1e-4 T
HMC_SCALE  = (1.0 / 1090.0) * 1e-4         # Tesla per LSB


class GY85Node(Node):

    def __init__(self):
        super().__init__('gy85_imu')

        self.declare_parameter('i2c_bus',    1)
        self.declare_parameter('frame_id',   'imu_link')
        self.declare_parameter('rate_hz',    50.0)

        bus_num     = self.get_parameter('i2c_bus').value
        self.frame  = self.get_parameter('frame_id').value
        rate_hz     = self.get_parameter('rate_hz').value

        self.bus = smbus.SMBus(bus_num)
        self._init_sensors()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub_imu = self.create_publisher(Imu,           'imu/data_raw', qos)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag',      qos)

        self.create_timer(1.0 / rate_hz, self._cb)
        self.get_logger().info(
            f'GY-85 on /dev/i2c-{bus_num} | frame={self.frame} | {rate_hz:.0f} Hz')

    # ── sensor init ─────────────────────────────────────────────────────────────

    def _init_sensors(self):
        # ADXL345: 100 Hz ODR, ±2g, measurement mode
        self.bus.write_byte_data(ADXL_ADDR, ADXL_BW_RATE,   0x0A)  # 100 Hz
        self.bus.write_byte_data(ADXL_ADDR, ADXL_DATA_FMT,  0x00)  # ±2g right-just.
        self.bus.write_byte_data(ADXL_ADDR, ADXL_POWER_CTL, 0x08)  # measure on

        # ITG3205: 100 Hz sample rate, FS=±2000°/s, 42 Hz low-pass
        # DLPF_FS: bits[4:3]=FS_SEL=3, bits[2:0]=DLPF_CFG=3 (42 Hz BW)
        self.bus.write_byte_data(ITG_ADDR, ITG_SMPLRT_DIV, 0x09)  # 1kHz/(9+1)=100 Hz
        self.bus.write_byte_data(ITG_ADDR, ITG_DLPF_FS,   0x1B)  # FS=3 | DLPF=3
        self.bus.write_byte_data(ITG_ADDR, ITG_INT_CFG,   0x00)

        # HMC5883L: 8 samples avg, 15 Hz, normal; gain 1090; continuous mode
        self.bus.write_byte_data(HMC_ADDR, HMC_CFG_A, 0x70)  # 8 avg, 15 Hz
        self.bus.write_byte_data(HMC_ADDR, HMC_CFG_B, 0x20)  # gain 1090
        self.bus.write_byte_data(HMC_ADDR, HMC_MODE,  0x00)  # continuous
        time.sleep(0.1)                                       # allow first measurement

        self.get_logger().info('ADXL345 + ITG3205 + HMC5883L initialized')

    # ── raw reads ───────────────────────────────────────────────────────────────

    def _read_accel(self):
        raw = self.bus.read_i2c_block_data(ADXL_ADDR, ADXL_DATAX0, 6)
        x, y, z = struct.unpack('<3h', bytes(raw))           # little-endian signed
        return x * ADXL_SCALE, y * ADXL_SCALE, z * ADXL_SCALE

    def _read_gyro(self):
        raw = self.bus.read_i2c_block_data(ITG_ADDR, ITG_GYRO_XOUT_H, 6)
        x, y, z = struct.unpack('>3h', bytes(raw))           # big-endian signed
        return x * ITG_SCALE, y * ITG_SCALE, z * ITG_SCALE

    def _read_mag(self):
        raw = self.bus.read_i2c_block_data(HMC_ADDR, HMC_DATA, 6)
        # HMC5883L byte order: XH XL ZH ZL YH YL  (Z and Y are SWAPPED)
        x = struct.unpack('>h', bytes(raw[0:2]))[0]
        z = struct.unpack('>h', bytes(raw[2:4]))[0]
        y = struct.unpack('>h', bytes(raw[4:6]))[0]
        return x * HMC_SCALE, y * HMC_SCALE, z * HMC_SCALE

    # ── publish callback ─────────────────────────────────────────────────────────

    def _cb(self):
        stamp = self.get_clock().now().to_msg()
        hdr   = Header(stamp=stamp, frame_id=self.frame)

        # IMU (accel + gyro)
        try:
            ax, ay, az = self._read_accel()
            gx, gy, gz = self._read_gyro()

            msg = Imu()
            msg.header = hdr
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.angular_velocity.x    = gx
            msg.angular_velocity.y    = gy
            msg.angular_velocity.z    = gz
            # No orientation estimate from raw sensors
            msg.orientation_covariance[0]         = -1.0
            # Approximate covariances (tune after calibration)
            msg.linear_acceleration_covariance[0] = 0.01
            msg.linear_acceleration_covariance[4] = 0.01
            msg.linear_acceleration_covariance[8] = 0.01
            msg.angular_velocity_covariance[0]    = 1e-4
            msg.angular_velocity_covariance[4]    = 1e-4
            msg.angular_velocity_covariance[8]    = 1e-4
            self.pub_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'accel/gyro error: {e}', throttle_duration_sec=5.0)

        # Magnetometer
        try:
            mx, my, mz = self._read_mag()
            msg = MagneticField()
            msg.header = hdr
            msg.magnetic_field.x = mx
            msg.magnetic_field.y = my
            msg.magnetic_field.z = mz
            msg.magnetic_field_covariance[0] = 0.01
            msg.magnetic_field_covariance[4] = 0.01
            msg.magnetic_field_covariance[8] = 0.01
            self.pub_mag.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'mag error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GY85Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
