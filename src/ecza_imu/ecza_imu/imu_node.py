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
  i2c_bus                  int    1          /dev/i2c-N to open
  frame_id                 str   'imu_link'
  rate_hz                  float  50.0      publish rate
  gyro_calibration_samples int    200       gyro bias samples averaged at
                                             startup (~100 Hz); robot must be
                                             stationary during this window
  gyro_still_band          float  0.18      rad/s — max-min spread of the
                                             RAW reading over the last ~0.4s
                                             must stay under this on all 3
                                             axes to count as "not rotating"
                                             (tests signal stability, not
                                             its absolute value — see
                                             _track_bias). 2026-08-03: raised
                                             from 0.03 — live raw spread while
                                             genuinely stationary measured
                                             ~0.13 rad/s (I2C bus noise, see
                                             "lost arbitration" errors on
                                             i2c-1), so 0.03 never passed and
                                             the tracker could never lock.
  gyro_still_duration_s    float  2.0       how long that stability must
                                             hold before the bias tracker
                                             starts nudging
  gyro_bias_adapt_rate     float  0.005     EMA weight applied per sample
                                             while continuously tracking bias
  gyro_still_abs_max       float  0.6       rad/s — in addition to the spread
                                             test, the RAW reading's mean over
                                             the window must stay under this
                                             on all 3 axes. Spread alone can't
                                             tell "not moving" apart from
                                             "rotating at a constant rate", and
                                             a motor-driven in-place turn holds
                                             a far steadier rate than a hand
                                             ever would — see _track_bias. Kept
                                             well above the sensor's own
                                             resting bias, which is NOT close
                                             to zero and drifts a lot between
                                             boots: ~-0.14 rad/s measured
                                             2026-08-01, ~-0.36 rad/s measured
                                             2026-08-11 on the same Z axis (see
                                             imu-ekf-integration memory, "gyro
                                             bias drift recurs"). A tight bound
                                             here would reject genuine
                                             stillness and reintroduce the
                                             relock-lockout bug fixed in
                                             _track_bias's history.
"""

import struct
from collections import deque
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
# Floor for the per-boot measured angular_velocity_covariance (see
# _calibrate_gyro). i2c-1 on this board intermittently throws "lost
# arbitration" errors (see gyro_still_band docstring), which showed up as a
# genuinely-stationary raw spread of ~0.13 rad/s peak-to-peak over 0.4s — but
# that noise is bursty, not constant, so a single ~2s calibration window can
# land in a quiet stretch and report a much tighter variance than reality.
# Treating the noise as roughly uniform over that peak-to-peak range:
# var ≈ range²/12 ≈ 0.13²/12. Floors against known bus noise instead of
# trusting whatever a possibly-quiet calibration window happened to measure.
GYRO_VAR_FLOOR  = 0.13 ** 2 / 12.0   # ≈ 1.4e-3 (rad/s)^2

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
        self.declare_parameter('gyro_calibration_samples', 200)
        self.declare_parameter('gyro_still_band', 0.18)          # rad/s
        self.declare_parameter('gyro_still_duration_s', 2.0)     # s
        self.declare_parameter('gyro_bias_adapt_rate', 0.005)    # EMA weight/sample
        self.declare_parameter('gyro_still_abs_max', 0.6)        # rad/s

        bus_num     = self.get_parameter('i2c_bus').value
        self.frame  = self.get_parameter('frame_id').value
        rate_hz     = self.get_parameter('rate_hz').value
        cal_samples = self.get_parameter('gyro_calibration_samples').value
        self._still_band = float(self.get_parameter('gyro_still_band').value)
        self._still_abs_max = float(self.get_parameter('gyro_still_abs_max').value)
        self._bias_adapt_rate = float(self.get_parameter('gyro_bias_adapt_rate').value)
        still_duration_s = float(self.get_parameter('gyro_still_duration_s').value)
        self._still_required = max(1, int(still_duration_s * rate_hz))
        self._still_count = 0
        # Short rolling window used only to test raw-signal stability
        # (spread), independent of still_duration_s — see _track_bias.
        self._raw_window = deque(maxlen=max(2, int(0.4 * rate_hz)))

        self.bus = smbus.SMBus(bus_num)
        self._init_sensors()
        self._gyro_bias = self._calibrate_gyro(cal_samples)
        # If startup calibration itself lands close to gyro_still_abs_max,
        # the raw reading at genuine rest will fail _track_bias's abs gate
        # forever (raw ≈ bias even when truly still) — the tracker can never
        # adapt again. Warn loudly instead of failing silently; see
        # gyro_still_abs_max docstring for why this drifts between boots.
        if any(abs(b) > 0.8 * self._still_abs_max for b in self._gyro_bias):
            self.get_logger().warn(
                f'gyro calibration bias {self._gyro_bias} is close to or past '
                f'gyro_still_abs_max ({self._still_abs_max} rad/s) — the bias '
                'tracker may never be able to re-adapt. Consider raising '
                'gyro_still_abs_max.'
            )

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

    def _calibrate_gyro(self, num_samples: int):
        # ITG3205 has a fixed (non-zero) rate output at rest — this GY-85
        # unit measured ~-0.14 rad/s (~-8°/s) on its Z axis. Uncorrected,
        # that integrates into several full turns of fake yaw drift within
        # a couple of minutes once nothing else (e.g. wheel-encoder yaw) is
        # bounding it — see imu-ekf-integration memory, 2026-08-01. Assumes
        # the robot is stationary at boot, same convention as most cheap
        # MEMS gyro drivers.
        self.get_logger().info(
            f'calibrating gyro bias ({num_samples} samples, ~{num_samples/100.0:.1f}s, '
            'keep the robot still)...'
        )
        samples = []
        for _ in range(num_samples):
            try:
                samples.append(self._read_gyro_raw())
            except Exception as e:
                self.get_logger().warn(f'gyro calibration read error: {e}')
            time.sleep(0.01)  # ITG3205 configured for 100 Hz ODR
        n = len(samples)
        if n > 0:
            bias = tuple(sum(s[axis] for s in samples) / n for axis in range(3))
            # Measured variance, not a guess — this feeds angular_velocity_covariance
            # below instead of the old hardcoded 1e-4. A wrong LOW covariance is the
            # dangerous direction: it tells the EKF to over-trust this axis. Floored
            # at GYRO_VAR_FLOOR (derived from already-documented i2c-1 bus noise,
            # not this window's sample) because that noise is bursty and a quiet
            # ~2s calibration window can otherwise under-report it.
            var = tuple(
                max(GYRO_VAR_FLOOR, sum((s[axis] - bias[axis]) ** 2 for s in samples) / n)
                for axis in range(3)
            )
        else:
            bias = (0.0, 0.0, 0.0)
            var = (GYRO_VAR_FLOOR,) * 3  # no samples at all — fall back to the noise floor
        self._gyro_var = var
        self.get_logger().info(
            f'gyro bias measured: x={bias[0]:+.4f} y={bias[1]:+.4f} z={bias[2]:+.4f} rad/s '
            f'({n}/{num_samples} samples)'
        )
        self.get_logger().info(
            f'gyro variance measured: x={var[0]:.2e} y={var[1]:.2e} z={var[2]:.2e} (rad/s)^2 '
            '— feeding angular_velocity_covariance'
        )
        return bias

    # ── raw reads ───────────────────────────────────────────────────────────────

    def _read_accel(self):
        raw = self.bus.read_i2c_block_data(ADXL_ADDR, ADXL_DATAX0, 6)
        x, y, z = struct.unpack('<3h', bytes(raw))           # little-endian signed
        return x * ADXL_SCALE, y * ADXL_SCALE, z * ADXL_SCALE

    def _read_gyro_raw(self):
        raw = self.bus.read_i2c_block_data(ITG_ADDR, ITG_GYRO_XOUT_H, 6)
        x, y, z = struct.unpack('>3h', bytes(raw))           # big-endian signed
        return x * ITG_SCALE, y * ITG_SCALE, z * ITG_SCALE

    def _read_gyro(self):
        x, y, z = self._read_gyro_raw()
        bx, by, bz = self._gyro_bias
        cx, cy, cz = x - bx, y - by, z - bz
        self._track_bias((x, y, z), (cx, cy, cz))
        return cx, cy, cz

    def _track_bias(self, raw, corrected) -> None:
        # This ITG3205 doesn't hold a fixed bias — measured a shift of
        # ~0.1 rad/s within seconds of the one-time startup calibration
        # (see imu-ekf-integration memory, 2026-08-01), which is enough to
        # integrate into several fake full turns of yaw drift within a
        # couple of minutes.
        #
        # 2026-08-01 v2: the first version of this gated adaptation on the
        # CORRECTED reading staying near zero. That's circular — once the
        # true bias has drifted far enough that the corrected reading no
        # longer looks "still", the tracker can never re-lock, because its
        # own stillness test depends on the bias estimate it's trying to
        # fix. Confirmed live: bias drifted back to ~-0.12 rad/s and stayed
        # there, un-adapted, for the rest of the session.
        #
        # Fixed by testing "stillness" on the RAW signal's short-term
        # SPREAD (max-min over a small rolling window) instead of its
        # absolute value — a call this can't get wrong regardless of how
        # stale the current bias estimate is: a raw reading that isn't
        # *changing* means angular velocity isn't changing, which is true
        # whether the current bias guess is 0 or wildly off. A real hand
        # rotation's onset (acceleration) breaks this immediately; a sustained
        # hand turn at a genuinely constant rate could in theory slip through,
        # but that's not how hand-turns actually happen in practice.
        #
        # 2026-08-11: that residual risk turned out to be real for a
        # MOTOR-driven in-place turn, which holds a far steadier rate than a
        # hand ever would (SLAM/Nav2 spin, joystick pivot). Added a second
        # gate on the raw mean's absolute magnitude (gyro_still_abs_max) to
        # catch that case. It deliberately stays far from zero — this
        # sensor's genuine at-rest raw output isn't close to zero and drifts
        # a lot between boots (~-0.14 rad/s one boot, ~-0.36 rad/s another —
        # see gyro_still_abs_max docstring) — a tight bound here reintroduces
        # the exact relock-lockout bug fixed above, just via a different gate.
        self._raw_window.append(raw)
        if len(self._raw_window) < self._raw_window.maxlen:
            return

        spreads = [
            max(w[axis] for w in self._raw_window) - min(w[axis] for w in self._raw_window)
            for axis in range(3)
        ]
        rx = sum(w[0] for w in self._raw_window) / len(self._raw_window)
        ry = sum(w[1] for w in self._raw_window) / len(self._raw_window)
        rz = sum(w[2] for w in self._raw_window) / len(self._raw_window)
        if (all(s < self._still_band for s in spreads)
                and all(abs(m) < self._still_abs_max for m in (rx, ry, rz))):
            self._still_count += 1
            if self._still_count >= self._still_required:
                bx, by, bz = self._gyro_bias
                a = self._bias_adapt_rate
                self._gyro_bias = (
                    bx + a * (rx - bx),
                    by + a * (ry - by),
                    bz + a * (rz - bz),
                )
        else:
            self._still_count = 0

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
            # Measured at startup calibration (_calibrate_gyro), not a guess —
            # this is what the EKF's trust in vyaw (angular_velocity_covariance
            # for /imu/data_raw, see config/ekf.yaml) is actually keyed off of.
            msg.angular_velocity_covariance[0]    = self._gyro_var[0]
            msg.angular_velocity_covariance[4]    = self._gyro_var[1]
            msg.angular_velocity_covariance[8]    = self._gyro_var[2]
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
