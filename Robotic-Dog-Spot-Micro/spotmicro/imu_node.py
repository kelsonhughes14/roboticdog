"""
imu_node.py
-----------
ROS2 node that reads the MPU-6050 IMU sensor and publishes:

  /imu/data   (sensor_msgs/Imu)
    Angular velocity (gyro) and linear acceleration (accel).

  /imu/euler  (geometry_msgs/Vector3)
    Estimated roll, pitch, yaw in degrees (complementary filter).

The MPU-6050 is connected via I2C at address 0x68.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import time

from spotmicro.robot_config import MPU6050_ADDRESS, IMU_PUBLISH_RATE, I2C_BUS

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


# ── MPU-6050 Register Map ─────────────────────────────────────────────────────
_PWR_MGMT_1   = 0x6B
_SMPLRT_DIV   = 0x19
_CONFIG_REG   = 0x1A
_GYRO_CONFIG  = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B
_GYRO_XOUT_H  = 0x43
_TEMP_OUT_H   = 0x41

_ACCEL_SCALE  = 16384.0   # LSB/g  for ±2g range
_GYRO_SCALE   = 131.0     # LSB/(°/s) for ±250°/s range


class MPU6050:
    """Lightweight driver for the MPU-6050."""

    def __init__(self, bus_number: int = 1, address: int = 0x68):
        if not SMBUS_AVAILABLE:
            raise ImportError('smbus2 is not installed')
        self.bus  = smbus2.SMBus(bus_number)
        self.addr = address
        self._init_sensor()

    def _init_sensor(self):
        self.bus.write_byte_data(self.addr, _PWR_MGMT_1,  0x00)  # Wake up
        self.bus.write_byte_data(self.addr, _SMPLRT_DIV,  0x07)  # 1 kHz / 8
        self.bus.write_byte_data(self.addr, _CONFIG_REG,  0x00)
        self.bus.write_byte_data(self.addr, _GYRO_CONFIG, 0x00)  # ±250°/s
        self.bus.write_byte_data(self.addr, _ACCEL_CONFIG,0x00)  # ±2g

    def _read_word_2c(self, reg: int) -> int:
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val  = (high << 8) + low
        return val - 65536 if val >= 0x8000 else val

    def read_accel(self) -> tuple[float, float, float]:
        """Returns acceleration in m/s²."""
        ax = self._read_word_2c(_ACCEL_XOUT_H)     / _ACCEL_SCALE * 9.80665
        ay = self._read_word_2c(_ACCEL_XOUT_H + 2) / _ACCEL_SCALE * 9.80665
        az = self._read_word_2c(_ACCEL_XOUT_H + 4) / _ACCEL_SCALE * 9.80665
        return ax, ay, az

    def read_gyro(self) -> tuple[float, float, float]:
        """Returns angular velocity in rad/s."""
        gx = math.radians(self._read_word_2c(_GYRO_XOUT_H)     / _GYRO_SCALE)
        gy = math.radians(self._read_word_2c(_GYRO_XOUT_H + 2) / _GYRO_SCALE)
        gz = math.radians(self._read_word_2c(_GYRO_XOUT_H + 4) / _GYRO_SCALE)
        return gx, gy, gz

    def read_temperature(self) -> float:
        """Returns temperature in °C."""
        raw = self._read_word_2c(_TEMP_OUT_H)
        return raw / 340.0 + 36.53


class ComplementaryFilter:
    """
    Simple complementary filter to estimate roll and pitch
    from accelerometer + gyroscope data.

    α = 0.98 weights the gyro heavily (high frequency)
    and corrects drift using the accel (low frequency).
    """

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.roll  = 0.0
        self.pitch = 0.0
        self.last_time = time.time()

    def update(self, ax: float, ay: float, az: float,
               gx: float, gy: float, gz: float) -> tuple[float, float]:
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        # Accel-based roll/pitch estimate
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        # Integrate gyro
        self.roll  = self.alpha * (self.roll  + math.degrees(gx) * dt) \
                     + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + math.degrees(gy) * dt) \
                     + (1.0 - self.alpha) * accel_pitch

        return self.roll, self.pitch


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('dry_run', not SMBUS_AVAILABLE)
        self.dry_run = self.get_parameter('dry_run').value

        self.imu_pub   = self.create_publisher(Imu,     'imu/data',  10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)

        self.filter = ComplementaryFilter(alpha=0.98)
        self.sensor = None

        if not self.dry_run:
            try:
                self.sensor = MPU6050(bus_number=I2C_BUS,
                                      address=MPU6050_ADDRESS)
                self.get_logger().info(
                    f'MPU-6050 connected at I2C address '
                    f'0x{MPU6050_ADDRESS:02X}'
                )
            except Exception as e:
                self.get_logger().error(f'MPU-6050 init failed: {e}')
                self.dry_run = True

        if self.dry_run:
            self.get_logger().warn('IMU node running in simulation mode.')

        period = 1.0 / IMU_PUBLISH_RATE
        self.timer = self.create_timer(period, self._publish_imu)

    # ────────────────────────────────────────────────────────────────
    def _publish_imu(self):
        now = self.get_clock().now().to_msg()

        if self.dry_run or self.sensor is None:
            # Publish zeroed data in dry-run mode
            self._publish_zeroed(now)
            return

        try:
            ax, ay, az = self.sensor.read_accel()
            gx, gy, gz = self.sensor.read_gyro()
        except Exception as e:
            self.get_logger().error(f'IMU read error: {e}')
            return

        roll, pitch = self.filter.update(ax, ay, az, gx, gy, gz)

        # ── Imu message ──────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # Covariance — unknown, set diagonal to small value
        cov = [0.0] * 9
        cov[0] = cov[4] = cov[8] = 0.01
        imu_msg.angular_velocity_covariance    = cov
        imu_msg.linear_acceleration_covariance = cov
        imu_msg.orientation_covariance[0]      = -1.0   # orientation not set

        self.imu_pub.publish(imu_msg)

        # ── Euler angles ─────────────────────────────────────────────
        euler_msg = Vector3()
        euler_msg.x = roll    # degrees
        euler_msg.y = pitch   # degrees
        euler_msg.z = 0.0     # yaw not estimated without magnetometer
        self.euler_pub.publish(euler_msg)

    # ────────────────────────────────────────────────────────────────
    def _publish_zeroed(self, stamp):
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        euler_msg = Vector3()
        self.euler_pub.publish(euler_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
