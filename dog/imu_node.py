"""
imu_node.py
-----------
Optional Jetson-side IMU node for a BNO085 9-DOF connected directly to
the Jetson's I2C bus.

NOTE: In the normal SASSA robot configuration the BNO085 is wired to the
Teensy 4.1 (Wire1, SDA=17/SCL=16).  The Teensy publishes /imu/data and
/imu/euler via micro-ROS, so this node is NOT started by dog_launch.py.

Run this node only if you have a BNO085 wired directly to the Jetson:
  ros2 run dog imu_node

The BNO085 runs on-chip ARVR-stabilised sensor fusion and outputs
quaternion orientation directly — no complementary filter needed.

Requires:
  pip install adafruit-circuitpython-bno08x adafruit-blinka

Published topics:
  /imu/data   (sensor_msgs/Imu)       — quaternion + accel + gyro
  /imu/euler  (geometry_msgs/Vector3) — roll, pitch, yaw in degrees (debug)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from dog.robot_config import BNO085_ADDRESS, IMU_PUBLISH_RATE

try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    BNO085_AVAILABLE = True
except ImportError:
    BNO085_AVAILABLE = False


def _quat_to_euler(qw, qx, qy, qz):
    """Return (roll, pitch, yaw) in degrees from a unit quaternion."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


# BNO085 ARVR rotation vector: ~1° RMS → variance ≈ (0.017 rad)² ≈ 3e-4
_ORIENT_VAR = 3e-4
_GYRO_VAR   = 1e-4   # datasheet noise density integrated at 50 Hz
_ACCEL_VAR  = 8e-3   # m²/s⁴ typical


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('dry_run', not BNO085_AVAILABLE)
        self.dry_run = self.get_parameter('dry_run').value

        self.imu_pub   = self.create_publisher(Imu,     'imu/data',  10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        self.sensor    = None

        if not self.dry_run:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = BNO08X_I2C(i2c, address=BNO085_ADDRESS)
                self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
                self.get_logger().info(
                    f'BNO085 connected at I2C 0x{BNO085_ADDRESS:02X}'
                )
            except Exception as e:
                self.get_logger().error(f'BNO085 init failed: {e}')
                self.dry_run = True

        if self.dry_run:
            self.get_logger().warn('IMU running in simulation mode.')

        self.create_timer(1.0 / IMU_PUBLISH_RATE, self._publish_imu)

    def _publish_imu(self):
        now = self.get_clock().now().to_msg()

        if self.dry_run or self.sensor is None:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)
            self.euler_pub.publish(Vector3())
            return

        try:
            # adafruit_bno08x returns (i, j, k, real) = (x, y, z, w)
            qx, qy, qz, qw = self.sensor.quaternion
            ax, ay, az      = self.sensor.acceleration
            gx, gy, gz      = self.sensor.gyro
        except Exception as e:
            self.get_logger().error(f'IMU read: {e}')
            return

        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = 'imu_link'

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance = [
            _ORIENT_VAR, 0.0, 0.0,
            0.0, _ORIENT_VAR, 0.0,
            0.0, 0.0, _ORIENT_VAR,
        ]

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance = [
            _GYRO_VAR, 0.0, 0.0,
            0.0, _GYRO_VAR, 0.0,
            0.0, 0.0, _GYRO_VAR,
        ]

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance = [
            _ACCEL_VAR, 0.0, 0.0,
            0.0, _ACCEL_VAR, 0.0,
            0.0, 0.0, _ACCEL_VAR,
        ]

        self.imu_pub.publish(imu)

        roll, pitch, yaw = _quat_to_euler(qw, qx, qy, qz)
        euler = Vector3()
        euler.x = roll
        euler.y = pitch
        euler.z = yaw
        self.euler_pub.publish(euler)


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
