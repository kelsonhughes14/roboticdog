"""
imu_leveling_node.py
--------------------
Converts raw BNO085 Euler angles from /imu/euler into a filtered
body-level correction published on /level_correction.

The gait_node consumes /level_correction and passes it to the gait
generator, which applies a body-frame pitch/roll adjustment so the
body stays level over uneven surfaces or compensates for hardware
height offsets between front and rear legs.

Unlike the joystick /body_pose (intentional operator lean), this
correction is automatic and always active during STANDING/WALKING.

Subscribed topics:
  /imu/euler         (geometry_msgs/Vector3)  — roll, pitch, yaw in degrees (Teensy)

Published topics:
  /level_correction  (geometry_msgs/Vector3)  — filtered roll (x), pitch (y) in degrees

Parameters:
  alpha        (float, 0.10)  — EMA smoothing coefficient [0=frozen, 1=raw IMU]
  max_angle    (float, 10.0)  — clamp: max correction magnitude in degrees
  invert_roll  (bool, False)  — flip roll sign if IMU is mounted inverted laterally
  invert_pitch (bool, False)  — flip pitch sign if IMU is mounted front-to-back inverted
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class ImuLevelingNode(Node):

    def __init__(self):
        super().__init__('imu_leveling_node')

        self.declare_parameter('alpha',        0.10)
        self.declare_parameter('max_angle',   10.0)
        self.declare_parameter('invert_roll',  False)
        self.declare_parameter('invert_pitch', False)

        self._alpha        = float(self.get_parameter('alpha').value)
        self._max_angle    = float(self.get_parameter('max_angle').value)
        self._invert_roll  = bool(self.get_parameter('invert_roll').value)
        self._invert_pitch = bool(self.get_parameter('invert_pitch').value)

        # EMA state
        self._roll_f  = 0.0
        self._pitch_f = 0.0

        self.create_subscription(Vector3, 'imu/euler', self._imu_callback, 10)
        self._pub = self.create_publisher(Vector3, 'level_correction', 10)

        self.get_logger().info(
            f'IMU leveling active — '
            f'alpha={self._alpha:.2f}  max={self._max_angle:.1f}°  '
            f'invert_roll={self._invert_roll}  invert_pitch={self._invert_pitch}'
        )

    def _imu_callback(self, msg: Vector3):
        roll  = msg.x * (-1.0 if self._invert_roll  else 1.0)
        pitch = msg.y * (-1.0 if self._invert_pitch else 1.0)

        # Exponential moving average — smooths sensor noise without a separate timer
        a = self._alpha
        self._roll_f  = a * roll  + (1.0 - a) * self._roll_f
        self._pitch_f = a * pitch + (1.0 - a) * self._pitch_f

        out = Vector3()
        out.x = max(-self._max_angle, min(self._max_angle, self._roll_f))
        out.y = max(-self._max_angle, min(self._max_angle, self._pitch_f))
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuLevelingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
