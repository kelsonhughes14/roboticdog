"""
controller_node.py
------------------
Watchdog node — re-publishes /joy_raw as /joy.
If the Xbox controller disconnects, publishes zeroed Joy messages
so the robot stops safely instead of holding its last command.

Subscribed topics:
  /joy_raw   (sensor_msgs/Joy)

Published topics:
  /joy       (sensor_msgs/Joy)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

WATCHDOG_TIMEOUT = 1.0   # seconds


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.last_msg_time = time.time()
        self.last_joy      = None

        self.create_subscription(Joy, 'joy_raw', self._joy_callback, 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.create_timer(0.05, self._watchdog_timer)

        self.get_logger().info(
            'Controller node ready. '
            'Listening on /joy_raw, publishing to /joy.'
        )

    def _joy_callback(self, msg: Joy):
        self.last_msg_time = time.time()
        self.last_joy = msg
        self.joy_pub.publish(msg)

    def _watchdog_timer(self):
        if time.time() - self.last_msg_time > WATCHDOG_TIMEOUT:
            if self.last_joy is not None:
                self.get_logger().warn('Controller timeout — publishing zeros')
                self.last_joy = None
            zero = Joy()
            zero.header.stamp = self.get_clock().now().to_msg()
            zero.axes    = [0.0] * 8
            zero.buttons = [0]   * 11
            self.joy_pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
