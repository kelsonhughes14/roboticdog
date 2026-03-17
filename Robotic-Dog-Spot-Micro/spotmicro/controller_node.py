"""
controller_node.py
------------------
ROS2 node that reads the Xbox One controller via the joy package
and re-publishes it on /joy.

This node acts as a watchdog — if the controller disconnects,
it publishes a zero Joy message and logs a warning.

In most cases you don't need this node running separately;
the joy_node from the joy package publishes directly to /joy.
This node is useful for:
  - Adding connection watchdog logic
  - Remapping buttons without editing YAML
  - Injecting test commands programmatically

Subscribed topics:
  /joy_raw   (sensor_msgs/Joy)   — raw output from joy_node

Published topics:
  /joy       (sensor_msgs/Joy)   — processed / watchdog-guarded output
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

WATCHDOG_TIMEOUT = 1.0   # seconds — if no message in this time, send zeros


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.last_msg_time = time.time()
        self.last_joy = None

        self.create_subscription(Joy, 'joy_raw', self._joy_callback, 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        self.create_timer(0.05, self._watchdog_timer)   # 20 Hz check

        self.get_logger().info(
            'Controller node ready. '
            'Listening on /joy_raw, publishing to /joy.'
        )

    # ────────────────────────────────────────────────────────────────
    def _joy_callback(self, msg: Joy):
        self.last_msg_time = time.time()
        self.last_joy = msg
        self.joy_pub.publish(msg)

    def _watchdog_timer(self):
        elapsed = time.time() - self.last_msg_time
        if elapsed > WATCHDOG_TIMEOUT:
            if self.last_joy is not None:
                self.get_logger().warn(
                    f'Controller timeout ({elapsed:.1f}s) — publishing zeros'
                )
                self.last_joy = None

            # Publish a zeroed message so the robot stops safely
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
