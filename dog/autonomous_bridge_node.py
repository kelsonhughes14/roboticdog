"""
autonomous_bridge_node.py
--------------------------
Bridges Nav2's /cmd_vel output into /gait_command while the robot is in
AUTONOMOUS state.  When the state is anything else, this node is silent —
state_manager drives /gait_command from the joystick as normal.

Subscribed topics:
  /cmd_vel       (geometry_msgs/Twist)  — Nav2 controller output
  /robot_state   (std_msgs/String)      — current robot state from state_manager

Published topics:
  /gait_command  (geometry_msgs/Twist)  — forwarded to gait_node (AUTONOMOUS only)
  /gait_type     (std_msgs/String)      — "TROT" published on autonomous entry

Safety watchdog:
  If no /cmd_vel arrives for more than CMD_VEL_TIMEOUT seconds while in
  AUTONOMOUS state, a zero-velocity Twist is published to stop the robot.
  This protects against Nav2 crashes or network hiccups.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from dog.state_manager import RobotState


class AutonomousBridgeNode(Node):

    CMD_VEL_TIMEOUT = 0.5   # seconds before watchdog fires

    def __init__(self):
        super().__init__('autonomous_bridge')

        self._state      = RobotState.SITTING
        self._last_cmd_t = time.time()
        self._timed_out  = False

        self.create_subscription(Twist,  'cmd_vel',     self._cmd_vel_cb,  10)
        self.create_subscription(String, 'robot_state', self._state_cb,    10)

        self._gait_pub  = self.create_publisher(Twist,  'gait_command', 10)
        self._gtype_pub = self.create_publisher(String, 'gait_type',    10)

        self.create_timer(0.02, self._watchdog_tick)   # 50 Hz

        self.get_logger().info(
            'Autonomous bridge ready — waiting for AUTONOMOUS state. '
            'Press Y on controller to activate.'
        )

    # ────────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        prev        = self._state
        self._state = msg.data

        if prev != RobotState.AUTONOMOUS and self._state == RobotState.AUTONOMOUS:
            # Publish TROT gait so gait_node is primed to move.
            gtype      = String()
            gtype.data = 'TROT'
            self._gtype_pub.publish(gtype)
            self._last_cmd_t = time.time()
            self._timed_out  = False
            self.get_logger().info(
                'AUTONOMOUS mode active — forwarding Nav2 /cmd_vel → /gait_command'
            )

        elif prev == RobotState.AUTONOMOUS and self._state != RobotState.AUTONOMOUS:
            self.get_logger().info('AUTONOMOUS mode deactivated.')

    def _cmd_vel_cb(self, msg: Twist):
        if self._state != RobotState.AUTONOMOUS:
            return
        self._last_cmd_t = time.time()
        self._timed_out  = False
        self._gait_pub.publish(msg)

    def _watchdog_tick(self):
        if self._state != RobotState.AUTONOMOUS:
            return
        if time.time() - self._last_cmd_t > self.CMD_VEL_TIMEOUT:
            if not self._timed_out:
                self.get_logger().warn(
                    f'No /cmd_vel for >{self.CMD_VEL_TIMEOUT}s — stopping robot '
                    '(Nav2 may have paused or reached goal).'
                )
                self._timed_out = True
            self._gait_pub.publish(Twist())   # zero velocity = stop


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
