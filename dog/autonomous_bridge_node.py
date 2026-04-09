"""
autonomous_bridge_node.py
--------------------------
Bridges Nav2's /cmd_vel output into /gait_command while the robot is in
AUTONOMOUS state.  When the state is anything else, this node is silent —
state_manager drives /gait_command from the joystick as normal.

Subscribed topics:
  /cmd_vel       (geometry_msgs/Twist)   — Nav2 controller output
  /robot_state   (std_msgs/String)       — current robot state from state_manager
  /scan          (sensor_msgs/LaserScan) — Hokuyo lidar for obstacle detection

Published topics:
  /gait_command  (geometry_msgs/Twist)  — forwarded to gait_node (AUTONOMOUS only)
  /gait_type     (std_msgs/String)      — "TROT" published on autonomous entry

Safety watchdog:
  If no /cmd_vel arrives for more than CMD_VEL_TIMEOUT seconds while in
  AUTONOMOUS state, a zero-velocity Twist is published to stop the robot.
  This protects against Nav2 crashes or network hiccups.

Obstacle hold:
  The robot is 8DOF (no hip motors) and cannot turn in place to navigate
  around obstacles.  When the lidar detects an obstacle within
  OBSTACLE_STOP_DIST metres inside the forward ±OBSTACLE_HALF_ANGLE cone,
  cmd_vel is suppressed (zero velocity published) until the path clears.
  Nav2 keeps running and retains the goal pose; motion resumes automatically
  once the obstacle moves away.  A small hysteresis band (OBSTACLE_CLEAR_DIST
  > OBSTACLE_STOP_DIST) prevents oscillation at the boundary.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from dog.state_manager import RobotState

# ── Obstacle-hold tuning ─────────────────────────────────────────────────────
# Stop if any lidar ray in the forward cone is closer than this.
OBSTACLE_STOP_DIST  = 0.50          # metres
# Only clear the hold once all rays in the cone exceed this (hysteresis).
OBSTACLE_CLEAR_DIST = 0.65          # metres
# Forward cone half-angle checked for obstacles.
OBSTACLE_HALF_ANGLE = math.radians(30)   # ±30°


class AutonomousBridgeNode(Node):

    CMD_VEL_TIMEOUT = 0.5   # seconds before watchdog fires

    def __init__(self):
        super().__init__('autonomous_bridge')

        self._state           = RobotState.SITTING
        self._last_cmd_t      = time.time()
        self._timed_out       = False
        self._obstacle_held   = False   # True while obstacle is blocking forward path

        self.create_subscription(Twist,     'cmd_vel',     self._cmd_vel_cb,  10)
        self.create_subscription(String,    'robot_state', self._state_cb,    10)
        self.create_subscription(LaserScan, 'scan',        self._scan_cb,     10)

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
            self._last_cmd_t    = time.time()
            self._timed_out     = False
            self._obstacle_held = False
            self.get_logger().info(
                'AUTONOMOUS mode active — forwarding Nav2 /cmd_vel → /gait_command'
            )

        elif prev == RobotState.AUTONOMOUS and self._state != RobotState.AUTONOMOUS:
            self._obstacle_held = False
            self.get_logger().info('AUTONOMOUS mode deactivated.')

    def _scan_cb(self, msg: LaserScan):
        """Check the forward lidar cone for obstacles and update _obstacle_held."""
        if self._state != RobotState.AUTONOMOUS:
            return

        # Determine which ray indices fall within the forward ±OBSTACLE_HALF_ANGLE cone.
        if msg.angle_increment == 0.0:
            return

        n = len(msg.ranges)
        blocked = False
        any_in_cone = False

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > OBSTACLE_HALF_ANGLE:
                continue
            any_in_cone = True
            # Ignore out-of-range readings (inf, nan, or below sensor minimum).
            if not math.isfinite(r) or r < msg.range_min:
                continue
            if r < OBSTACLE_STOP_DIST:
                blocked = True
                break

        if not any_in_cone:
            return   # no rays in forward cone (unexpected scan geometry) — ignore

        if not self._obstacle_held and blocked:
            self._obstacle_held = True
            self._gait_pub.publish(Twist())   # stop immediately
            self.get_logger().warn(
                f'Obstacle detected within {OBSTACLE_STOP_DIST:.2f} m — '
                'holding in place until path clears.'
            )
        elif self._obstacle_held and not blocked:
            # Only clear if all cone rays exceed the hysteresis distance.
            all_clear = all(
                (not math.isfinite(msg.ranges[i]) or
                 msg.ranges[i] < msg.range_min or
                 msg.ranges[i] > OBSTACLE_CLEAR_DIST)
                for i in range(n)
                if abs(msg.angle_min + i * msg.angle_increment) <= OBSTACLE_HALF_ANGLE
            )
            if all_clear:
                self._obstacle_held = False
                self.get_logger().info(
                    'Path clear — resuming navigation toward goal.'
                )

    def _cmd_vel_cb(self, msg: Twist):
        if self._state != RobotState.AUTONOMOUS:
            return
        self._last_cmd_t = time.time()
        self._timed_out  = False
        if self._obstacle_held:
            # Suppress Nav2 velocity commands while an obstacle is in the way.
            self._gait_pub.publish(Twist())
            return
        self._gait_pub.publish(msg)

    def _watchdog_tick(self):
        if self._state != RobotState.AUTONOMOUS:
            return
        if self._obstacle_held:
            # Obstacle hold already publishing zero — watchdog not needed.
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
