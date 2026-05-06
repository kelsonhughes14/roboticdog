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
  /gait_command      (geometry_msgs/Twist)            — forwarded to gait_node (AUTONOMOUS only)
  /gait_type         (std_msgs/String)                — "SHUFFLE" published on autonomous entry
  /obstacle_markers  (visualization_msgs/MarkerArray) — RViz stop markers

Safety watchdog:
  If no /cmd_vel arrives for more than CMD_VEL_TIMEOUT seconds while in
  AUTONOMOUS state, a zero-velocity Twist is published to stop the robot.
  This protects against Nav2 crashes or network hiccups.

Obstacle hold:
  The lidar checks a forward corridor matching the robot's physical width
  (BODY_WIDTH from robot_config) plus a safety margin. If any point within
  that corridor is within OBSTACLE_STOP_DIST directly in front of the lidar,
  cmd_vel is suppressed and gait is forced to STAND until the path clears
  past OBSTACLE_CLEAR_DIST (hysteresis).
  The actual closest obstacle distance is logged; repeated logs are
  suppressed if the distance changes by less than REPORT_DELTA metres.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from dog.state_manager import RobotState
from dog.robot_config import BODY_WIDTH, BODY_LENGTH

# ── Robot footprint (converted to metres) ────────────────────────────────────
_BODY_WIDTH_M  = BODY_WIDTH  / 1000.0   # 0.48 m
_BODY_LENGTH_M = BODY_LENGTH / 1000.0   # 0.72 m

# ── Obstacle-hold tuning ─────────────────────────────────────────────────────
# Stop if any lidar point inside the robot corridor is within this forward
# distance from the lidar (x direction, not radial range).
OBSTACLE_STOP_DIST  = 0.60   # metres (600 mm)
# Only clear the hold once all corridor points exceed this (hysteresis).
OBSTACLE_CLEAR_DIST = 0.70   # metres
# Extra clearance added to each side of the robot body width.
SAFETY_MARGIN = 0.10         # metres
# Half-width of the corridor the robot needs to pass through.
_HALF_CORRIDOR = (_BODY_WIDTH_M / 2.0) + SAFETY_MARGIN   # 0.34 m
# Suppress repeated log lines if distance changes by less than this.
REPORT_DELTA = 0.05          # metres
_BLOCKED_GAIT = 'STAND'
_RESUME_GAIT  = 'SHUFFLE'

# ── RViz marker IDs ──────────────────────────────────────────────────────────
_NS_SPHERES = "obstacle_spheres"
_NS_TEXT    = "obstacle_text"
_ID_SPHERES = 0
_ID_TEXT    = 1


class AutonomousBridgeNode(Node):

    CMD_VEL_TIMEOUT = 0.5   # seconds before watchdog fires

    def __init__(self):
        super().__init__('autonomous_bridge')

        self._state                = RobotState.IDLE
        self._last_cmd_t           = time.time()
        self._last_scan_t          = 0.0
        self._timed_out            = False
        self._obstacle_held        = False
        self._last_reported_dist   = None

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.create_subscription(Twist,     'cmd_vel',     self._cmd_vel_cb,       10)
        self.create_subscription(String,    'robot_state', self._state_cb,         10)
        self.create_subscription(LaserScan, 'scan',        self._scan_cb,    scan_qos)

        self._gait_pub   = self.create_publisher(Twist,       'gait_command',     10)
        self._gtype_pub  = self.create_publisher(String,      'gait_type',        10)
        self._marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)

        self.create_timer(0.02, self._watchdog_tick)   # 50 Hz
        self.create_timer(0.05, self._obstacle_hold_tick)  # 20 Hz hard-hold while blocked
        self.create_timer(1.0, self._diagnostic_tick)  # scan/state health

        self.get_logger().info(
            f'Autonomous bridge ready — '
            f'robot {_BODY_WIDTH_M*1000:.0f} mm wide x {_BODY_LENGTH_M*1000:.0f} mm long | '
            f'corridor check: +/-{_HALF_CORRIDOR*100:.0f} cm from centreline | '
            f'stop at {OBSTACLE_STOP_DIST:.1f} m. '
            'Waiting for AUTONOMOUS state.'
        )

    # ── State ─────────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        prev        = self._state
        self._state = msg.data

        if prev != RobotState.AUTONOMOUS and self._state == RobotState.AUTONOMOUS:
            gtype      = String()
            gtype.data = _RESUME_GAIT
            self._gtype_pub.publish(gtype)
            self._last_cmd_t         = time.time()
            self._timed_out          = False
            self._obstacle_held      = False
            self._last_reported_dist = None
            self.get_logger().info(
                'AUTONOMOUS mode active — forwarding Nav2 /cmd_vel -> /gait_command'
            )

        elif prev == RobotState.AUTONOMOUS and self._state != RobotState.AUTONOMOUS:
            self._obstacle_held      = False
            self._last_reported_dist = None
            self._publish_clear_markers()
            self.get_logger().info('AUTONOMOUS mode deactivated.')

    # ── Scan / obstacle detection ─────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_t = time.time()
        if self._state != RobotState.AUTONOMOUS:
            return
        if msg.angle_increment == 0.0:
            return

        # Collect points that are in front of the lidar (x > 0), within the
        # forward stop window, and inside the robot corridor.
        obstacle_pts = []
        closest = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue
            angle   = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            if x <= 0.0 or x >= OBSTACLE_STOP_DIST:
                continue
            y = r * math.sin(angle)
            if abs(y) < _HALF_CORRIDOR:
                obstacle_pts.append((x, y))
                if x < closest:
                    closest = x

        blocked = len(obstacle_pts) > 0

        if blocked and not self._obstacle_held:
            # Newly blocked
            self._obstacle_held      = True
            self._last_reported_dist = closest
            gtype = String()
            gtype.data = _BLOCKED_GAIT
            self._gtype_pub.publish(gtype)
            self._gait_pub.publish(Twist())
            self.get_logger().warn(
                '\n'
                '╔══════════════════════════════════════════════════════╗\n'
                '║                      STOP!                          ║\n'
                f'║  Obstacle at {closest:.2f} m — robot ({_BODY_WIDTH_M*1000:.0f} mm wide) cannot fit  ║\n'
                '╚══════════════════════════════════════════════════════╝'
            )
            self._publish_stop_markers(msg.header.stamp, msg.header.frame_id, obstacle_pts)

        elif blocked and self._obstacle_held:
            # Still blocked — only re-log if distance shifted more than REPORT_DELTA
            if abs(closest - self._last_reported_dist) > REPORT_DELTA:
                self._last_reported_dist = closest
                self.get_logger().warn(
                    f'STOP! Obstacle at {closest:.2f} m — robot cannot fit through.'
                )
            self._publish_stop_markers(msg.header.stamp, msg.header.frame_id, obstacle_pts)

        elif not blocked and self._obstacle_held:
            # Check hysteresis — all corridor rays must exceed CLEAR_DIST
            corridor_clear = all(
                not (
                    math.isfinite(r) and
                    r >= msg.range_min and
                    0.0 < (r * math.cos(msg.angle_min + i * msg.angle_increment)) < OBSTACLE_CLEAR_DIST and
                    abs(r * math.sin(msg.angle_min + i * msg.angle_increment)) < _HALF_CORRIDOR
                )
                for i, r in enumerate(msg.ranges)
            )
            if corridor_clear:
                self._obstacle_held      = False
                self._last_reported_dist = None
                gtype = String()
                gtype.data = _RESUME_GAIT
                self._gtype_pub.publish(gtype)
                self.get_logger().info('Path clear — resuming navigation toward goal.')
                self._publish_clear_markers()

    # ── Velocity ──────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        if self._state != RobotState.AUTONOMOUS:
            return
        self._last_cmd_t = time.time()
        self._timed_out  = False
        if self._obstacle_held:
            self._gait_pub.publish(Twist())
            return
        self._gait_pub.publish(msg)

    def _watchdog_tick(self):
        if self._state != RobotState.AUTONOMOUS:
            return
        if self._obstacle_held:
            return
        if time.time() - self._last_cmd_t > self.CMD_VEL_TIMEOUT:
            if not self._timed_out:
                self.get_logger().warn(
                    f'No /cmd_vel for >{self.CMD_VEL_TIMEOUT}s — stopping robot '
                    '(Nav2 may have paused or reached goal).'
                )
                self._timed_out = True
            self._gait_pub.publish(Twist())

    def _obstacle_hold_tick(self):
        """Continuously enforce non-ESTOP obstacle pause while blocked."""
        if self._state != RobotState.AUTONOMOUS or not self._obstacle_held:
            return
        gtype = String()
        gtype.data = _BLOCKED_GAIT
        self._gtype_pub.publish(gtype)
        self._gait_pub.publish(Twist())

    def _diagnostic_tick(self):
        """Warn if autonomy is active but scan stream is missing."""
        if self._state != RobotState.AUTONOMOUS:
            return
        now = time.time()
        if self._last_scan_t == 0.0 or (now - self._last_scan_t) > 1.0:
            self.get_logger().warn(
                'AUTONOMOUS active but no recent /scan received; obstacle hold cannot engage. '
                'Check lidar node/topic/QoS.'
            )

    # ── RViz markers ─────────────────────────────────────────────────────────

    def _publish_stop_markers(self, stamp, frame_id: str, obstacle_pts):
        markers = MarkerArray()

        # Red spheres at each obstacle point inside the corridor
        sphere = Marker()
        sphere.header.stamp    = stamp
        sphere.header.frame_id = frame_id
        sphere.ns     = _NS_SPHERES
        sphere.id     = _ID_SPHERES
        sphere.type   = Marker.SPHERE_LIST
        sphere.action = Marker.ADD
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
        sphere.color  = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        for x, y in obstacle_pts:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            sphere.points.append(p)
        markers.markers.append(sphere)

        # Floating STOP! text above the sensor
        text = Marker()
        text.header.stamp    = stamp
        text.header.frame_id = frame_id
        text.ns     = _NS_TEXT
        text.id     = _ID_TEXT
        text.type   = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x    = 0.0
        text.pose.position.y    = 0.0
        text.pose.position.z    = 0.8
        text.pose.orientation.w = 1.0
        text.scale.z = 0.6
        text.color   = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        text.text    = 'STOP!'
        markers.markers.append(text)

        self._marker_pub.publish(markers)

    def _publish_clear_markers(self):
        markers = MarkerArray()
        for ns, mid in [(_NS_SPHERES, _ID_SPHERES), (_NS_TEXT, _ID_TEXT)]:
            m = Marker()
            m.ns     = ns
            m.id     = mid
            m.action = Marker.DELETE
            markers.markers.append(m)
        self._marker_pub.publish(markers)


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
