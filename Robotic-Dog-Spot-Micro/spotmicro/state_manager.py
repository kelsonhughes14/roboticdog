"""
state_manager.py
----------------
Manages the high-level robot state machine.

States
------
  IDLE        Robot is powered but not initialised.
  SITTING     All legs folded, safe resting position.
  STANDING    Robot is up and holding still.
  WALKING     Gait controller is active.
  ESTOP       Emergency stop — all servos disabled.

Subscribed topics:
  /joy                        (sensor_msgs/Joy)
  /imu/euler                  (geometry_msgs/Vector3)

Published topics:
  /robot_state                (std_msgs/String)    — current state name
  /gait_command               (geometry_msgs/Twist) — velocity command for gait node
  /body_pose                  (geometry_msgs/Vector3) — roll, pitch, yaw request
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

from spotmicro.robot_config import (
    BTN_A, BTN_BACK, BTN_START, BTN_LB, BTN_RB, BTN_X,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y,
    AXIS_LT, AXIS_RT,
    JOYSTICK_SCALE, TURN_SCALE, TURBO_MULTIPLIER, JOYSTICK_DEADZONE,
    MAX_BODY_ROLL, MAX_BODY_PITCH,
    NEUTRAL_ANGLES, SIT_ANGLES,
)
from spotmicro.gait_generator import GaitType

# Ordered cycle of walk gaits — X button steps through these
_WALK_GAIT_CYCLE = [GaitType.WALK, GaitType.CRAWL, GaitType.TROT]


class RobotState:
    IDLE     = 'IDLE'
    SITTING  = 'SITTING'
    STANDING = 'STANDING'
    WALKING  = 'WALKING'
    ESTOP    = 'ESTOP'


class StateManagerNode(Node):

    def __init__(self):
        super().__init__('state_manager')

        # ── State ────────────────────────────────────────────────────
        self.state          = RobotState.IDLE
        self.prev_a         = 0
        self.prev_back      = 0
        self.prev_start     = 0
        self.prev_x         = 0
        self._front_stand_timer = None

        # Walk gait selection (cycled with BTN_X)
        self._walk_gait_idx  = 0   # index into _WALK_GAIT_CYCLE
        self._active_walk_gait = _WALK_GAIT_CYCLE[0]

        # ── Subscriptions ────────────────────────────────────────────
        self.create_subscription(Joy,    'joy',       self._joy_callback,   10)
        self.create_subscription(Vector3,'imu/euler', self._imu_callback,   10)

        # ── Publishers ───────────────────────────────────────────────
        self.state_pub    = self.create_publisher(String,          'robot_state',  10)
        self.gait_pub     = self.create_publisher(Twist,           'gait_command', 10)
        self.pose_pub     = self.create_publisher(Vector3,         'body_pose',    10)
        self.servo_pub    = self.create_publisher(Float32MultiArray,'servo_angles', 10)
        self.gait_type_pub = self.create_publisher(String,         'gait_type',    10)

        # ── Status timer ─────────────────────────────────────────────
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info('State manager ready. Press A to stand up.')

        # Transition to SITTING on start
        self._set_state(RobotState.SITTING)
        self._publish_servo_angles(SIT_ANGLES)

    # ────────────────────────────────────────────────────────────────
    # STATE MACHINE
    # ────────────────────────────────────────────────────────────────
    def _set_state(self, new_state: str):
        if new_state == self.state:
            return
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        self._publish_state()  # Immediate publish so gait_node reacts instantly

    def _joy_callback(self, msg: Joy):
        if not msg.buttons or not msg.axes:
            return

        # ── E-STOP: BACK button ──────────────────────────────────────
        back = msg.buttons[BTN_BACK] if BTN_BACK < len(msg.buttons) else 0
        if back == 1 and self.prev_back == 0:
            self._set_state(RobotState.ESTOP)
            self.get_logger().warn('E-STOP activated!')
        self.prev_back = back

        if self.state == RobotState.ESTOP:
            # START button clears E-stop and goes back to sitting
            start = msg.buttons[BTN_START] if BTN_START < len(msg.buttons) else 0
            if start == 1 and self.prev_start == 0:
                self.get_logger().info('E-STOP cleared.')
                self._set_state(RobotState.SITTING)
                self._publish_servo_angles(SIT_ANGLES)
            self.prev_start = start
            return

        # ── A BUTTON: SIT ↔ STAND toggle ────────────────────────────
        a = msg.buttons[BTN_A] if BTN_A < len(msg.buttons) else 0
        if a == 1 and self.prev_a == 0:
            if self.state in (RobotState.IDLE, RobotState.SITTING):
                self._set_state(RobotState.STANDING)
                # Phase 1: rear legs rise immediately, front legs stay at SIT
                rear_first = list(SIT_ANGLES[:6]) + list(NEUTRAL_ANGLES[6:])
                self._publish_servo_angles(rear_first)
                # Phase 2: front legs rise after a short delay
                self._cancel_front_stand_timer()
                self._front_stand_timer = self.create_timer(0.2, self._complete_stand)
            elif self.state in (RobotState.STANDING, RobotState.WALKING):
                self._cancel_front_stand_timer()
                self._set_state(RobotState.SITTING)
                self._publish_servo_angles(SIT_ANGLES)
        self.prev_a = a

        # ── X BUTTON: cycle walk gait (available when STANDING or WALKING) ──
        if self.state in (RobotState.STANDING, RobotState.WALKING):
            x_btn = msg.buttons[BTN_X] if BTN_X < len(msg.buttons) else 0
            if x_btn == 1 and self.prev_x == 0:
                self._walk_gait_idx = (self._walk_gait_idx + 1) % len(_WALK_GAIT_CYCLE)
                self._active_walk_gait = _WALK_GAIT_CYCLE[self._walk_gait_idx]
                self._publish_gait_type(self._active_walk_gait)
                self.get_logger().info(
                    f'Walk gait → {self._active_walk_gait.name}'
                )
            self.prev_x = x_btn

        # ── MOVEMENT (only when STANDING or WALKING, LB held) ────────
        if self.state not in (RobotState.STANDING, RobotState.WALKING):
            return

        lb = msg.buttons[BTN_LB] if BTN_LB < len(msg.buttons) else 0
        rb = msg.buttons[BTN_RB] if BTN_RB < len(msg.buttons) else 0

        def axis(i: int) -> float:
            return float(msg.axes[i]) if i < len(msg.axes) else 0.0

        def deadzone(v: float) -> float:
            return v if abs(v) > JOYSTICK_DEADZONE else 0.0

        vx  = -deadzone(axis(AXIS_LEFT_Y))   # forward / back  (joy Y is up=-1, invert)
        vy  = -deadzone(axis(AXIS_LEFT_X))   # strafe          (joy X is right=-1, invert)
        yaw = -deadzone(axis(AXIS_RIGHT_X))  # turn            (joy X is right=-1, invert)

        # Body tilt via right stick Y and triggers
        body_pitch = deadzone(axis(AXIS_RIGHT_Y)) * MAX_BODY_PITCH
        lt         = (1.0 - axis(AXIS_LT)) / 2.0   # triggers report -1→+1, map to 0→1
        rt         = (1.0 - axis(AXIS_RT)) / 2.0
        body_roll  = (rt - lt) * MAX_BODY_ROLL

        speed_mult = TURBO_MULTIPLIER if rb else 1.0

        moving = abs(vx) > 0 or abs(vy) > 0 or abs(yaw) > 0

        if lb and moving:
            if self.state == RobotState.STANDING:
                self._set_state(RobotState.WALKING)
                self._publish_gait_type(self._active_walk_gait)

            twist = Twist()
            twist.linear.x  = vx  * JOYSTICK_SCALE * speed_mult
            twist.linear.y  = vy  * JOYSTICK_SCALE * speed_mult
            twist.angular.z = yaw * TURN_SCALE      * speed_mult
            self.gait_pub.publish(twist)

        elif not moving and self.state == RobotState.WALKING:
            self._set_state(RobotState.STANDING)
            self._stop_gait()
            self._publish_servo_angles(NEUTRAL_ANGLES)

        # Always publish body pose
        pose = Vector3()
        pose.x = body_roll
        pose.y = body_pitch
        pose.z = 0.0
        self.pose_pub.publish(pose)

    # ────────────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Vector3):
        """Monitor IMU for falls. Auto E-stop if tilt exceeds safe limits."""
        if self.state not in (RobotState.STANDING, RobotState.WALKING):
            return
        roll  = abs(msg.x)
        pitch = abs(msg.y)
        if roll > 45.0 or pitch > 45.0:
            self.get_logger().error(
                f'Fall detected! roll={roll:.1f}° pitch={pitch:.1f}° '
                f'— triggering E-STOP'
            )
            self._set_state(RobotState.ESTOP)

    # ────────────────────────────────────────────────────────────────
    def _complete_stand(self):
        """Phase 2 of stand-up: raise front legs to NEUTRAL after rear legs settled."""
        self._cancel_front_stand_timer()
        if self.state == RobotState.STANDING:
            self._publish_servo_angles(NEUTRAL_ANGLES)

    def _cancel_front_stand_timer(self):
        if self._front_stand_timer is not None:
            self._front_stand_timer.cancel()
            self._front_stand_timer = None

    def _stop_gait(self):
        """Send zero velocity to halt movement."""
        self.gait_pub.publish(Twist())

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_gait_type(self, gait_type: GaitType):
        msg = String()
        msg.data = gait_type.name
        self.gait_type_pub.publish(msg)

    def _publish_servo_angles(self, angles: list[float]):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.servo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
