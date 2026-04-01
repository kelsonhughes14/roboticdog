"""
leg_joy_node.py
---------------
Control one or two legs with the Xbox controller while all others hold neutral.
Joint angles are in RADIANS (AK45-36 CAN motor commands).

Controls:
  LB (press)       cycle primary leg  (FR → FL → RR → RL)
  RB (press)       cycle secondary leg through the other 3 legs
  L2 (trigger)     remove secondary leg — back to single-leg mode
  Left stick X     hip   angle
  Left stick Y     shoulder angle
  Right stick Y    knee  angle
  A button         reset active leg(s) to neutral
  BACK button      E-stop — exit MIT motor mode (disable all motors)
  START button     Re-enable motors (enter MIT motor mode)

Published topics:
  /joint_angles  (std_msgs/Float32MultiArray)  — 12 motor commands in radians
  /can_enable    (std_msgs/Bool)               — motor enable / disable
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool

from dog.robot_config import (
    NEUTRAL_ANGLES,
    JOINT_ANGLE_MIN, JOINT_ANGLE_MAX,
    JOINT_DIRECTION,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_Y, AXIS_LT,
    BTN_LB, BTN_RB, BTN_A, BTN_BACK, BTN_START,
    PS4_BTN_SQUARE, PS4_BTN_BACK, PS4_BTN_START,
    JOYSTICK_DEADZONE,
)

LEG_NAMES = ['FR', 'FL', 'RR', 'RL']

JOINT_RATE_RAD_S = 1.0    # rad/s — how fast joints move per full stick deflection
CONTROL_RATE_HZ  = 20
LT_THRESHOLD     = 0.5


def _clamp(v: float) -> float:
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, v))


def _deadzone(v: float) -> float:
    return v if abs(v) > JOYSTICK_DEADZONE else 0.0


class LegJoyNode(Node):

    def __init__(self):
        super().__init__('leg_joy_node')

        self.declare_parameter('controller_type', 'xbox')
        ctrl_type = self.get_parameter('controller_type').get_parameter_value().string_value
        if ctrl_type == 'ps4':
            self._btn_a    = PS4_BTN_SQUARE
            self._btn_back = PS4_BTN_BACK
        else:
            self._btn_a    = BTN_A
            self._btn_back = BTN_BACK

        # Start at neutral standing angles
        self._angles    = list(NEUTRAL_ANGLES)
        self._primary   = 0
        self._secondary = None

        self._prev_lb    = 0
        self._prev_rb    = 0
        self._prev_a     = 0
        self._prev_back  = 0
        self._prev_start = 0
        self._prev_lt    = False

        self._hip_axis = 0.0
        self._sho_axis = 0.0
        self._kne_axis = 0.0

        self._motors_enabled = False

        self._pub        = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self._enable_pub = self.create_publisher(Bool, 'can_enable', 10)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)

        # Auto-enable after 2 s to give micro-ROS agent time to connect (fires once)
        self._auto_enable_timer = self.create_timer(2.0, self._auto_enable)

        self._log()

    @property
    def _active_legs(self) -> tuple:
        if self._secondary is None:
            return (self._primary,)
        return (self._primary, self._secondary)

    def _other_legs(self) -> list:
        return [l for l in range(4) if l != self._primary]

    def _next_secondary(self) -> int:
        options = self._other_legs()
        if self._secondary is None or self._secondary not in options:
            return options[0]
        idx = options.index(self._secondary)
        return options[(idx + 1) % len(options)]

    def _log(self) -> None:
        legs = self._active_legs
        name = ' + '.join(LEG_NAMES[l] for l in legs)
        mode = 'single' if len(legs) == 1 else 'dual'
        parts = []
        for leg in legs:
            b = leg * 3
            parts.append(
                f'{LEG_NAMES[leg]}: '
                f'hip={self._angles[b]:.3f} '
                f'sho={self._angles[b+1]:.3f} '
                f'kne={self._angles[b+2]:.3f} rad'
            )
        self.get_logger().info(f'[{mode}] {name}  |  ' + '  |  '.join(parts))

    def _set_enable(self, enabled: bool) -> None:
        self._motors_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self._enable_pub.publish(msg)
        state = 'ENABLED' if enabled else 'DISABLED'
        self.get_logger().info(f'Motors {state}')

    def _auto_enable(self) -> None:
        """Called once 2 s after startup — enter MIT motor mode."""
        self._auto_enable_timer.cancel()
        self._set_enable(True)

    def _joy_cb(self, msg: Joy) -> None:
        def btn(i):
            return msg.buttons[i] if i < len(msg.buttons) else 0

        def ax(i):
            return float(msg.axes[i]) if i < len(msg.axes) else 0.0

        lb    = btn(BTN_LB)
        rb    = btn(BTN_RB)
        a     = btn(self._btn_a)
        back  = btn(self._btn_back)
        start = btn(BTN_START)
        lt    = ax(AXIS_LT) > LT_THRESHOLD

        if lb == 1 and self._prev_lb == 0:
            self._primary = (self._primary + 1) % 4
            if self._secondary == self._primary:
                self._secondary = None
            self._log()

        if rb == 1 and self._prev_rb == 0:
            self._secondary = self._next_secondary()
            self._log()

        if lt and not self._prev_lt:
            if self._secondary is not None:
                self._secondary = None
                self._log()

        if a == 1 and self._prev_a == 0:
            for leg in self._active_legs:
                self._reset_leg(leg)
            name = ' + '.join(LEG_NAMES[l] for l in self._active_legs)
            self.get_logger().info(f'{name} reset to neutral')

        if back == 1 and self._prev_back == 0:
            self._set_enable(False)

        if start == 1 and self._prev_start == 0:
            self._set_enable(True)

        self._prev_lb    = lb
        self._prev_rb    = rb
        self._prev_a     = a
        self._prev_back  = back
        self._prev_start = start
        self._prev_lt    = lt

        self._hip_axis = _deadzone(ax(AXIS_LEFT_X))
        self._sho_axis = _deadzone(-ax(AXIS_LEFT_Y))
        self._kne_axis = _deadzone(-ax(AXIS_RIGHT_Y))

    def _control_loop(self) -> None:
        if not self._motors_enabled:
            return

        dt    = 1.0 / CONTROL_RATE_HZ
        scale = JOINT_RATE_RAD_S * dt

        for leg in self._active_legs:
            base = leg * 3
            hip_delta = self._hip_axis * scale * JOINT_DIRECTION[(leg, 0)]
            sho_delta = self._sho_axis * scale * JOINT_DIRECTION[(leg, 1)]
            kne_delta = self._kne_axis * scale * JOINT_DIRECTION[(leg, 2)]

            if hip_delta:
                self._angles[base + 0] = _clamp(self._angles[base + 0] + hip_delta)
            if sho_delta:
                self._angles[base + 1] = _clamp(self._angles[base + 1] + sho_delta)
            if kne_delta:
                self._angles[base + 2] = _clamp(self._angles[base + 2] + kne_delta)

        msg = Float32MultiArray()
        msg.data = [float(a) for a in self._angles]
        self._pub.publish(msg)

    def _reset_leg(self, leg: int) -> None:
        base = leg * 3
        for j in range(3):
            self._angles[base + j] = NEUTRAL_ANGLES[base + j]


def main(args=None):
    rclpy.init(args=args)
    node = LegJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
