"""
leg_joy_node.py
---------------
Control one or two legs with the Xbox controller while all others hold neutral.

Controls:
  LB (press)       cycle primary leg  (FR → FL → RR → RL)
  RB (press)       cycle secondary leg through the other 3 legs
                   (first press adds one, subsequent presses change it)
  L2 (trigger)     remove secondary leg — back to single-leg mode
  Left stick X     hip   angle
  Left stick Y     shoulder angle
  Right stick Y    knee  angle
  A button         reset active leg(s) to neutral
  BACK button      reset all legs to neutral
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

from spotmicro.robot_config import (
    NEUTRAL_ANGLES,
    SERVO_ANGLE_MIN, SERVO_ANGLE_MAX,
    SERVO_DIRECTION,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_Y, AXIS_LT,
    BTN_LB, BTN_RB, BTN_A, BTN_BACK,
    JOYSTICK_DEADZONE,
)

LEG_NAMES = ['FR', 'FL', 'RR', 'RL']

JOINT_RATE_DEG_S = 60.0
CONTROL_RATE_HZ  = 20
LT_THRESHOLD     = 0.5   # L2 axis value to treat as a button press


def _clamp(v: float) -> float:
    return max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, v))


def _deadzone(v: float) -> float:
    return v if abs(v) > JOYSTICK_DEADZONE else 0.0


class LegJoyNode(Node):

    def __init__(self):
        super().__init__('leg_joy_node')

        self._angles        = [90.0] * 12
        self._primary       = 0       # always set, 0–3
        self._secondary     = None    # None = single mode, 0–3 = dual mode

        # Button / axis edge detection
        self._prev_lb  = 0
        self._prev_rb  = 0
        self._prev_a   = 0
        self._prev_back = 0
        self._prev_lt  = False

        # Live joystick axes
        self._hip_axis = 0.0
        self._sho_axis = 0.0
        self._kne_axis = 0.0

        self._pub = self.create_publisher(Float32MultiArray, 'servo_angles', 10)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)

        self._log()

    # ── Active legs ───────────────────────────────────────────────────────────

    @property
    def _active_legs(self) -> tuple:
        if self._secondary is None:
            return (self._primary,)
        return (self._primary, self._secondary)

    def _other_legs(self) -> list:
        """The 3 legs that are not the current primary, in cycle order."""
        return [l for l in range(4) if l != self._primary]

    def _next_secondary(self) -> int:
        """Next secondary option cycling through the other 3 legs."""
        options = self._other_legs()
        if self._secondary is None or self._secondary not in options:
            return options[0]
        idx = options.index(self._secondary)
        return options[(idx + 1) % len(options)]

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self) -> None:
        legs = self._active_legs
        name = ' + '.join(LEG_NAMES[l] for l in legs)
        mode = 'single' if len(legs) == 1 else 'dual'
        parts = []
        for leg in legs:
            b = leg * 3
            parts.append(
                f'{LEG_NAMES[leg]}: '
                f'hip={self._angles[b]:.1f}° '
                f'sho={self._angles[b+1]:.1f}° '
                f'kne={self._angles[b+2]:.1f}°'
            )
        self.get_logger().info(f'[{mode}] {name}  |  ' + '  |  '.join(parts))

    # ── Joy callback ──────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        def btn(i: int) -> int:
            return msg.buttons[i] if i < len(msg.buttons) else 0

        def axis(i: int) -> float:
            return float(msg.axes[i]) if i < len(msg.axes) else 0.0

        lb   = btn(BTN_LB)
        rb   = btn(BTN_RB)
        a    = btn(BTN_A)
        back = btn(BTN_BACK)
        lt   = axis(AXIS_LT) > LT_THRESHOLD

        # LB press → cycle primary leg
        if lb == 1 and self._prev_lb == 0:
            self._primary = (self._primary + 1) % 4
            # If secondary became the same as primary, clear it
            if self._secondary == self._primary:
                self._secondary = None
            self._log()

        # RB press → cycle secondary leg (excludes primary)
        if rb == 1 and self._prev_rb == 0:
            self._secondary = self._next_secondary()
            self._log()

        # L2 press → remove secondary (back to single)
        if lt and not self._prev_lt:
            if self._secondary is not None:
                self._secondary = None
                self._log()

        # A press → reset active legs
        if a == 1 and self._prev_a == 0:
            for leg in self._active_legs:
                self._reset_leg(leg)
            name = ' + '.join(LEG_NAMES[l] for l in self._active_legs)
            self.get_logger().info(f'{name} reset to neutral')

        # BACK press → reset all
        if back == 1 and self._prev_back == 0:
            self._angles = [90.0] * 12
            self.get_logger().info('All legs reset to neutral')

        self._prev_lb   = lb
        self._prev_rb   = rb
        self._prev_a    = a
        self._prev_back = back
        self._prev_lt   = lt

        self._hip_axis = _deadzone(axis(AXIS_LEFT_X))
        self._sho_axis = _deadzone(-axis(AXIS_LEFT_Y))
        self._kne_axis = _deadzone(-axis(AXIS_RIGHT_Y))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        dt    = 1.0 / CONTROL_RATE_HZ
        scale = JOINT_RATE_DEG_S * dt

        for leg in self._active_legs:
            base = leg * 3
            hip_delta = self._hip_axis * scale * SERVO_DIRECTION[(leg, 0)]
            sho_delta = self._sho_axis * scale * SERVO_DIRECTION[(leg, 1)]
            kne_delta = self._kne_axis * scale * SERVO_DIRECTION[(leg, 2)]

            if hip_delta:
                self._angles[base + 0] = _clamp(self._angles[base + 0] + hip_delta)
            if sho_delta:
                self._angles[base + 1] = _clamp(self._angles[base + 1] + sho_delta)
            if kne_delta:
                self._angles[base + 2] = _clamp(self._angles[base + 2] + kne_delta)

        msg = Float32MultiArray()
        msg.data = [float(a) for a in self._angles]
        self._pub.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

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
