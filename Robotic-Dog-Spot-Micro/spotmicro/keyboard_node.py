"""
keyboard_node.py
----------------
Keyboard teleoperation for Spot Micro.

Publishes sensor_msgs/Joy to /joy_raw, going through controller_node
(same path as the Xbox joy_node), so state_manager sees a single
publisher on /joy with no competing zero-messages.

Controls
--------
  w / s         Forward / backward
  a / d         Strafe left / right
  q / e         Turn left / right
  SPACE         Sit <-> Stand  (BTN_A)
  Backspace     E-stop         (BTN_BACK)
  Enter         Clear E-stop   (BTN_START)
  g             Cycle gait     (BTN_X)
  W/S/A/D/Q/E   Same as above but turbo speed (RB held)
  Ctrl+C        Quit

Movement keys automatically hold the LB deadman switch, so the robot
will start walking as soon as you press w/a/s/d/q/e while standing.

Run in a dedicated terminal (controller_node must also be running):
  ros2 run spotmicro keyboard_node

Note: do not run joy_node at the same time — both would publish to
/joy_raw and fight each other.
"""

import curses
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from spotmicro.robot_config import (
    BTN_A, BTN_BACK, BTN_START, BTN_LB, BTN_RB, BTN_X,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X,
)

PUBLISH_HZ  = 20     # Joy message publish rate
KEY_TIMEOUT = 0.15   # seconds — key considered released if not seen within this


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self._lock    = threading.Lock()
        self._axes    = [0.0] * 8
        self._buttons = [0]   * 11
        self._running = True

        self.joy_pub = self.create_publisher(Joy, 'joy_raw', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info(
            'Keyboard node ready — terminal UI starting.'
        )

    # ----------------------------------------------------------------
    def update(self, axes: list, buttons: list):
        with self._lock:
            self._axes    = axes
            self._buttons = buttons

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.axes    = list(self._axes)
            msg.buttons = list(self._buttons)
        self.joy_pub.publish(msg)

    def stop(self):
        self._running = False


# ────────────────────────────────────────────────────────────────────
_HELP = [
    "Spot Micro  --  Keyboard Teleop",
    "",
    "  w / s       Forward / backward",
    "  a / d       Strafe left / right",
    "  q / e       Turn left / right",
    "  SPACE       Sit <-> Stand",
    "  Backspace   E-stop",
    "  Enter       Clear E-stop",
    "  g           Cycle gait",
    "  W/A/S/D     Turbo (uppercase letters)",
    "  Ctrl+C      Quit",
    "",
]

_LOWER_MOVE = {ord(c) for c in 'wasdqe'}
_UPPER_MOVE = {ord(c) for c in 'WASDQE'}
_ALL_MOVE   = _LOWER_MOVE | _UPPER_MOVE


def _run_curses(stdscr, node: KeyboardNode):
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.nodelay(True)   # non-blocking getch

    key_last      = {}   # keycode -> last-seen monotonic time  (held-key tracking)
    btn_until     = {}   # button index -> expire time          (one-shot pulse)
    btn_last_fire = {}   # button index -> time of last firing  (cooldown guard)

    ONESHOT_COOLDOWN = 0.5   # seconds — ignore key-repeat for one-shot buttons

    def _oneshot(btn: int):
        """Fire btn for one publish cycle, honouring the cooldown."""
        if now - btn_last_fire.get(btn, 0) > ONESHOT_COOLDOWN:
            btn_until[btn]     = now + 0.08
            btn_last_fire[btn] = now

    while node._running:
        key = stdscr.getch()
        now = time.monotonic()

        # ── Record key press ─────────────────────────────────────────
        if key != -1:
            if key in _ALL_MOVE:
                key_last[key] = now
            elif key == ord(' '):
                _oneshot(BTN_A)
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                _oneshot(BTN_BACK)
            elif key in (curses.KEY_ENTER, ord('\n'), ord('\r')):
                _oneshot(BTN_START)
            elif key == ord('g'):
                _oneshot(BTN_X)

        # ── Determine which movement keys are still "held" ───────────
        held_lower = {chr(k) for k, t in key_last.items()
                      if k in _LOWER_MOVE and now - t < KEY_TIMEOUT}
        held_upper = {chr(k).lower() for k, t in key_last.items()
                      if k in _UPPER_MOVE and now - t < KEY_TIMEOUT}
        held  = held_lower | held_upper
        turbo = bool(held_upper)

        # ── Build axes ───────────────────────────────────────────────
        # state_manager inverts axes: vx = -axis(LEFT_Y), vy = -axis(LEFT_X),
        # yaw = -axis(RIGHT_X), so set the sign opposite to desired direction.
        axes = [0.0] * 8
        axes[AXIS_LEFT_Y]  = (-1.0 if 'w' in held else 0.0) \
                           + ( 1.0 if 's' in held else 0.0)
        axes[AXIS_LEFT_X]  = ( 1.0 if 'a' in held else 0.0) \
                           + (-1.0 if 'd' in held else 0.0)
        axes[AXIS_RIGHT_X] = ( 1.0 if 'q' in held else 0.0) \
                           + (-1.0 if 'e' in held else 0.0)

        # ── Build buttons ────────────────────────────────────────────
        buttons = [0] * 11
        buttons[BTN_LB] = 1 if held  else 0   # auto deadman when moving
        buttons[BTN_RB] = 1 if turbo else 0   # turbo when uppercase

        for btn, until in btn_until.items():
            if now < until:
                buttons[btn] = 1

        node.update(axes, buttons)

        # ── Draw UI ──────────────────────────────────────────────────
        stdscr.clear()
        h, w = stdscr.getmaxyx()
        for i, line in enumerate(_HELP):
            if i < h - 2:
                try:
                    stdscr.addstr(i, 0, line[:w - 1])
                except curses.error:
                    pass
        status = f"  Active keys: {', '.join(sorted(held)) or 'none'}" \
                 + ("  [TURBO]" if turbo else "")
        try:
            stdscr.addstr(len(_HELP), 0, status[:w - 1])
        except curses.error:
            pass
        stdscr.refresh()

        time.sleep(0.02)   # 50 Hz poll — faster than publish rate


# ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    # Spin rclpy in a background thread so curses can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(_run_curses, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
