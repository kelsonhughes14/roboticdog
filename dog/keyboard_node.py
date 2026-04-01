"""
keyboard_node.py
----------------
Keyboard teleoperation for the Dog robot.

Publishes sensor_msgs/Joy to /joy_raw → controller_node → /joy → state_manager.

Controls
--------
  w / s         Forward / backward
  a / d         Strafe left / right
  q / e         Turn left / right
  SPACE         Sit <-> Stand         (BTN_A)
  Backspace     E-stop                (BTN_BACK)
  Enter         Clear E-stop          (BTN_START)
  f             Self-right            (BTN_B, from E-stop only)
  g             Cycle gait            (BTN_X)
  y             Toggle autonomous     (BTN_Y)
  j             Jump forward          (BTN_B, when standing/walking)
  b             Backflip              (BTN_B + BTN_RB)
  r             Reset sim             (sim mode only)
  W/S/A/D/Q/E   Turbo speed (RB held)
  Ctrl+C        Quit

Note: f and j both send BTN_B — state_manager decides the action:
  ESTOP state    → self-right
  STANDING/WALKING → jump forward

Run in a dedicated terminal (controller_node must be running):
  ros2 run dog keyboard_node

Do NOT run joy_node at the same time.
"""

import curses
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from dog.robot_config import (
    BTN_A, BTN_B, BTN_BACK, BTN_START, BTN_LB, BTN_RB, BTN_X, BTN_Y,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X,
)

PUBLISH_HZ  = 20
KEY_TIMEOUT = 0.15


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self._lock    = threading.Lock()
        self._axes    = [0.0] * 8
        self._buttons = [0]   * 11
        self._running = True

        self.joy_pub   = self.create_publisher(Joy,   'joy_raw',   10)
        self.reset_pub = self.create_publisher(Empty, 'sim_reset', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info('Keyboard node ready — terminal UI starting.')

    def update(self, axes, buttons):
        with self._lock:
            self._axes    = axes
            self._buttons = buttons

    def publish_reset(self):
        self.reset_pub.publish(Empty())

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.axes    = list(self._axes)
            msg.buttons = list(self._buttons)
        self.joy_pub.publish(msg)

    def stop(self):
        self._running = False


_HELP = [
    "Dog Robot  --  Keyboard Teleop",
    "",
    "  w / s       Forward / backward",
    "  a / d       Strafe left / right",
    "  q / e       Turn left / right",
    "  SPACE       Sit <-> Stand",
    "  Backspace   E-stop",
    "  Enter       Clear E-stop",
    "  f / j       Self-right (E-stop) / Jump forward (standing)",
    "  g           Cycle gait",
    "  y           Toggle autonomous mode",
    "  b           Backflip",
    "  r           Reset sim (sim mode only)",
    "  W/A/S/D     Turbo (uppercase)",
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
    stdscr.nodelay(True)

    key_last      = {}
    btn_until     = {}
    btn_last_fire = {}

    ONESHOT_COOLDOWN = 0.5

    def _oneshot(btn):
        if now - btn_last_fire.get(btn, 0) > ONESHOT_COOLDOWN:
            btn_until[btn]     = now + 0.08
            btn_last_fire[btn] = now

    while node._running:
        key = stdscr.getch()
        now = time.monotonic()

        if key != -1:
            if key in _ALL_MOVE:
                key_last[key] = now
            elif key == ord(' '):
                _oneshot(BTN_A)
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                _oneshot(BTN_BACK)
            elif key in (curses.KEY_ENTER, ord('\n'), ord('\r')):
                _oneshot(BTN_START)
            elif key in (ord('f'), ord('j')):
                _oneshot(BTN_B)              # self-right (ESTOP) or jump (standing)
            elif key == ord('g'):
                _oneshot(BTN_X)
            elif key == ord('y'):
                _oneshot(BTN_Y)              # toggle autonomous mode
            elif key == ord('b'):
                _oneshot(BTN_B)              # backflip = B + RB
                _oneshot(BTN_RB)
            elif key == ord('r'):
                node.publish_reset()

        held_lower = {chr(k) for k, t in key_last.items()
                      if k in _LOWER_MOVE and now - t < KEY_TIMEOUT}
        held_upper = {chr(k).lower() for k, t in key_last.items()
                      if k in _UPPER_MOVE and now - t < KEY_TIMEOUT}
        held  = held_lower | held_upper
        turbo = bool(held_upper)

        axes = [0.0] * 8
        axes[AXIS_LEFT_Y]  = (-1.0 if 'w' in held else 0.0) + (1.0 if 's' in held else 0.0)
        axes[AXIS_LEFT_X]  = ( 1.0 if 'a' in held else 0.0) + (-1.0 if 'd' in held else 0.0)
        axes[AXIS_RIGHT_X] = (-1.0 if 'q' in held else 0.0) + ( 1.0 if 'e' in held else 0.0)

        buttons = [0] * 11
        buttons[BTN_LB] = 1 if held  else 0
        buttons[BTN_RB] = 1 if turbo else 0

        for btn, until in btn_until.items():
            if now < until:
                buttons[btn] = 1

        node.update(axes, buttons)

        stdscr.clear()
        h, w = stdscr.getmaxyx()
        for i, line in enumerate(_HELP):
            if i < h - 2:
                try:
                    stdscr.addstr(i, 0, line[:w - 1])
                except curses.error:
                    pass
        status = f"  Active: {', '.join(sorted(held)) or 'none'}" \
                 + ("  [TURBO]" if turbo else "")
        try:
            stdscr.addstr(len(_HELP), 0, status[:w - 1])
        except curses.error:
            pass
        stdscr.refresh()

        time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

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
