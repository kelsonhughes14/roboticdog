"""
keyboard_node.py
----------------
Keyboard teleoperation for the Dog robot.

Publishes sensor_msgs/Joy to /joy_raw → controller_node → /joy → state_manager.
Subscribes to /robot_state to show context-sensitive help and live state display.

Controls — ESTOP (default at startup)
--------------------------------------
  Enter         Clear E-stop → enter POSITIONING
  Backspace     E-stop (from any state)

Controls — POSITIONING (after pressing Enter from E-stop)
----------------------------------------------------------
  1             Capture FR (front-right) leg
  2             Capture FL (front-left)  leg
  3             Capture RR (rear-right)  leg
  4             Capture RL (rear-left)   leg
  Enter         Confirm all legs and stand up

Controls — STANDING / WALKING
-------------------------------
  w / s         Forward / backward
  a / d         Strafe left / right
  q / e         Turn left / right
  W/S/A/D/Q/E   Turbo speed (uppercase)
  SPACE         Standup recovery animation
  g             Cycle gait
  y             Toggle autonomous mode
  j             Jump forward
  b             Backflip
  Backspace     E-stop

Other
-----
  r             Reset sim (sim mode only)
  Ctrl+C        Quit

Run in a dedicated terminal (controller_node must be running):
  ros2 run dog keyboard_node
"""

import curses
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String

from dog.robot_config import (
    BTN_A, BTN_B, BTN_BACK, BTN_START, BTN_LB, BTN_RB, BTN_X, BTN_Y,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X,
)
from dog.gait_generator import GaitType
from dog.state_manager import RobotState

PUBLISH_HZ  = 20
KEY_TIMEOUT = 0.15
_WALK_GAIT_CYCLE = [
    GaitType.SHUFFLE.name,
    GaitType.CRAWL.name,
    GaitType.WALK.name,
    GaitType.DIAG_TROT.name,
    GaitType.TROT.name,
    GaitType.GALLOP.name,
    GaitType.STEP.name,
    GaitType.TURTLE.name,
]


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self._lock        = threading.Lock()
        self._axes        = [0.0] * 8
        self._buttons     = [0]   * 11
        self._running     = True
        self._robot_state = RobotState.ESTOP
        self._gait_type   = GaitType.SHUFFLE.name

        self.joy_pub   = self.create_publisher(Joy,   'joy_raw',   10)
        self.reset_pub = self.create_publisher(Empty, 'sim_reset', 10)

        self.create_subscription(String, 'robot_state', self._state_cb, 10)
        self.create_subscription(String, 'gait_type', self._gait_cb, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info('Keyboard node ready.')

    def _state_cb(self, msg: String):
        with self._lock:
            self._robot_state = msg.data

    def get_state(self) -> str:
        with self._lock:
            return self._robot_state

    def _gait_cb(self, msg: String):
        with self._lock:
            self._gait_type = msg.data

    def get_gait_type(self) -> str:
        with self._lock:
            return self._gait_type

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


# ── Context-sensitive help blocks ─────────────────────────────────────────────

_HELP_ESTOP = [
    "  State: E-STOP",
    "",
    "  Enter       Clear E-stop → POSITIONING",
    "  Backspace   E-stop (already active)",
]

_HELP_POSITIONING = [
    "  State: POSITIONING  — move legs by hand, then lock each one",
    "",
    "  1           Capture FR (front-right) leg",
    "  2           Capture FL (front-left)  leg",
    "  3           Capture RR (rear-right)  leg",
    "  4           Capture RL (rear-left)   leg",
    "  Enter       Confirm all legs and stand up",
    "  Backspace   E-stop / abort",
    "",
    "  Lock all 4 legs before pressing Enter.",
]

_HELP_STANDING = [
    "  State: STANDING",
    "",
    "  Hold movement key + deadman (auto) to walk:",
    "  w / s       Forward / backward",
    "  a / d       Strafe left / right",
    "  q / e       Turn left / right",
    "  W/A/S/D/Q/E Turbo (uppercase)",
    "",
    "  SPACE       Standup recovery animation",
    "  g           Cycle gait",
    "  h           Switch to SHUFFLE gait",
    "  y           Toggle autonomous mode",
    "  j           Jump forward",
    "  b           Backflip",
    "  Backspace   E-stop",
]

_HELP_WALKING = [
    "  State: WALKING",
    "",
    "  w / s       Forward / backward",
    "  a / d       Strafe left / right",
    "  q / e       Turn left / right",
    "  W/A/S/D/Q/E Turbo (uppercase)",
    "",
    "  Release all keys to return to STANDING",
    "  SPACE       Standup recovery animation",
    "  g           Cycle gait",
    "  h           Switch to SHUFFLE gait",
    "  Backspace   E-stop",
]

_HELP_OTHER = [
    "  State: {}",
    "",
    "  Backspace   E-stop",
    "  r           Reset sim",
]

_HELP_FOOTER = [
    "",
    "  r           Reset sim (sim mode only)",
    "  Ctrl+C      Quit",
]


def _help_for_state(state: str):
    if state == RobotState.ESTOP:
        return _HELP_ESTOP + _HELP_FOOTER
    if state == RobotState.POSITIONING:
        return _HELP_POSITIONING + _HELP_FOOTER
    if state == RobotState.STANDING:
        return _HELP_STANDING + _HELP_FOOTER
    if state == RobotState.WALKING:
        return _HELP_WALKING + _HELP_FOOTER
    lines = [l.format(state) for l in _HELP_OTHER]
    return lines + _HELP_FOOTER


# ── Key sets ──────────────────────────────────────────────────────────────────

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
    gait_cycle_until_shuffle = 0
    gait_last_cycle_time = 0.0

    ONESHOT_COOLDOWN = 0.5
    SHUFFLE_PULSE_GAP = 0.2

    def _oneshot(btn):
        if now - btn_last_fire.get(btn, 0) > ONESHOT_COOLDOWN:
            btn_until[btn]     = now + 0.08
            btn_last_fire[btn] = now

    while node._running:
        key   = stdscr.getch()
        now   = time.monotonic()
        state = node.get_state()

        if key != -1:
            if state == RobotState.POSITIONING:
                # POSITIONING: number keys lock individual legs
                if key == ord('1'):
                    _oneshot(BTN_A)          # FR
                elif key == ord('2'):
                    _oneshot(BTN_B)          # FL
                elif key == ord('3'):
                    _oneshot(BTN_X)          # RR
                elif key == ord('4'):
                    _oneshot(BTN_Y)          # RL
                elif key in (curses.KEY_ENTER, ord('\n'), ord('\r')):
                    _oneshot(BTN_START)      # confirm + stand
                elif key in (curses.KEY_BACKSPACE, 127, 8):
                    _oneshot(BTN_BACK)       # abort → E-stop

            else:
                # All other states: standard mapping
                if key in _ALL_MOVE:
                    key_last[key] = now
                elif key == ord(' '):
                    _oneshot(BTN_A)          # sit / stand toggle
                elif key in (curses.KEY_BACKSPACE, 127, 8):
                    _oneshot(BTN_BACK)       # E-stop
                elif key in (curses.KEY_ENTER, ord('\n'), ord('\r')):
                    _oneshot(BTN_START)      # clear E-stop → POSITIONING
                elif key == ord('j'):
                    _oneshot(BTN_B)          # jump forward (standing/walking)
                elif key == ord('g'):
                    _oneshot(BTN_X)          # cycle gait
                elif key == ord('h'):
                    gait = node.get_gait_type()
                    if gait in _WALK_GAIT_CYCLE:
                        idx = _WALK_GAIT_CYCLE.index(gait)
                        gait_cycle_until_shuffle = idx
                    else:
                        gait_cycle_until_shuffle = 0
                elif key == ord('y'):
                    _oneshot(BTN_Y)          # toggle autonomous
                elif key == ord('b'):
                    _oneshot(BTN_B)          # backflip = B + RB
                    _oneshot(BTN_RB)
                elif key == ord('r'):
                    node.publish_reset()

        if gait_cycle_until_shuffle > 0 and now - gait_last_cycle_time >= SHUFFLE_PULSE_GAP:
            btn_until[BTN_X] = now + 0.08
            gait_last_cycle_time = now
            gait_cycle_until_shuffle -= 1

        held_lower = {chr(k) for k, t in key_last.items()
                      if k in _LOWER_MOVE and now - t < KEY_TIMEOUT}
        held_upper = {chr(k).lower() for k, t in key_last.items()
                      if k in _UPPER_MOVE and now - t < KEY_TIMEOUT}
        held  = held_lower | held_upper
        turbo = bool(held_upper)

        axes = [0.0] * 8
        if state not in (RobotState.POSITIONING, RobotState.ESTOP):
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

        # ── Draw UI ───────────────────────────────────────────────────────────
        stdscr.clear()
        h, w = stdscr.getmaxyx()

        title = "Dog Robot  —  Keyboard Teleop"
        try:
            stdscr.addstr(0, 0, title[:w - 1], curses.A_BOLD)
        except curses.error:
            pass

        help_lines = _help_for_state(state)
        for i, line in enumerate(help_lines):
            row = i + 2
            if row >= h - 2:
                break
            try:
                stdscr.addstr(row, 0, line[:w - 1])
            except curses.error:
                pass

        if state in (RobotState.STANDING, RobotState.WALKING):
            status = (
                f"  Gait: {node.get_gait_type()}  |  Keys: {', '.join(sorted(held)) or 'none'}"
                + ("  [TURBO]" if turbo else "")
            )
        elif state == RobotState.POSITIONING:
            status = "  Move legs by hand, press 1/2/3/4 to lock, Enter to stand"
        else:
            status = f"  State: {state}"

        try:
            stdscr.addstr(h - 1, 0, status[:w - 1], curses.A_REVERSE)
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
