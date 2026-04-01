"""
keyboard_leg_node.py
--------------------
Curses-based keyboard control for individual leg joints.
Hold keys for continuous movement — all 3 joints controllable simultaneously.
Publishes directly to /joint_angles — no Xbox controller needed.

Controls
--------
  1 / 2 / 3 / 4   Select leg: FR / FL / RR / RL
  q / a            Hip      + / -
  w / s            Shoulder + / -
  e / d            Knee     + / -
  r                Reset selected leg to neutral
  t                Reset ALL legs to neutral
  [ / ]            Decrease / increase joint speed
  Ctrl+C           Quit

Published topics:
  /joint_angles  (std_msgs/Float32MultiArray)  — 12 motor commands in radians
"""

import curses
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from dog.robot_config import NEUTRAL_ANGLES, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX

LEG_NAMES   = ['FR', 'FL', 'RR', 'RL']
JOINT_NAMES = ['Hip', 'Shoulder', 'Knee']

# rad/s at full key-hold — adjustable with [ / ]
SPEED_OPTIONS = [0.1, 0.3, 0.5, 1.0, 2.0]

# How long after last keypress a key is considered "held" (seconds)
KEY_HOLD_WINDOW = 0.12

LOOP_HZ    = 20
PUBLISH_HZ = 20

# Motion keys: map key → (joint_index, direction)
MOTION_KEYS = {
    ord('q'): (0, +1),  # hip +
    ord('a'): (0, -1),  # hip -
    ord('w'): (1, +1),  # shoulder +
    ord('s'): (1, -1),  # shoulder -
    ord('e'): (2, +1),  # knee +
    ord('d'): (2, -1),  # knee -
}


def _idx(leg, joint):
    return leg * 3 + joint


def _clamp(v):
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, v))


class KeyboardLegNode(Node):

    def __init__(self):
        super().__init__('keyboard_leg_node')
        self._pub     = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self._lock    = threading.Lock()
        self._angles  = list(NEUTRAL_ANGLES)
        self._running = True
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def _publish(self):
        msg = Float32MultiArray()
        with self._lock:
            msg.data = list(self._angles)
        self._pub.publish(msg)

    def nudge(self, leg, joint, delta):
        with self._lock:
            i = _idx(leg, joint)
            self._angles[i] = _clamp(self._angles[i] + delta)

    def reset_leg(self, leg):
        with self._lock:
            for j in range(3):
                self._angles[_idx(leg, j)] = NEUTRAL_ANGLES[_idx(leg, j)]

    def reset_all(self):
        with self._lock:
            self._angles = list(NEUTRAL_ANGLES)

    def get_angles(self):
        with self._lock:
            return list(self._angles)

    def stop(self):
        self._running = False


def _run_curses(stdscr, node: KeyboardLegNode):
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    sel_leg   = 0
    speed_idx = 2        # default 0.5 rad/s
    key_last  = {}       # key → last-seen timestamp
    dt        = 1.0 / LOOP_HZ

    while node._running:
        key = stdscr.getch()
        now = time.monotonic()

        if key != -1:
            if key in MOTION_KEYS:
                key_last[key] = now
            elif key in (ord('1'), ord('2'), ord('3'), ord('4')):
                sel_leg = key - ord('1')
            elif key == ord('r'):
                node.reset_leg(sel_leg)
            elif key == ord('t'):
                node.reset_all()
            elif key == ord('['):
                speed_idx = max(0, speed_idx - 1)
            elif key == ord(']'):
                speed_idx = min(len(SPEED_OPTIONS) - 1, speed_idx + 1)

        # Apply motion for all held keys
        speed = SPEED_OPTIONS[speed_idx]
        held  = set()
        for k, t in key_last.items():
            if now - t < KEY_HOLD_WINDOW:
                held.add(k)
                joint, direction = MOTION_KEYS[k]
                node.nudge(sel_leg, joint, direction * speed * dt)

        # ── draw UI ──────────────────────────────────────────────────────────
        angles = node.get_angles()
        stdscr.clear()
        h, w = stdscr.getmaxyx()
        row = 0

        def addline(text='', attr=0):
            nonlocal row
            if row < h - 1:
                try:
                    stdscr.addstr(row, 0, text[:w - 1], attr)
                except curses.error:
                    pass
            row += 1

        addline("  Dog Robot — Keyboard Leg Test", curses.A_BOLD)
        addline("  1-4=leg   q/a=hip   w/s=shoulder   e/d=knee   r=reset   t=all   [/]=speed")
        addline()
        addline(f"  Leg: {LEG_NAMES[sel_leg]}   Speed: {speed:.1f} rad/s", curses.A_BOLD)
        addline()
        addline(f"  {'Leg':<4}  {'Hip':>8}  {'Shoulder':>9}  {'Knee':>8}")
        addline(f"  {'─'*4}  {'─'*8}  {'─'*9}  {'─'*8}")

        for leg in range(4):
            vals = [angles[_idx(leg, j)] for j in range(3)]
            active_keys = {MOTION_KEYS[k][0] for k in held} if leg == sel_leg else set()
            parts = []
            for joint, val in enumerate(vals):
                s = f"{val:+8.3f}"
                parts.append(('*' + s[1:] + '*') if joint in active_keys else s)
            marker = ">>" if leg == sel_leg else "  "
            addline(f"  {marker}{LEG_NAMES[leg]:<2}  {parts[0]}  {parts[1]}  {parts[2]}")

        stdscr.refresh()
        time.sleep(dt)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardLegNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(_run_curses, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.reset_all()
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
