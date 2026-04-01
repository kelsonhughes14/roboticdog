"""
leg_test_node.py
----------------
Interactive CLI tool for moving individual joints while all others hold neutral.
Joint angles are in RADIANS (AK45-36 CAN motor commands).

Usage:
  ros2 run dog leg_test_node

Commands:
  fr / fl / rr / rl     select leg
  h / s / k             select joint (hip / shoulder / knee)
  +N  / -N              nudge selected joint by N radians  (e.g. +0.1, -0.05)
  =N                    set selected joint to absolute angle in radians  (e.g. =1.5)
  r                     reset selected leg to neutral
  ra                    reset all legs to neutral
  t                     print current angle table
  q                     quit

Published topics:
  /joint_angles  (std_msgs/Float32MultiArray)  — 12 motor commands in radians
"""

import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from dog.robot_config import NEUTRAL_ANGLES, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX, JOINT_DIRECTION

LEG_NAMES   = ['FR', 'FL', 'RR', 'RL']
LEG_ALIAS   = {'fr': 0, 'fl': 1, 'rr': 2, 'rl': 3,
               '0':  0, '1':  1, '2':  2, '3':  3}
JOINT_NAMES = ['Hip', 'Shoulder', 'Knee']
JOINT_ALIAS = {'h': 0, 'hip': 0,
               's': 1, 'shoulder': 1, 'sho': 1,
               'k': 2, 'knee': 2}


def _clamp(v):
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, v))


def _idx(leg, joint):
    return leg * 3 + joint


def _print_table(angles, sel_leg, sel_joint):
    print()
    print(f'  {"":3}  {"Leg":<4}  {"Joint":<10}  {"Angle (rad)":>11}  {"Δ neutral":>10}')
    print(f'  {"─"*3}  {"─"*4}  {"─"*10}  {"─"*11}  {"─"*10}')
    for leg in range(4):
        for joint in range(3):
            i      = _idx(leg, joint)
            marker = '>>>' if (leg == sel_leg and joint == sel_joint) else '   '
            delta  = angles[i] - NEUTRAL_ANGLES[i]
            dstr   = f'{delta:+.3f}' if abs(delta) > 0.001 else '—'
            print(f'  {marker}  {LEG_NAMES[leg]:<4}  {JOINT_NAMES[joint]:<10}'
                  f'  {angles[i]:10.3f}  {dstr:>10}')
        print()


class LegTestNode(Node):

    def __init__(self):
        super().__init__('leg_test_node')
        self._pub        = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self._enable_pub = self.create_publisher(Bool, 'can_enable', 10)
        self._angles     = list(NEUTRAL_ANGLES)

    def wait_for_teensy(self, timeout_s: float = 10.0) -> bool:
        print('  Waiting for Teensy (micro-ROS agent + /joint_angles)…', end='', flush=True)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pub.get_subscription_count() > 0:
                print(' connected.')
                return True
        print(' TIMEOUT — is the micro-ROS agent running?')
        return False

    def enable_motors(self) -> None:
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)
        time.sleep(0.1)   # give motors time to enter MIT mode

    def disable_motors(self) -> None:
        msg = Bool()
        msg.data = False
        self._enable_pub.publish(msg)
        time.sleep(0.05)

    def send(self, move_s: float = 0.3) -> None:
        msg = Float32MultiArray()
        motor = []
        for leg in range(4):
            for joint in range(3):
                i = leg * 3 + joint
                d = JOINT_DIRECTION.get((leg, joint), 1)
                motor.append(float(d * self._angles[i]))
        msg.data = motor
        self._pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(move_s)

    def reset_leg(self, leg: int) -> None:
        for j in range(3):
            self._angles[_idx(leg, j)] = NEUTRAL_ANGLES[_idx(leg, j)]

    def reset_all(self) -> None:
        self._angles = list(NEUTRAL_ANGLES)


def run(node: LegTestNode) -> None:
    sel_leg   = 0
    sel_joint = 0

    print()
    print('=' * 56)
    print('  Dog Robot — Single Leg Test  (angles in radians)')
    print('=' * 56)
    print()
    print('  Commands:')
    print('    fr/fl/rr/rl   select leg')
    print('    h / s / k     select joint (hip/shoulder/knee)')
    print('    +N / -N       nudge by N radians  (e.g. +0.1)')
    print('    =N            set absolute angle  (e.g. =1.5)')
    print('    r             reset selected leg to neutral')
    print('    ra            reset all legs to neutral')
    print('    t             print table')
    print('    q             quit')
    print()

    if not node.wait_for_teensy():
        return

    print('  Enabling motors (entering MIT mode)…')
    node.enable_motors()
    node.send(move_s=0.5)
    _print_table(node._angles, sel_leg, sel_joint)

    while True:
        prompt = f'  [{LEG_NAMES[sel_leg]} | {JOINT_NAMES[sel_joint]}] > '
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            break

        cmd = raw.lower()

        if cmd in ('q', 'quit'):
            break

        if cmd in LEG_ALIAS:
            sel_leg = LEG_ALIAS[cmd]
            i = _idx(sel_leg, sel_joint)
            print(f'  → Leg: {LEG_NAMES[sel_leg]}  '
                  f'(hip={node._angles[_idx(sel_leg,0)]:.3f}  '
                  f'sho={node._angles[_idx(sel_leg,1)]:.3f}  '
                  f'kne={node._angles[_idx(sel_leg,2)]:.3f} rad)')
            continue

        if cmd in JOINT_ALIAS:
            sel_joint = JOINT_ALIAS[cmd]
            i = _idx(sel_leg, sel_joint)
            print(f'  → Joint: {JOINT_NAMES[sel_joint]}  '
                  f'(current: {node._angles[i]:.3f} rad)')
            continue

        if cmd in ('t', ''):
            _print_table(node._angles, sel_leg, sel_joint)
            continue

        if cmd == 'r':
            node.reset_leg(sel_leg)
            node.send()
            print(f'  {LEG_NAMES[sel_leg]} reset to neutral.')
            continue

        if cmd == 'ra':
            node.reset_all()
            node.send()
            print('  All legs reset to neutral.')
            continue

        # Nudge: +N / -N
        m = re.fullmatch(r'([+-])(\d+\.?\d*)', cmd)
        if m:
            delta = float(m.group(1) + m.group(2))
            i = _idx(sel_leg, sel_joint)
            node._angles[i] = _clamp(node._angles[i] + delta)
            node.send()
            print(f'  {LEG_NAMES[sel_leg]} {JOINT_NAMES[sel_joint]} '
                  f'→ {node._angles[i]:.3f} rad')
            continue

        # Absolute set: =N
        m = re.fullmatch(r'=(-?\d+\.?\d*)', cmd)
        if m:
            i = _idx(sel_leg, sel_joint)
            node._angles[i] = _clamp(float(m.group(1)))
            node.send()
            print(f'  {LEG_NAMES[sel_leg]} {JOINT_NAMES[sel_joint]} '
                  f'→ {node._angles[i]:.3f} rad')
            continue

        print('  Unknown command. Type t to see table, q to quit.')

    print('\n  Returning all legs to neutral…')
    node.reset_all()
    node.send(move_s=0.5)
    node.disable_motors()


def main(args=None):
    rclpy.init(args=args)
    node = LegTestNode()
    try:
        run(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
