"""
leg_test_node.py
----------------
Interactive tool for moving a single leg while all others hold neutral.

Usage (after starting the micro-ROS agent):
  ros2 run spotmicro leg_test_node

Commands:
  fr / fl / rr / rl     select leg
  h / s / k             select joint (hip / shoulder / knee)
  +N  / -N              nudge selected joint by N degrees  (e.g. +5, -2.5)
  =N                    set selected joint to absolute angle  (e.g. =90)
  r                     reset selected leg to neutral
  ra                    reset all legs to neutral
  t                     print the current angle table
  q                     quit
"""

import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from spotmicro.robot_config import NEUTRAL_ANGLES, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX

LEG_NAMES  = ['FR', 'FL', 'RR', 'RL']
LEG_ALIAS  = {'fr': 0, 'fl': 1, 'rr': 2, 'rl': 3,
              '0':  0, '1':  1, '2':  2, '3':  3}

JOINT_NAMES = ['Hip', 'Shoulder', 'Knee']
JOINT_ALIAS = {'h': 0, 'hip': 0,
               's': 1, 'shoulder': 1, 'sho': 1,
               'k': 2, 'knee': 2}


def _clamp(v: float) -> float:
    return max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, v))


def _idx(leg: int, joint: int) -> int:
    return leg * 3 + joint


def _print_table(angles: list, sel_leg: int, sel_joint: int) -> None:
    print()
    print(f'  {"":3}  {"Leg":<4}  {"Joint":<10}  {"Angle":>7}  {"Δ neutral":>9}')
    print(f'  {"─"*3}  {"─"*4}  {"─"*10}  {"─"*7}  {"─"*9}')
    for leg in range(4):
        for joint in range(3):
            i      = _idx(leg, joint)
            marker = '>>>' if (leg == sel_leg and joint == sel_joint) else '   '
            delta  = angles[i] - NEUTRAL_ANGLES[i]
            dstr   = f'{delta:+.1f}°' if abs(delta) > 0.05 else '—'
            print(f'  {marker}  {LEG_NAMES[leg]:<4}  {JOINT_NAMES[joint]:<10}'
                  f'  {angles[i]:6.1f}°  {dstr:>9}')
        print()


class LegTestNode(Node):

    def __init__(self):
        super().__init__('leg_test_node')
        self._pub    = self.create_publisher(Float32MultiArray, 'servo_angles', 10)
        self._angles = list(NEUTRAL_ANGLES)

    def wait_for_teensy(self, timeout_s: float = 10.0) -> bool:
        print('  Waiting for Teensy to connect to /servo_angles…', end='', flush=True)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pub.get_subscription_count() > 0:
                print(' connected.')
                return True
        print(' TIMEOUT — is the micro-ROS agent running?')
        return False

    def send(self, move_s: float = 0.4) -> None:
        msg = Float32MultiArray()
        msg.data = [float(a) for a in self._angles]
        self._pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(move_s)

    def reset_leg(self, leg: int) -> None:
        for j in range(3):
            self._angles[_idx(leg, j)] = NEUTRAL_ANGLES[_idx(leg, j)]

    def reset_all(self) -> None:
        self._angles = list(NEUTRAL_ANGLES)


def run(node: LegTestNode) -> None:
    sel_leg   = 0   # FR
    sel_joint = 0   # Hip

    print()
    print('=' * 52)
    print('  Spot Micro — Single Leg Test')
    print('=' * 52)
    print()
    print('  Commands:')
    print('    fr/fl/rr/rl   select leg')
    print('    h / s / k     select joint (hip/shoulder/knee)')
    print('    +N / -N       nudge by N degrees')
    print('    =N            set absolute angle')
    print('    r             reset selected leg to neutral')
    print('    ra            reset all legs to neutral')
    print('    t             print table')
    print('    q             quit')
    print()

    if not node.wait_for_teensy():
        return

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

        # Select leg
        if cmd in LEG_ALIAS:
            sel_leg = LEG_ALIAS[cmd]
            i = _idx(sel_leg, sel_joint)
            print(f'  → Leg: {LEG_NAMES[sel_leg]}  '
                  f'(hip={node._angles[_idx(sel_leg,0)]:.1f}°  '
                  f'shoulder={node._angles[_idx(sel_leg,1)]:.1f}°  '
                  f'knee={node._angles[_idx(sel_leg,2)]:.1f}°)')
            continue

        # Select joint
        if cmd in JOINT_ALIAS:
            sel_joint = JOINT_ALIAS[cmd]
            i = _idx(sel_leg, sel_joint)
            print(f'  → Joint: {JOINT_NAMES[sel_joint]}  '
                  f'(current: {node._angles[i]:.1f}°)')
            continue

        # Print table
        if cmd in ('t', ''):
            _print_table(node._angles, sel_leg, sel_joint)
            continue

        # Reset selected leg
        if cmd == 'r':
            node.reset_leg(sel_leg)
            node.send()
            print(f'  {LEG_NAMES[sel_leg]} reset to neutral.')
            continue

        # Reset all
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
                  f'→ {node._angles[i]:.1f}°')
            continue

        # Absolute set: =N
        m = re.fullmatch(r'=(\d+\.?\d*)', cmd)
        if m:
            i = _idx(sel_leg, sel_joint)
            node._angles[i] = _clamp(float(m.group(1)))
            node.send()
            print(f'  {LEG_NAMES[sel_leg]} {JOINT_NAMES[sel_joint]} '
                  f'→ {node._angles[i]:.1f}°')
            continue

        print('  Unknown command. Type t to see table, q to quit.')

    print('\n  Returning all legs to neutral…')
    node.reset_all()
    node.send(move_s=0.5)


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
