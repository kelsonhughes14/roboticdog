"""
calibration_node.py
-------------------
Interactive servo calibration tool for Spot Micro.

Run servo_node first (Terminal 1):
  ros2 launch spotmicro calibration_launch.py

Then run this tool (Terminal 2):
  ros2 run spotmicro calibration_node

Phase 1 — Channel test:
  Wiggles each servo individually so you can confirm the right
  physical joint moved and it is on the correct channel.

Phase 2 — Neutral pose tuning:
  All servos hold the standing pose. Select any servo by number
  and nudge it with +/- commands until the robot stands correctly.

Phase 3 — Save:
  Writes the updated NEUTRAL_ANGLES back into robot_config.py.
  Run 'colcon build --symlink-install' afterwards to apply.
"""

import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from spotmicro.robot_config import (
    NEUTRAL_ANGLES,
    SERVO_ANGLE_MAX,
    SERVO_ANGLE_MIN,
    SERVO_CHANNEL_MAP,
)

# ── Servo labels in flat order [0-11] ────────────────────────────────────────
NAMES = [
    'FR Hip',      'FR Shoulder', 'FR Knee',
    'FL Hip',      'FL Shoulder', 'FL Knee',
    'RR Hip',      'RR Shoulder', 'RR Knee',
    'RL Hip',      'RL Shoulder', 'RL Knee',
]

# What to look for when each servo is at its mechanical zero (IK reference)
ZERO_HINTS = [
    'Hip bracket level — upper leg hangs straight down from body side',
    'Upper leg (femur) points straight down toward floor',
    'Lower leg (tibia) fully extended, inline with upper leg',
] * 4   # same pattern repeats for all four legs

# Build flat channel lookup: index → (board, channel)
_CHANNEL_INFO: dict[int, tuple[int, int]] = {
    leg * 3 + joint: (board, ch)
    for (leg, joint), (board, ch) in SERVO_CHANNEL_MAP.items()
}


def _clamp(v: float) -> float:
    return max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, v))


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')
        self._pub = self.create_publisher(Float32MultiArray, 'servo_angles', 10)
        self._angles = list(NEUTRAL_ANGLES)
        self._connected = False

    def wait_for_servo_node(self, timeout_s: float = 5.0) -> bool:
        """Block until servo_node subscribes to /servo_angles (or timeout)."""
        print('  Waiting for servo_node to connect…', end='', flush=True)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pub.get_subscription_count() > 0:
                self._connected = True
                print(' connected.')
                return True
        print(' TIMEOUT — is servo_node running?')
        return False

    def send(self, angles: list[float], move_s: float = 0.5) -> None:
        """Publish angles and sleep long enough for servos to physically move."""
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(move_s)   # MG996R needs ~0.17 s/60° — 0.5 s is safe


# ── Phase 1 — channel test ────────────────────────────────────────────────────

def phase1_channel_test(node: CalibrationNode) -> bool:
    print('\n' + '─' * 56)
    print('  PHASE 1 — Channel Verification')
    print('─' * 56)
    print('  Each servo will wiggle. Confirm the correct joint moved.')
    print('  Commands:  [Enter] = correct   s = skip   q = quit\n')

    if not node.wait_for_servo_node():
        return False

    for i, name in enumerate(NAMES):
        board, ch = _CHANNEL_INFO[i]

        # Start from all-neutral
        angles = [90.0] * 12

        print(f'  [{i + 1:2d}/12]  {name:<15}  (Board {board}, Channel {ch})')
        print( '         Wiggling…', end='', flush=True)

        # Wiggle: 70° → 110° → 90°
        for target in (70.0, 110.0, 90.0):
            angles[i] = target
            node.send(angles, move_s=0.5)
        print(' done.')

        resp = input('         Correct joint moved? [Enter / s=skip / q=quit]: '
                     ).strip().lower()
        print()
        if resp == 'q':
            return False
        # 's' just continues to the next servo

    print('  Channel verification complete.\n')
    return True


# ── Phase 2 — neutral pose tuning ────────────────────────────────────────────

def _print_pose_table(angles: list[float], selected: int) -> None:
    print()
    print(f'  {"#":>2}  {"Servo":<15}  {"Angle":>7}  {"Δ from default":>14}')
    print(f'  {"─"*2}  {"─"*15}  {"─"*7}  {"─"*14}')
    for i, name in enumerate(NAMES):
        marker = '>>>' if i == selected else '   '
        delta  = angles[i] - NEUTRAL_ANGLES[i]
        delta_str = f'{delta:+.1f}°' if abs(delta) > 0.05 else '—'
        print(f'  {marker} {i:2d}  {name:<15}  {angles[i]:6.1f}°  {delta_str:>14}')
    print()
    print(f'  Selected: [{selected}] {NAMES[selected]}')
    print()


def phase2_neutral_tune(node: CalibrationNode) -> list[float] | None:
    print('─' * 56)
    print('  PHASE 2 — Standing Pose Tuning')
    print('─' * 56)
    print('  Adjust each servo until the robot stands correctly.')
    print()
    print('  Commands:')
    print('    <number>      select servo (0–11)')
    print('    +N  / -N      nudge by N degrees  (e.g. +5, -2.5)')
    print('    =N            set absolute angle  (e.g. =90)')
    print('    r             reset selected servo to default')
    print('    t             print table again')
    print('    done          finish and move to save step')
    print('    quit          exit without saving')
    print()

    working = list(NEUTRAL_ANGLES)
    node.send(working, move_s=0.3)

    selected = 0
    _print_pose_table(working, selected)

    while True:
        try:
            raw = input(f'  [servo {selected} | {NAMES[selected]}] > ').strip()
        except (EOFError, KeyboardInterrupt):
            return None

        cmd = raw.lower()

        if cmd in ('q', 'quit'):
            return None

        if cmd in ('d', 'done'):
            return working

        if cmd in ('t', ''):
            _print_pose_table(working, selected)
            continue

        # Select servo by number
        if re.fullmatch(r'\d+', cmd):
            idx = int(cmd)
            if 0 <= idx <= 11:
                selected = idx
                print(f'  → Selected [{selected}] {NAMES[selected]}  '
                      f'(current: {working[selected]:.1f}°)')
                print(f'    Hint: {ZERO_HINTS[selected]}')
            else:
                print('  Invalid servo number. Choose 0–11.')
            continue

        # Nudge  (+N / -N)
        m = re.fullmatch(r'([+-])(\d+\.?\d*)', cmd)
        if m:
            delta = float(m.group(1) + m.group(2))
            working[selected] = _clamp(working[selected] + delta)
            node.send(working, move_s=0.3)
            print(f'  {NAMES[selected]} → {working[selected]:.1f}°')
            continue

        # Absolute set  (=N)
        m = re.fullmatch(r'=(\d+\.?\d*)', cmd)
        if m:
            working[selected] = _clamp(float(m.group(1)))
            node.send(working, move_s=0.3)
            print(f'  {NAMES[selected]} → {working[selected]:.1f}°')
            continue

        # Reset to default
        if cmd == 'r':
            working[selected] = NEUTRAL_ANGLES[selected]
            node.send(working, move_s=0.3)
            print(f'  {NAMES[selected]} reset → {working[selected]:.1f}°')
            continue

        print('  Unknown command.')

    return None


# ── Phase 3 — save ────────────────────────────────────────────────────────────

def _save_neutral_angles(new_angles: list[float]) -> bool:
    config_path = os.path.join(os.path.dirname(__file__), 'robot_config.py')

    try:
        with open(config_path, 'r') as f:
            source = f.read()
    except OSError as e:
        print(f'  Could not read robot_config.py: {e}')
        return False

    leg_labels = ['FR', 'FL', 'RR', 'RL']
    lines = []
    for leg in range(4):
        h, u, l = (new_angles[leg * 3 + j] for j in range(3))
        lines.append(f'    {h:.1f}, {u:.1f}, {l:.1f},'
                     f'   # {leg_labels[leg]}: hip, upper, lower')

    new_block = 'NEUTRAL_ANGLES = [\n' + '\n'.join(lines) + '\n]'

    updated = re.sub(
        r'NEUTRAL_ANGLES\s*=\s*\[.*?\]',
        new_block,
        source,
        flags=re.DOTALL,
    )

    if updated == source:
        print('  ERROR: Could not locate NEUTRAL_ANGLES block in robot_config.py.')
        print('  Update it manually:')
        for i, name in enumerate(NAMES):
            print(f'    {name}: {new_angles[i]:.1f}°')
        return False

    try:
        with open(config_path, 'w') as f:
            f.write(updated)
    except OSError as e:
        print(f'  Could not write robot_config.py: {e}')
        return False

    print(f'  Saved → {config_path}')
    return True


def phase3_save(new_angles: list[float]) -> None:
    print('─' * 56)
    print('  PHASE 3 — Save Results')
    print('─' * 56)

    changed = [(i, NEUTRAL_ANGLES[i], new_angles[i])
               for i in range(12) if abs(new_angles[i] - NEUTRAL_ANGLES[i]) > 0.05]

    if not changed:
        print('  No changes from defaults — nothing to save.')
        return

    print('  Changes:')
    for i, old, new in changed:
        print(f'    {NAMES[i]:<15}  {old:.1f}° → {new:.1f}°  ({new - old:+.1f}°)')
    print()

    resp = input('  Save to robot_config.py? [y/N]: ').strip().lower()
    if resp != 'y':
        print('  Aborted — no changes written.')
        return

    if _save_neutral_angles(new_angles):
        print()
        print('  Done! Rebuild to apply:')
        print('    colcon build --packages-select spotmicro --symlink-install')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    print()
    print('=' * 56)
    print('  Spot Micro — Servo Calibration Tool')
    print('=' * 56)
    print()
    print('  Make sure servo_node is already running:')
    print('    ros2 launch spotmicro calibration_launch.py')
    print()

    input('  Press Enter to start...')

    try:
        # Phase 1
        if not phase1_channel_test(node):
            print('  Aborted.')
            return

        # Phase 2
        new_angles = phase2_neutral_tune(node)
        if new_angles is None:
            print('  Aborted — no changes saved.')
            return

        # Phase 3
        phase3_save(new_angles)

    except KeyboardInterrupt:
        print('\n  Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
