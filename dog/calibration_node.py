"""
calibration_node.py
-------------------
Interactive CAN motor calibration tool for the Dog robot.

Calibration workflow
--------------------
Phase 1 — Motor ID verification:
  Each AK45-36 motor receives a small position command (+0.2 rad then back).
  Confirm the correct physical joint moved.

Phase 2 — Zero position:
  You physically position each joint at its mechanical zero:
    Hip:      leg pointing straight out laterally (no abduction)
    Shoulder: upper leg pointing straight down (vertical)
    Knee:     lower leg fully extended (inline with upper leg)
  Press Enter → the tool publishes the motor ID to /can_calibrate.
  The Teensy sends the "set zero position" CAN frame to that motor.
  The motor stores this as its permanent zero.

Phase 3 — Verification:
  All motors are commanded to 0.0 rad.  Each joint should be at mechanical zero.

Prerequisites
-------------
  1. Teensy firmware flashed and micro-ROS agent running
  2. All motors powered and in motor mode

Start the micro-ROS bridge first:
  ros2 launch dog calibration_launch.py

Then in another terminal:
  ros2 run dog calibration_node

Published topics:
  /joint_angles    (std_msgs/Float32MultiArray)  — 12 motor commands in radians
  /can_calibrate   (std_msgs/UInt8)              — motor ID to send "set zero"
  /can_enable      (std_msgs/Bool)               — enter / exit motor mode
"""

import math
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8, Bool

from dog.robot_config import (
    NEUTRAL_ANGLES,
    JOINT_ANGLE_MIN, JOINT_ANGLE_MAX,
    MOTOR_IDS,
)

NAMES = [
    'FR Hip',      'FR Shoulder', 'FR Knee',
    'FL Hip',      'FL Shoulder', 'FL Knee',
    'RR Hip',      'RR Shoulder', 'RR Knee',
    'RL Hip',      'RL Shoulder', 'RL Knee',
]

ZERO_HINTS = [
    'Hip bracket parallel to body — upper leg hangs straight down laterally',
    'Upper leg (femur) points straight DOWN toward the floor',
    'Lower leg (tibia) fully extended — inline with upper leg',
] * 4


def _clamp(v):
    return max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, v))


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')

        self._joint_pub   = self.create_publisher(Float32MultiArray, 'joint_angles',  10)
        self._cal_pub     = self.create_publisher(UInt8,             'can_calibrate', 10)
        self._enable_pub  = self.create_publisher(Bool,              'can_enable',    10)
        self._angles      = [0.0] * 12
        self._connected   = False

    def wait_for_teensy(self, timeout_s: float = 10.0) -> bool:
        print('  Waiting for Teensy (micro-ROS agent + /joint_angles)…', end='', flush=True)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_pub.get_subscription_count() > 0:
                self._connected = True
                print(' connected.')
                return True
        print(' TIMEOUT — is the micro-ROS agent running?')
        return False

    def enable_motors(self, enable: bool) -> None:
        msg = Bool()
        msg.data = enable
        self._enable_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.2)

    def send_angles(self, angles: list, move_s: float = 0.5) -> None:
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._joint_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(move_s)

    def send_zero_cmd(self, motor_id: int) -> None:
        """Tell the Teensy to send the 'set zero position' CAN frame to motor_id."""
        msg = UInt8()
        msg.data = motor_id
        self._cal_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.1)


# ── Phase 1 — motor ID verification ──────────────────────────────────────────

def phase1_verify(node: CalibrationNode) -> bool:
    print('\n' + '─' * 60)
    print('  PHASE 1 — Motor ID Verification')
    print('─' * 60)
    print('  Each motor will nudge +0.2 rad then return to 0.')
    print('  Confirm the correct physical joint moved.')
    print('  Commands:  [Enter] = correct   s = skip   q = quit\n')

    node.enable_motors(True)
    time.sleep(0.5)

    for i, name in enumerate(NAMES):
        motor_id = MOTOR_IDS[i]
        angles   = [0.0] * 12

        print(f'  [{i + 1:2d}/12]  {name:<16}  (Motor ID {motor_id})')
        print('         Nudging +0.2 rad…', end='', flush=True)

        angles[i] = 0.2
        node.send_angles(angles, move_s=0.6)
        angles[i] = 0.0
        node.send_angles(angles, move_s=0.5)
        print(' done.')

        resp = input('         Correct joint moved? [Enter / s=skip / q=quit]: '
                     ).strip().lower()
        print()
        if resp == 'q':
            return False

    print('  Motor ID verification complete.\n')
    return True


# ── Phase 2 — zero position ────────────────────────────────────────────────

def _position_joint_interactively(node: CalibrationNode, joint_idx: int,
                                   current_angles: list) -> bool:
    """Drive a single joint by typing degree values until the user confirms.

    Returns True when the user confirms (set zero), False to quit.
    Typing 's' skips this joint without setting zero.
    """
    print('         Commands:')
    print('           <angle>  — move this joint to that angle in degrees')
    print('                      (e.g.  20  or  -15)')
    print('           z        — confirm: set this position as zero')
    print('           s        — skip this joint')
    print('           q        — quit calibration\n')

    cmd_deg = 0.0

    while True:
        raw = input(f'         Current cmd: {cmd_deg:+.1f}°  > ').strip().lower()

        if raw == 'q':
            return None   # signal quit

        if raw == 's':
            return False  # signal skip

        if raw in ('z', ''):
            return True   # signal confirm

        try:
            cmd_deg = float(raw)
        except ValueError:
            print('         Invalid — enter a number in degrees, z, s, or q.')
            continue

        cmd_rad = math.radians(cmd_deg)
        cmd_rad = _clamp(cmd_rad)
        current_angles[joint_idx] = cmd_rad
        node.send_angles(current_angles, move_s=0.4)
        print(f'         Moved to {math.degrees(cmd_rad):+.1f}°  ({cmd_rad:+.4f} rad)')


def phase2_zero(node: CalibrationNode) -> bool:
    print('─' * 60)
    print('  PHASE 2 — Set Zero Position')
    print('─' * 60)
    print('  For each joint, type degree values to drive the motor to its')
    print('  mechanical zero, then type  z  to save that position as zero.\n')
    print('  Mechanical zeros:')
    print('    Hip      — leg bracket parallel to body, upper leg hangs straight down')
    print('    Shoulder — upper leg (femur) points straight DOWN to the floor')
    print('    Knee     — lower leg (tibia) fully extended, inline with upper leg\n')

    # Start all joints at 0 so they go to their current stored zero first
    current_angles = [0.0] * 12
    node.send_angles(current_angles, move_s=0.5)

    for i, name in enumerate(NAMES):
        motor_id = MOTOR_IDS[i]
        print(f'  [{i + 1:2d}/12]  {name:<16}  (Motor ID {motor_id})')
        print(f'         Target: {ZERO_HINTS[i]}')

        result = _position_joint_interactively(node, i, current_angles)

        if result is None:   # quit
            return False
        if result is False:  # skip
            print('         Skipped.\n')
            continue

        node.send_zero_cmd(motor_id)
        # MIT protocol: set-zero exits motor mode on that motor.
        # Re-enter motor mode on all motors so they hold position again.
        node.enable_motors(True)
        # After zeroing, the motor's internal position resets to 0 — reflect
        # that in our local tracker so subsequent joints are commanded relative
        # to the newly zeroed reference.
        current_angles[i] = 0.0
        node.send_angles(current_angles, move_s=0.3)
        print(f'         Zero set on Motor ID {motor_id}.\n')

    return True


# ── Phase 3 — verification ─────────────────────────────────────────────────

def phase3_verify(node: CalibrationNode) -> None:
    print('─' * 60)
    print('  PHASE 3 — Zero Position Verification')
    print('─' * 60)
    print('  Commanding all motors to 0.0 rad.')
    print('  Every joint should now be at its mechanical zero.')
    print('  Check: hips straight out, shoulders vertical, knees straight.\n')

    node.send_angles([0.0] * 12, move_s=1.0)

    input('  Inspect the robot, then press Enter to command NEUTRAL standing '
          'angles…')

    node.send_angles(list(NEUTRAL_ANGLES), move_s=1.0)
    print('  Robot should now be in the neutral standing pose.')
    print('  If any joint looks wrong, adjust JOINT_OFFSETS in robot_config.py.')
    print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    print()
    print('=' * 60)
    print('  Dog Robot — CAN Motor Calibration Tool')
    print('=' * 60)
    print()
    print('  Ensure the micro-ROS agent is running:')
    print('    ros2 launch dog calibration_launch.py')
    print()
    print('  SAFETY: Ensure there is nothing in the robot\'s sweep area.')
    print()

    input('  Press Enter to start…')

    try:
        if not node.wait_for_teensy():
            print('  Aborted — could not connect to Teensy.')
            return

        if not phase1_verify(node):
            print('  Aborted during Phase 1.')
            return

        if not phase2_zero(node):
            print('  Aborted during Phase 2.')
            return

        phase3_verify(node)
        print('  Calibration complete!')

    except KeyboardInterrupt:
        print('\n  Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
