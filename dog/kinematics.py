"""
kinematics.py
-------------
Inverse kinematics solver for a 3-DOF leg.

Coordinate system (body frame):
  X  →  forward
  Y  →  left
  Z  ↑  up

Each leg has three joints:
  1. Hip      (abduction / adduction   — rotates in the YZ plane)
  2. Shoulder (flexion / extension     — rotates in the XZ sagittal plane)
  3. Knee     (flexion                 — rotates in the XZ sagittal plane)

Returns joint angles in RADIANS relative to mechanical zero.
Motor command = direction * ik_angle_rad + radians(offset_deg)

Raises IKError when the target point is unreachable.
"""

import math
from dog.robot_config import (
    HIP_LENGTH, UPPER_LENGTH, LOWER_LENGTH,
    JOINT_DIRECTION, JOINT_OFFSETS,
    JOINT_ANGLE_MIN, JOINT_ANGLE_MAX,
)


class IKError(Exception):
    """Raised when a foot position is outside the reachable workspace."""
    pass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def solve_leg_ik(foot_x: float, foot_y: float, foot_z: float,
                 leg_side: int = 1) -> tuple[float, float, float]:
    """
    Compute the three joint angles for a single leg.

    Parameters
    ----------
    foot_x : float
        Foot X position in the hip frame (forward positive, mm).
    foot_y : float
        Foot Y position in the hip frame (outward positive, mm).
    foot_z : float
        Foot Z position in the hip frame (downward positive, mm).
    leg_side : int
        +1 for right legs, -1 for left legs (mirrors hip direction).

    Returns
    -------
    (hip_angle, shoulder_angle, knee_angle) in DEGREES (IK frame).
    hip_angle      : abduction angle  (0° = straight out)
    shoulder_angle : upper leg angle  (0° = vertical)
    knee_angle     : knee angle       (0° = straight, negative = bent)
    """

    # ── Hip (abduction) ──────────────────────────────────────────────
    # Positive foot_y = outward for any leg; JOINT_DIRECTION handles
    # the physical mirroring between right (+1) and left (-1) hips.
    hip_angle = math.degrees(math.atan2(foot_y, foot_z))

    hip_to_foot_yz = math.sqrt(foot_y ** 2 + foot_z ** 2)
    vertical_offset = math.sqrt(
        max(0.0, hip_to_foot_yz ** 2 - HIP_LENGTH ** 2)
    )

    # ── Reach from shoulder pivot to foot ────────────────────────────
    reach = math.sqrt(foot_x ** 2 + vertical_offset ** 2)

    max_reach = UPPER_LENGTH + LOWER_LENGTH
    min_reach = abs(UPPER_LENGTH - LOWER_LENGTH)
    if reach > max_reach:
        raise IKError(
            f"Foot ({foot_x:.1f}, {foot_y:.1f}, {foot_z:.1f}) out of reach "
            f"— distance {reach:.1f} mm > max {max_reach:.1f} mm"
        )
    if reach < min_reach + 1e-3:
        raise IKError(
            f"Foot too close to hip — distance {reach:.1f} mm < min {min_reach:.1f} mm"
        )

    # ── Knee angle using cosine rule ──────────────────────────────────
    cos_knee = (UPPER_LENGTH ** 2 + LOWER_LENGTH ** 2 - reach ** 2) / \
               (2.0 * UPPER_LENGTH * LOWER_LENGTH)
    cos_knee = clamp(cos_knee, -1.0, 1.0)
    knee_angle = math.degrees(math.acos(cos_knee)) - 180.0   # negative = bent

    # ── Shoulder angle ────────────────────────────────────────────────
    cos_sho = (UPPER_LENGTH ** 2 + reach ** 2 - LOWER_LENGTH ** 2) / \
              (2.0 * UPPER_LENGTH * reach)
    cos_sho = clamp(cos_sho, -1.0, 1.0)
    alpha = math.degrees(math.acos(cos_sho))
    beta  = -math.degrees(math.atan2(foot_x, vertical_offset))
    shoulder_angle = alpha + beta

    return hip_angle, shoulder_angle, knee_angle


def ik_to_joint_angles(shoulder_ik: float, knee_ik: float,
                        leg_index: int) -> tuple[float, float]:
    """
    Convert raw IK angles (degrees) to motor position commands (radians).
    Hip is omitted — physically replaced with a static dummy (8DOF).

    direction * ik_angle_rad + offset_rad = motor_cmd_rad
    A motor zeroed at mechanical zero receives 0.0 rad when the leg is
    at IK angle 0° (shoulder vertical, knee straight).

    Belt-drive knee coupling
    ------------------------
    The lower motor sits at the body and drives the knee via a timing belt
    along the upper leg (MIT Mini Cheetah layout). Its encoder position
    encodes the knee angle relative to the BODY, not relative to the upper leg.
    Therefore the motor must be commanded to (shoulder_ik + knee_ik) so that
    the knee-to-upper-leg angle equals knee_ik after the shoulder has rotated.

    Parameters
    ----------
    shoulder_ik, knee_ik : float — IK output angles in degrees.
    leg_index : int — 0=FR, 1=FL, 2=RR, 3=RL

    Returns
    -------
    (shoulder_rad, knee_rad) — motor position commands in radians.
    """
    def to_rad(ik_deg: float, leg: int, joint: int) -> float:
        direction = JOINT_DIRECTION.get((leg, joint), 1)
        offset_rad = math.radians(JOINT_OFFSETS.get((leg, joint), 0.0))
        cmd = direction * math.radians(ik_deg) + offset_rad
        return clamp(cmd, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX)

    # Belt drive: lower motor position = shoulder + knee in IK space.
    knee_coupled = shoulder_ik + knee_ik

    return (
        to_rad(shoulder_ik,  leg_index, 1),
        to_rad(knee_coupled, leg_index, 2),
    )


def compute_all_legs(foot_positions: list[tuple[float, float, float]]
                     ) -> list[float]:
    """
    Run IK for all four legs and return a flat list of 8 motor angles (radians).
    Hip is omitted — physically replaced with a static dummy (8DOF).

    Parameters
    ----------
    foot_positions : list of (x, y, z) tuples, one per leg.
        Order: [FR, FL, RR, RL]

    Returns
    -------
    List of 8 floats in radians:
        [FR_sho, FR_kne,
         FL_sho, FL_kne,
         RR_sho, RR_kne,
         RL_sho, RL_kne]
    """
    sides = [1, -1, 1, -1]   # right=+1, left=-1
    angles = []

    for i, (pos, side) in enumerate(zip(foot_positions, sides)):
        try:
            _, sho, kne = solve_leg_ik(*pos, leg_side=side)
            s_rad, k_rad = ik_to_joint_angles(sho, kne, i)
        except IKError:
            from dog.robot_config import NEUTRAL_ANGLES
            s_rad = NEUTRAL_ANGLES[i * 2]
            k_rad = NEUTRAL_ANGLES[i * 2 + 1]

        angles += [s_rad, k_rad]

    return angles
