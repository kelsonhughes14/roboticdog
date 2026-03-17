"""
kinematics.py
-------------
Inverse kinematics solver for a 3-DOF leg.

Coordinate system (body frame):
  X  →  forward
  Y  →  left
  Z  ↑  up

Each leg has three joints:
  1. Hip    (abduction / adduction  — rotates in the XZ plane)
  2. Upper  (flexion / extension    — rotates in the YZ plane)
  3. Lower  (knee flexion           — rotates in the YZ plane)

Returns joint angles in degrees relative to the servo neutral position.
Raises IKError when the target point is unreachable.
"""

import math
from spotmicro.robot_config import HIP_LENGTH, UPPER_LENGTH, LOWER_LENGTH


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
    (hip_angle, upper_angle, lower_angle) in degrees.
    hip_angle  : abduction angle  (0° = straight out)
    upper_angle: upper leg angle  (0° = vertical)
    lower_angle: knee angle       (0° = straight)
    """

    # ── Hip (abduction) ──────────────────────────────────────────────
    # Project foot onto the YZ plane to find the hip abduction angle.
    hip_angle = math.degrees(math.atan2(foot_y, foot_z)) * leg_side

    # Distance from hip pivot to foot, projected onto the sagittal plane
    # after accounting for the hip offset.
    hip_to_foot_yz = math.sqrt(foot_y ** 2 + foot_z ** 2)
    vertical_offset = math.sqrt(
        max(0.0, hip_to_foot_yz ** 2 - HIP_LENGTH ** 2)
    )

    # ── Reach from upper-leg pivot to foot ───────────────────────────
    # In the sagittal plane the upper pivot is at (0, 0) and the foot
    # is at (foot_x, vertical_offset).
    reach = math.sqrt(foot_x ** 2 + vertical_offset ** 2)

    # Check reachability
    max_reach = UPPER_LENGTH + LOWER_LENGTH
    min_reach = abs(UPPER_LENGTH - LOWER_LENGTH)
    if reach > max_reach:
        raise IKError(
            f"Foot position ({foot_x:.1f}, {foot_y:.1f}, {foot_z:.1f}) "
            f"is out of reach — distance {reach:.1f} mm > max {max_reach:.1f} mm"
        )
    if reach < min_reach + 1e-3:
        raise IKError(
            f"Foot position is too close to the hip — "
            f"distance {reach:.1f} mm < min {min_reach:.1f} mm"
        )

    # ── Knee angle (lower leg) using cosine rule ──────────────────────
    cos_knee = (UPPER_LENGTH ** 2 + LOWER_LENGTH ** 2 - reach ** 2) / \
               (2.0 * UPPER_LENGTH * LOWER_LENGTH)
    cos_knee = clamp(cos_knee, -1.0, 1.0)
    lower_angle = math.degrees(math.acos(cos_knee)) - 180.0  # negative = bent

    # ── Upper leg angle ───────────────────────────────────────────────
    cos_upper_part = (UPPER_LENGTH ** 2 + reach ** 2 - LOWER_LENGTH ** 2) / \
                     (2.0 * UPPER_LENGTH * reach)
    cos_upper_part = clamp(cos_upper_part, -1.0, 1.0)
    alpha = math.degrees(math.acos(cos_upper_part))
    beta  = math.degrees(math.atan2(foot_x, vertical_offset))
    upper_angle = alpha + beta

    return hip_angle, upper_angle, lower_angle


def ik_to_servo_angles(hip_ik: float, upper_ik: float, lower_ik: float,
                        leg_index: int) -> tuple[float, float, float]:
    """
    Convert raw IK angles (degrees) to servo command angles (0°–180°).

    The servo neutral position (90°) corresponds to the robot standing
    in its default pose. Offsets and directions from robot_config are
    applied here.
    """
    from spotmicro.robot_config import SERVO_OFFSETS, SERVO_DIRECTION

    def to_servo(ik_angle: float, leg: int, joint: int) -> float:
        direction = SERVO_DIRECTION.get((leg, joint), 1)
        offset    = SERVO_OFFSETS.get((leg, joint), 0.0)
        servo_angle = 90.0 + direction * ik_angle + offset
        return clamp(servo_angle, 0.0, 180.0)

    return (
        to_servo(hip_ik,   leg_index, 0),
        to_servo(upper_ik, leg_index, 1),
        to_servo(lower_ik, leg_index, 2),
    )


def compute_all_legs(foot_positions: list[tuple[float, float, float]]
                     ) -> list[float]:
    """
    Run IK for all four legs and return a flat list of 12 servo angles.

    Parameters
    ----------
    foot_positions : list of (x, y, z) tuples, one per leg.
        Order: [FR, FL, RR, RL]

    Returns
    -------
    List of 12 floats: [FR_hip, FR_upper, FR_lower,
                        FL_hip, FL_upper, FL_lower,
                        RR_hip, RR_upper, RR_lower,
                        RL_hip, RL_upper, RL_lower]
    """
    # Right legs: index 0, 2  →  leg_side = +1
    # Left  legs: index 1, 3  →  leg_side = -1
    sides = [1, -1, 1, -1]
    angles = []

    for i, (pos, side) in enumerate(zip(foot_positions, sides)):
        try:
            hip, upper, lower = solve_leg_ik(*pos, leg_side=side)
            servo_h, servo_u, servo_l = ik_to_servo_angles(
                hip, upper, lower, i
            )
        except IKError:
            # Fall back to neutral angles to avoid damaging servos
            from spotmicro.robot_config import NEUTRAL_ANGLES
            servo_h = NEUTRAL_ANGLES[i * 3]
            servo_u = NEUTRAL_ANGLES[i * 3 + 1]
            servo_l = NEUTRAL_ANGLES[i * 3 + 2]

        angles += [servo_h, servo_u, servo_l]

    return angles
