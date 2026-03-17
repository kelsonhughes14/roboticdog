"""
test_kinematics.py
------------------
Unit tests for the IK solver.
Run with: pytest test/test_kinematics.py
"""

import sys
import os
import math

# Add package to path for direct testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'spotmicro'))

from kinematics import solve_leg_ik, compute_all_legs, IKError
from robot_config import STAND_HEIGHT, BODY_WIDTH


def test_neutral_position():
    """Foot directly below the hip should always solve."""
    hip, upper, lower = solve_leg_ik(0.0, BODY_WIDTH / 2, STAND_HEIGHT)
    assert isinstance(hip, float)
    assert isinstance(upper, float)
    assert isinstance(lower, float)


def test_forward_reach():
    """Foot in front of the robot should solve."""
    hip, upper, lower = solve_leg_ik(40.0, BODY_WIDTH / 2, STAND_HEIGHT)
    assert hip is not None


def test_out_of_reach_raises():
    """Foot too far away should raise IKError."""
    try:
        solve_leg_ik(500.0, 0.0, 500.0)
        assert False, "Expected IKError"
    except IKError:
        pass


def test_compute_all_legs_returns_12():
    """compute_all_legs should return exactly 12 angles."""
    foot_positions = [
        (0.0,  BODY_WIDTH / 2, STAND_HEIGHT),
        (0.0, -BODY_WIDTH / 2, STAND_HEIGHT),
        (0.0,  BODY_WIDTH / 2, STAND_HEIGHT),
        (0.0, -BODY_WIDTH / 2, STAND_HEIGHT),
    ]
    angles = compute_all_legs(foot_positions)
    assert len(angles) == 12
    for a in angles:
        assert 0.0 <= a <= 180.0, f"Angle {a} out of servo range"


def test_servo_angles_in_range():
    """All returned servo angles must be in [0, 180]."""
    foot_positions = [
        (30.0,  BODY_WIDTH / 2, STAND_HEIGHT + 20),
        (30.0, -BODY_WIDTH / 2, STAND_HEIGHT + 20),
        (-20.0, BODY_WIDTH / 2, STAND_HEIGHT),
        (-20.0,-BODY_WIDTH / 2, STAND_HEIGHT),
    ]
    angles = compute_all_legs(foot_positions)
    for a in angles:
        assert 0.0 <= a <= 180.0


if __name__ == '__main__':
    test_neutral_position()
    test_forward_reach()
    test_out_of_reach_raises()
    test_compute_all_legs_returns_12()
    test_servo_angles_in_range()
    print("All kinematics tests passed.")
