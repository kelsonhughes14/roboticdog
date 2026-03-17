"""
test_gait.py
------------
Unit tests for the gait generator.
Run with: pytest test/test_gait.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'spotmicro'))

from gait_generator import GaitGenerator, GaitType, default_foot_positions


def test_default_positions():
    positions = default_foot_positions()
    assert len(positions) == 4
    for pos in positions:
        assert len(pos) == 3


def test_stand_returns_four_positions():
    g = GaitGenerator()
    g.set_gait(GaitType.STAND)
    result = g.update(0.0, 0.0, 0.0)
    assert len(result) == 4


def test_trot_forward():
    g = GaitGenerator()
    g.set_gait(GaitType.TROT)
    for _ in range(20):
        result = g.update(1.0, 0.0, 0.0)
        assert len(result) == 4


def test_crawl_forward():
    g = GaitGenerator()
    g.set_gait(GaitType.CRAWL)
    for _ in range(20):
        result = g.update(1.0, 0.0, 0.0)
        assert len(result) == 4


def test_body_tilt():
    g = GaitGenerator()
    g.set_gait(GaitType.STAND)
    result = g.update(0.0, 0.0, 0.0, body_roll=10.0, body_pitch=5.0)
    assert len(result) == 4


if __name__ == '__main__':
    test_default_positions()
    test_stand_returns_four_positions()
    test_trot_forward()
    test_crawl_forward()
    test_body_tilt()
    print("All gait tests passed.")
