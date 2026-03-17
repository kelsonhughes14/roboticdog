"""
gait_generator.py
-----------------
Generates foot trajectories for different gaits.

Supported gaits:
  - STAND  : All feet on the ground, no movement.
  - CRAWL  : Lateral sequence (RL→FL→RR→FR). Strict 3-point support. Slowest, most stable.
  - WALK   : Diagonal sequence (RL→FR→RR→FL). 2-3 point support. Faster than crawl.
  - TROT   : Diagonal pairs move simultaneously. 2-point support. Fastest.

Foot positions are returned in the HIP frame (mm):
  X → forward, Y → outward, Z → downward (positive = foot lower)
"""

import math
import time
from enum import Enum, auto
from spotmicro.robot_config import (
    BODY_LENGTH, BODY_WIDTH,
    STAND_HEIGHT, REAR_STAND_HEIGHT, STEP_HEIGHT, STEP_LENGTH, STEP_DURATION
)


class GaitType(Enum):
    STAND = auto()
    CRAWL = auto()
    WALK  = auto()
    TROT  = auto()


# Walk gait constants
# Diagonal-sequence offsets per leg [FR, FL, RR, RL]:
#   RL(0.00) → FR(0.25) → RR(0.50) → FL(0.75)
# Each hind leg is followed by the contralateral (opposite) front leg.
# Duty 0.65: swing fraction 0.35 > offset 0.25 → adjacent legs briefly
# overlap in swing, giving 2-limb support ~10% of each beat.
_WALK_OFFSETS = [0.25, 0.75, 0.50, 0.00]   # [FR, FL, RR, RL]
_WALK_DUTY    = 0.65   # fraction of cycle each foot is on the ground


# Default resting foot positions relative to each hip pivot (mm)
# Order: [FR, FL, RR, RL]
def default_foot_positions() -> list[tuple[float, float, float]]:
    return [
        (0.0,  BODY_WIDTH / 2,  STAND_HEIGHT),       # FR
        (0.0, -BODY_WIDTH / 2,  STAND_HEIGHT),       # FL
        (0.0,  BODY_WIDTH / 2,  REAR_STAND_HEIGHT),  # RR — deeper reach levels the body
        (0.0, -BODY_WIDTH / 2,  REAR_STAND_HEIGHT),  # RL
    ]


def _swing_trajectory(phase: float, start: tuple, end: tuple,
                      step_height: float) -> tuple[float, float, float]:
    """
    Compute foot position along a swing (in-air) trajectory.

    phase : 0.0 → 1.0  (0 = start, 1 = end of swing)
    Uses a half-sine arc for the vertical lift.
    """
    t = max(0.0, min(1.0, phase))
    x = start[0] + (end[0] - start[0]) * t
    y = start[1] + (end[1] - start[1]) * t
    z_base = start[2] + (end[2] - start[2]) * t
    z_lift = step_height * math.sin(math.pi * t)
    z = z_base - z_lift   # subtract because Z is downward
    return x, y, z


class GaitGenerator:
    """
    Stateful gait generator. Call update() every control cycle with the
    current velocity command to get new foot positions.
    """

    def __init__(self):
        self.gait_type   = GaitType.STAND
        self.foot_pos    = default_foot_positions()
        self.phase       = [0.0, 0.0, 0.0, 0.0]   # per-leg phase 0→1
        self.last_time   = time.time()

        # Crawl: lateral sequence — hind leg first, then same-side front leg
        # Trot: diagonal pairs (0,3) and (1,2)
        self._crawl_order = [3, 1, 2, 0]   # RL → FL → RR → FR
        self._crawl_active_leg = 0
        self._crawl_leg_phase  = 0.0

        # Walk: phase-based, one leg at a time
        self._walk_phase = 0.0

        # Trot: two simultaneous pairs
        self._trot_pair   = 0   # 0 = FR+RL swinging, 1 = FL+RR swinging
        self._trot_phase  = 0.0

    def set_gait(self, gait_type: GaitType):
        self.gait_type = gait_type
        if gait_type == GaitType.STAND:
            self.foot_pos = default_foot_positions()
        elif gait_type == GaitType.WALK:
            self._walk_phase = 0.0
        elif gait_type == GaitType.CRAWL:
            self._crawl_leg_phase  = 0.0
            self._crawl_active_leg = 0
        elif gait_type == GaitType.TROT:
            self._trot_phase = 0.0
            self._trot_pair  = 0

    def update(self, vx: float, vy: float, yaw: float,
               body_roll: float = 0.0, body_pitch: float = 0.0
               ) -> list[tuple[float, float, float]]:
        """
        Advance the gait by one time step.

        Parameters
        ----------
        vx        : forward velocity command  (-1.0 → 1.0)
        vy        : lateral velocity command  (-1.0 → 1.0)
        yaw       : yaw rate command          (-1.0 → 1.0)
        body_roll : desired body roll  (degrees)
        body_pitch: desired body pitch (degrees)

        Returns
        -------
        List of four (x, y, z) foot positions in hip frame.
        """
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        if self.gait_type == GaitType.STAND:
            return self._stand_pose(body_roll, body_pitch)

        # Scale commands
        sx   = vx  * STEP_LENGTH
        sy   = vy  * STEP_LENGTH
        syaw = yaw * STEP_LENGTH * 0.5

        if self.gait_type == GaitType.CRAWL:
            return self._crawl_update(dt, sx, sy, syaw, body_roll, body_pitch)
        elif self.gait_type == GaitType.WALK:
            return self._walk_update(dt, sx, sy, syaw, body_roll, body_pitch)
        elif self.gait_type == GaitType.TROT:
            return self._trot_update(dt, sx, sy, syaw, body_roll, body_pitch)

        return default_foot_positions()

    # ─────────────────────────────────────────
    # STAND POSE  (body lean only)
    # ─────────────────────────────────────────
    def _stand_pose(self, roll: float, pitch: float
                    ) -> list[tuple[float, float, float]]:
        r = math.radians(roll)
        p = math.radians(pitch)
        positions = []
        defaults = default_foot_positions()
        for i, (x, y, z) in enumerate(defaults):
            dx = z * math.sin(p)
            dz_roll  = y * math.sin(r)
            dz_pitch = x * math.sin(p)
            positions.append((x + dx, y, z + dz_roll + dz_pitch))
        return positions

    # ─────────────────────────────────────────
    # CRAWL GAIT
    # ─────────────────────────────────────────
    def _crawl_update(self, dt: float, sx: float, sy: float, syaw: float,
                      roll: float, pitch: float
                      ) -> list[tuple[float, float, float]]:
        defaults   = default_foot_positions()
        step_speed = 1.0 / STEP_DURATION

        self._crawl_leg_phase += dt * step_speed
        if self._crawl_leg_phase >= 1.0:
            self._crawl_leg_phase = 0.0
            self._crawl_active_leg = (
                (self._crawl_active_leg + 1) % 4
            )

        active = self._crawl_order[self._crawl_active_leg]
        positions = list(defaults)

        # Move all stance legs backward slightly
        for i in range(4):
            if i != active:
                dx = -sx * 0.25
                dy = -sy * 0.25
                x, y, z = positions[i]
                positions[i] = (x + dx, y + dy, z)

        # Swing the active leg forward
        start = defaults[active]
        end   = (
            defaults[active][0] + sx,
            defaults[active][1] + sy,
            defaults[active][2],
        )
        positions[active] = _swing_trajectory(
            self._crawl_leg_phase, start, end, STEP_HEIGHT
        )

        # Apply body tilt
        positions = self._apply_body_pose(positions, roll, pitch)
        return positions

    # ─────────────────────────────────────────
    # WALK GAIT
    # ─────────────────────────────────────────
    def _walk_update(self, dt: float, sx: float, sy: float, syaw: float,
                     roll: float, pitch: float
                     ) -> list[tuple[float, float, float]]:
        """
        Diagonal-sequence walk: each hind leg is followed by the contralateral
        front leg. Faster and more efficient than crawl.

        Sequence: RL(0.00) → FR(0.25) → RR(0.50) → FL(0.75)
        Duty 0.65: swing fraction 0.35 slightly exceeds the 0.25 offset, so
        adjacent legs briefly overlap in swing → 2-limb support for ~10% of
        each beat. Minimum support is always 2 limbs, typically 3.

        Full walk cycle = STEP_DURATION * 2 (between crawl × 4 and trot × 1).
        """
        step_speed = 1.0 / (STEP_DURATION * 2)
        self._walk_phase = (self._walk_phase + dt * step_speed) % 1.0

        defaults = default_foot_positions()
        leg_side  = [1, -1, 1, -1]   # +1 right legs, -1 left legs
        positions = []

        for i in range(4):
            step_x     = sx + syaw * leg_side[i]
            step_y     = sy
            local_ph   = (self._walk_phase - _WALK_OFFSETS[i]) % 1.0

            if local_ph < _WALK_DUTY:
                # Stance: foot sweeps from +step/2 (behind default) to -step/2
                t = local_ph / _WALK_DUTY
                x = defaults[i][0] + step_x * (0.5 - t)
                y = defaults[i][1] + step_y * (0.5 - t)
                positions.append((x, y, defaults[i][2]))
            else:
                # Swing: arc from -step/2 back to +step/2
                swing_ph = (local_ph - _WALK_DUTY) / (1.0 - _WALK_DUTY)
                start = (defaults[i][0] - step_x * 0.5,
                         defaults[i][1] - step_y * 0.5,
                         defaults[i][2])
                end   = (defaults[i][0] + step_x * 0.5,
                         defaults[i][1] + step_y * 0.5,
                         defaults[i][2])
                positions.append(_swing_trajectory(swing_ph, start, end, STEP_HEIGHT))

        return self._apply_body_pose(positions, roll, pitch)

    # ─────────────────────────────────────────
    # TROT GAIT
    # ─────────────────────────────────────────
    def _trot_update(self, dt: float, sx: float, sy: float, syaw: float,
                     roll: float, pitch: float
                     ) -> list[tuple[float, float, float]]:
        defaults   = default_foot_positions()
        step_speed = 1.0 / (STEP_DURATION * 0.5)

        self._trot_phase += dt * step_speed
        if self._trot_phase >= 1.0:
            self._trot_phase = 0.0
            self._trot_pair  = 1 - self._trot_pair

        # Pair 0: FR (0) + RL (3) swing;  FL (1) + RR (2) stance
        # Pair 1: FL (1) + RR (2) swing;  FR (0) + RL (3) stance
        if self._trot_pair == 0:
            swing_legs  = [0, 3]
            stance_legs = [1, 2]
        else:
            swing_legs  = [1, 2]
            stance_legs = [0, 3]

        # +1 for right legs (FR=0, RR=2), -1 for left legs (FL=1, RL=3)
        leg_side = [1, -1, 1, -1]

        positions = list(defaults)

        for i in stance_legs:
            # Per-leg step includes yaw contribution
            step_x = sx + syaw * leg_side[i]
            step_y = sy
            # Stance sweeps from +step/2 at phase=0 to -step/2 at phase=1,
            # propelling the body forward.  Starts exactly where the previous
            # swing ended, so there is no discontinuity at phase transitions.
            positions[i] = (
                defaults[i][0] + step_x * 0.5 * (1.0 - 2.0 * self._trot_phase),
                defaults[i][1] + step_y * 0.5 * (1.0 - 2.0 * self._trot_phase),
                defaults[i][2],
            )

        for i in swing_legs:
            step_x = sx + syaw * leg_side[i]
            step_y = sy
            # Swing goes from the stance-end position back to the stance-start,
            # symmetric around the default foot position.
            start = (
                defaults[i][0] - step_x * 0.5,
                defaults[i][1] - step_y * 0.5,
                defaults[i][2],
            )
            end = (
                defaults[i][0] + step_x * 0.5,
                defaults[i][1] + step_y * 0.5,
                defaults[i][2],
            )
            positions[i] = _swing_trajectory(
                self._trot_phase, start, end, STEP_HEIGHT
            )

        positions = self._apply_body_pose(positions, roll, pitch)
        return positions

    # ─────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────
    def _apply_body_pose(self, positions: list, roll: float, pitch: float
                         ) -> list:
        r = math.radians(roll)
        p = math.radians(pitch)
        out = []
        for x, y, z in positions:
            dz = y * math.sin(r) + x * math.sin(p)
            out.append((x, y, z + dz))
        return out
