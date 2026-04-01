"""
gait_generator.py
-----------------
Hierarchical gait control based on the MIT Cheetah algorithm:

  J. Lee, "Hierarchical controller for highly dynamic locomotion utilizing
  pattern modulation and impedance control," MIT SM Thesis, 2013.

Three layers:

  1. Gait Pattern Modulator
     - Assigns per-leg phase signals from a continuous master clock
     - Phase lags ΔS define gait pattern (trot, gallop, walk, crawl)
     - T_sw = constant (biological finding: swing duration is speed-invariant)
     - T_st = 2·L_span / v_d  (scales with commanded speed)

  2. Leg Trajectory Generator
     - Swing phase: 11th-degree Bezier curve (12 control points, Table 3.2)
     - Stance phase: sinusoidal wave with penetration depth δ

  3. Low-level leg compliance
     - Virtual impedance gains (Kp_r, Kd_r, Kp_θ, Kd_θ) are exposed for
       the Teensy impedance controller. Trajectory errors become joint torques
       via J_polar^T · [Kp·e + Kd·ė] (Eq. 3.17).

Foot positions returned in the HIP frame (mm):
  X  →  forward
  Y  →  outward (right-positive, left-negative)
  Z  ↓  downward (positive = foot lower)
"""

import math
import time
from enum import Enum, auto

from dog.robot_config import (
    STAND_HEIGHT, STEP_HEIGHT, STEP_LENGTH,
)

# ── Leg indices ────────────────────────────────────────────────────────────────
# 0 = FR (reference leg), 1 = FL, 2 = RR, 3 = RL
_LEG_SIDE = [1, -1, 1, -1]   # +1 = right, -1 = left


class GaitType(Enum):
    STAND  = auto()
    TURTLE = auto()
    CRAWL  = auto()
    WALK   = auto()
    TROT   = auto()
    GALLOP = auto()


# ── Physical speed at joystick magnitude 1.0 ──────────────────────────────────
MAX_SPEED_MS = 1.5        # m/s

# ── Swing duration — constant per biology (Maes et al. 2008) ──────────────────
# Trot/gallop: 0.25 s  |  Walk: 0.35 s  |  Crawl: 0.40 s  |  Turtle: 0.50 s
_T_SW = {
    GaitType.TROT:   0.25,
    GaitType.GALLOP: 0.25,
    GaitType.WALK:   0.35,
    GaitType.CRAWL:  0.40,
    GaitType.TURTLE: 0.50,
}

# ── Default stance duration for slow / stationary gaits ───────────────────────
_T_ST_SLOW = {
    GaitType.TROT:   0.50,
    GaitType.GALLOP: 0.40,
    GaitType.WALK:   0.50,
    GaitType.CRAWL:  0.60,
    GaitType.TURTLE: 1.50,   # ~75% duty cycle — one foot off at a time
}

# ── Phase offsets ΔS_i relative to FR (leg 0) — Eq. (3.7) / (3.8) ────────────
# Order: [FR, FL, RR, RL]
_PHASE_OFFSETS = {
    GaitType.TROT:   [0.00, 0.50, 0.50, 0.00],  # diagonal pairs
    GaitType.GALLOP: [0.00, 0.20, 0.55, 0.75],  # transverse gallop
    GaitType.WALK:   [0.00, 0.50, 0.25, 0.75],  # diagonal walk (duty ≈ 0.75)
    GaitType.CRAWL:  [0.00, 0.50, 0.75, 0.25],  # lateral-sequence (duty ≈ 0.80)
    GaitType.TURTLE: [0.00, 0.50, 0.75, 0.25],  # same pattern as crawl, much slower
}

# ── Stance penetration depth δ (mm) — generates vertical impulse via compliance
# MIT robot values (500 mm tall): δ_front = 36 mm, δ_rear = 10 mm.
# Scaled proportionally to our robot's stand height.
_HEIGHT_RATIO   = STAND_HEIGHT / 500.0
_DELTA_FRONT    = 36.0 * _HEIGHT_RATIO   # mm — front legs
_DELTA_REAR     = 10.0 * _HEIGHT_RATIO   # mm — rear  legs

# ── Half-stroke length L_span (mm) ────────────────────────────────────────────
# The foot travels from +L_span to -L_span during stance.
# L_span is also the x-axis scaling reference for the Bezier curve.
_L_SPAN_DEFAULT = STEP_LENGTH / 2   # mm (STEP_LENGTH is full stroke)

# ─────────────────────────────────────────────────────────────────────────────
# Bezier swing trajectory — 12 control points, MIT Table 3.2
# Reference geometry: L_span = 200 mm, ground level at y = 500 mm (y ↓)
# The trajectory is parameterised by S_sw ∈ [0, 1]:
#   S_sw = 0 → c0 = Lift-Off  (−L_span from neutral, on ground)
#   S_sw = 1 → c11 = Touch-Down (+L_span from neutral, on ground)
# ─────────────────────────────────────────────────────────────────────────────
_REF_CTRL = (
    (-200.0, 500.0),   # c0   LO — at ground, zero swing velocity
    (-280.5, 500.0),   # c1   follow-through (zero vel → direction change)
    (-300.0, 361.1),   # c2 ┐
    (-300.0, 361.1),   # c3 │ protraction — triple overlap → zero acceleration
    (-300.0, 361.1),   # c4 ┘
    (   0.0, 361.1),   # c5 ┐ mid-air
    (   0.0, 361.1),   # c6 ┘
    (   0.0, 321.4),   # c7   peak clearance (178.6 mm above ground)
    ( 303.2, 321.4),   # c8 ┐ retraction — triple overlap → zero acceleration
    ( 303.2, 321.4),   # c9 ┘
    ( 282.6, 500.0),   # c10  approach TD — zero swing velocity
    ( 200.0, 500.0),   # c11  TD — at ground
)
_REF_L_SPAN    = 200.0
_REF_GROUND    = 500.0
_REF_CLEARANCE = _REF_GROUND - 321.4   # 178.6 mm

# ─────────────────────────────────────────────────────────────────────────────
# Virtual leg impedance gains — Eq. (3.17)
# u = J_polar^T · [Kp_r·e_r + Kd_r·ė_r, Kp_θ·e_θ + Kd_θ·ė_θ]
# These are exposed on this object for the Teensy impedance controller.
# MIT experiment values: Kp_r=5000 N/m, Kd_r=100 Ns/m,
#                        Kp_θ=100 Nm/rad, Kd_θ=4 Nms/rad
# ─────────────────────────────────────────────────────────────────────────────
KP_RADIAL   = 5000.0   # N/m
KD_RADIAL   =  100.0   # Ns/m
KP_ANGULAR  =  100.0   # Nm/rad
KD_ANGULAR  =    4.0   # Nms/rad


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _bezier(ctrl_pts: tuple, t: float) -> tuple[float, float]:
    """Evaluate an 11th-degree Bezier curve at t ∈ [0,1] via de Casteljau."""
    pts = [[p[0], p[1]] for p in ctrl_pts]
    for _ in range(len(pts) - 1):
        pts = [
            [pts[j][0] + (pts[j + 1][0] - pts[j][0]) * t,
             pts[j][1] + (pts[j + 1][1] - pts[j][1]) * t]
            for j in range(len(pts) - 1)
        ]
    return pts[0][0], pts[0][1]


def _scale_ctrl_pts(l_span: float,
                    stand_z: float, step_h: float) -> list:
    """
    Scale reference Bezier control points to actual robot geometry.

    Scaling rules:
      x: proportional to l_span / L_span_ref
      z: ground → stand_z, peak → stand_z − step_h
    """
    x_scale = l_span / _REF_L_SPAN
    z_scale = step_h / _REF_CLEARANCE
    pts = []
    for rx, ry in _REF_CTRL:
        x = rx * x_scale
        # ry = _REF_GROUND → z = stand_z
        # ry = _REF_PEAK   → z = stand_z - step_h
        z = stand_z - (_REF_GROUND - ry) * z_scale
        pts.append((x, z))
    return pts


def default_foot_positions() -> list[tuple[float, float, float]]:
    """Neutral standing foot positions in the hip frame (mm).
    foot_y = 0 keeps the hip (abduction) motor flat against the body.
    Lateral offset is added only when strafing (vy != 0).
    """
    return [
        (0.0, 0.0, STAND_HEIGHT),  # FR
        (0.0, 0.0, STAND_HEIGHT),  # FL
        (0.0, 0.0, STAND_HEIGHT),  # RR
        (0.0, 0.0, STAND_HEIGHT),  # RL
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Main gait generator
# ─────────────────────────────────────────────────────────────────────────────

class GaitGenerator:
    """
    Stateful MIT-Cheetah hierarchical gait generator.

    Call update() at your control rate. It returns four (x, y, z) foot
    positions in the hip frame. The caller is responsible for running IK
    and sending motor commands.

    Typical usage:
        gen = GaitGenerator()
        gen.set_gait(GaitType.TROT)
        while running:
            positions = gen.update(vx, vy, yaw)
            angles = compute_all_legs(positions)
    """

    def __init__(self):
        self.gait_type = GaitType.STAND
        self.foot_pos  = default_foot_positions()

        # Impedance gains (read-only by external nodes)
        self.kp_radial  = KP_RADIAL
        self.kd_radial  = KD_RADIAL
        self.kp_angular = KP_ANGULAR
        self.kd_angular = KD_ANGULAR

        # Internal state
        self._last_time    = time.time()
        self._phase_clock  = 0.0   # master clock elapsed since last reset (s)
        self._t_st         = _T_ST_SLOW[GaitType.TROT]
        self._l_span       = _L_SPAN_DEFAULT

    # ── Public API ────────────────────────────────────────────────────────────

    def set_gait(self, gait_type: GaitType):
        """Switch to a new gait. Resets the phase clock."""
        self.gait_type = gait_type
        self._phase_clock = 0.0
        if gait_type == GaitType.STAND:
            self.foot_pos = default_foot_positions()

    def update(self, vx: float, vy: float, yaw: float,
               body_roll: float = 0.0, body_pitch: float = 0.0
               ) -> list[tuple[float, float, float]]:
        """
        Compute desired foot-end positions for the current control cycle.

        Parameters
        ----------
        vx  : forward velocity command, normalised [-1, 1]
        vy  : lateral velocity command, normalised [-1, 1]
        yaw : yaw rate command, normalised [-1, 1]
        body_roll, body_pitch : body tilt in degrees (from IMU)

        Returns
        -------
        List of four (x, y, z) tuples in the hip frame (mm).
        """
        now = time.time()
        dt  = max(now - self._last_time, 1e-4)
        self._last_time = now

        if self.gait_type == GaitType.STAND:
            return self._stand_pose(body_roll, body_pitch)

        if self.gait_type not in _PHASE_OFFSETS:
            return default_foot_positions()

        # ── Gait timing parameters ──────────────────────────────────────────
        t_sw  = _T_SW[self.gait_type]
        v_mag = math.hypot(vx, vy)

        if v_mag > 0.05:
            # Speed-adaptive stance duration: T_st = 2·L_span / v_d (Eq. 3.1)
            # When strafing, l_span is scaled up by (1 + |dir_y|); the timing
            # formula must use the same scaled span so body speed stays correct.
            lat_frac = abs(vy) / v_mag   # fraction of motion that is lateral
            span_for_timing = self._l_span * (1.0 + lat_frac)
            v_phys_mms = v_mag * MAX_SPEED_MS * 1000.0   # mm/s
            t_st = max(2.0 * span_for_timing / v_phys_mms, t_sw * 0.5)
            # Backward motion needs longer stance to prevent tipping
            if vx < 0.0:
                t_st = max(t_st, t_sw)
        else:
            t_st = _T_ST_SLOW[self.gait_type]

        t_stride = t_st + t_sw
        self._t_st = t_st

        # ── Advance master clock and wrap within one stride ─────────────────
        self._phase_clock = (self._phase_clock + dt) % t_stride

        phase_offsets = _PHASE_OFFSETS[self.gait_type]
        positions = []

        for leg in range(4):
            pos = self._leg_position(
                leg, phase_offsets[leg], t_st, t_sw, t_stride, vx, vy, yaw
            )
            positions.append(pos)

        return self._apply_body_pose(positions, body_roll, body_pitch)

    # ── Gait pattern modulator ────────────────────────────────────────────────

    def _leg_position(self, leg: int, ds: float,
                      t_st: float, t_sw: float, t_stride: float,
                      vx: float, vy: float, yaw: float
                      ) -> tuple[float, float, float]:
        """
        Compute foot position for one leg using its phase signal.

        Phase signal (Eq. 3.4):
            t_i = (t_elapsed − ΔS_i · T_stride)  mod  T_stride
            t_i ∈ [0, T_st)    → stance   (S^st = t_i / T_st)
            t_i ∈ [T_st, T_stride) → swing (S^sw = (t_i − T_st) / T_sw)
        """
        t_i = (self._phase_clock - ds * t_stride) % t_stride

        # Per-leg step parameters
        side      = _LEG_SIDE[leg]
        is_front  = (leg < 2)
        # Scale l_span up when strafing: the hip alone has a much shorter
        # lever arm than shoulder+knee, so lateral strides need to be wider
        # to generate the same lateral body displacement per cycle.
        base_span = self._l_span + yaw * side * self._l_span * 0.4
        base_span = max(base_span, 5.0)   # avoid zero/negative span

        # Normalised stride direction in the (forward, lateral) plane.
        # Using side * dir_y in trajectory functions ensures all legs push
        # the body in the same world direction when strafing.
        v_total = math.hypot(vx, vy)
        if v_total > 0.02:
            dir_x = vx / v_total
            dir_y = vy / v_total
        else:
            dir_x, dir_y = 1.0, 0.0

        # Hip lever arm is shorter than shoulder/knee, so double the stride
        # length when strafing so each step produces the same body displacement.
        l_span = base_span * (1.0 + abs(dir_y))

        p0_x = 0.0
        gl = 0.0

        if is_front:
            delta = _DELTA_FRONT
        else:
            if self.gait_type == GaitType.GALLOP:
                delta = 25.0 * _HEIGHT_RATIO
                p0_x = -50.0 if dir_x >= 0.0 else 0.0   # rear legs rearward only on forward motion
                gl   =  1.0 if dir_x >= 0.0 else 0.0    # gallop lean only on forward motion
            else:
                delta = _DELTA_REAR

        if t_i < t_st:
            s_st = t_i / t_st
            return self._stance_trajectory(leg, s_st, dir_x, dir_y, l_span, delta, p0_x, gl)
        else:
            s_sw = (t_i - t_st) / t_sw
            s_sw = max(0.0, min(1.0, s_sw))
            return self._swing_trajectory(leg, s_sw, dir_x, dir_y, l_span, p0_x)

    # ── Leg trajectory generator ──────────────────────────────────────────────

    def _stance_trajectory(self, leg: int, s_st: float, dir_x: float,
                           dir_y: float, l_span: float, delta: float,
                           p0_x: float, gl: float
                           ) -> tuple[float, float, float]:
        """
        Stance phase: sinusoidal reference trajectory. Eq. (3.12) / (3.13) / (4.1)

        The stride is generalised to 2D: the foot sweeps from +l_span to -l_span
        along the (dir_x, dir_y) direction.  Multiplying dir_y by the leg side
        ensures all four legs push the body in the same world direction when
        strafing (positive vy = strafe right).

        Horizontal:  stroke = L_span · (1 − 2·S^st)
                     foot_x = stroke · dir_x + P0_x
                     foot_y = stroke · side  · dir_y
        Vertical:    p_z = Z0 + δ · (cos(π·stroke / (2·L_span)) − GL/2 · sin(π·stroke / L_span))
        """
        stroke = l_span * (1.0 - 2.0 * s_st)
        foot_x = stroke * dir_x + p0_x
        foot_y = stroke * _LEG_SIDE[leg] * dir_y

        # Vertical: sinusoidal penetration below stand height
        if l_span > 1.0:
            foot_z = STAND_HEIGHT + delta * (
                math.cos(math.pi * stroke / (2.0 * l_span))
                - (gl / 2.0) * math.sin(math.pi * stroke / l_span)
            )
        else:
            foot_z = STAND_HEIGHT

        return foot_x, foot_y, foot_z

    def _swing_trajectory(self, leg: int, s_sw: float, dir_x: float,
                          dir_y: float, l_span: float, p0_x: float
                          ) -> tuple[float, float, float]:
        """
        Swing phase: 11th-degree Bezier curve. Eq. (3.9)

        Control points are scaled from the MIT reference geometry
        (L_span=200mm, stand=500mm) to our robot's dimensions.

        The Bezier x-axis (bx) represents the along-stride distance; it is
        rotated into the 2D (dir_x, dir_y) direction so the swing liftoff
        and touchdown positions match the start/end of the stance sweep.
        """
        ctrl = _scale_ctrl_pts(l_span, STAND_HEIGHT, STEP_HEIGHT)
        bx, bz = _bezier(ctrl, s_sw)

        foot_x = bx * dir_x + p0_x
        foot_y = bx * _LEG_SIDE[leg] * dir_y

        return foot_x, foot_y, bz

    # ── Body pose adjustment ──────────────────────────────────────────────────

    def _stand_pose(self, roll: float, pitch: float
                    ) -> list[tuple[float, float, float]]:
        """Static standing pose with optional body tilt compensation."""
        r = math.radians(roll)
        p = math.radians(pitch)
        out = []
        for x, y, z in default_foot_positions():
            dx = z * math.sin(p)
            dz = y * math.sin(r) + x * math.sin(p)
            out.append((x + dx, y, z + dz))
        return out

    def _apply_body_pose(self, positions: list, roll: float, pitch: float
                         ) -> list[tuple[float, float, float]]:
        """Adjust foot positions for body roll/pitch from IMU."""
        r = math.radians(roll)
        p = math.radians(pitch)
        out = []
        for x, y, z in positions:
            dz = y * math.sin(r) + x * math.sin(p)
            out.append((x, y, z + dz))
        return out
