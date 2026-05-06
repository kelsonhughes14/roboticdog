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
    BODY_LENGTH, BODY_WIDTH,
    UPPER_LENGTH, LOWER_LENGTH,
    GAIT_START_UPPER_ANGLE_DEG, GAIT_START_LOWER_ANGLE_DEG,
)

# ── Leg indices ────────────────────────────────────────────────────────────────
# 0 = FR (reference leg), 1 = FL, 2 = RR, 3 = RL
_LEG_SIDE = [1, -1, 1, -1]   # +1 = right, -1 = left


class GaitType(Enum):
    STAND   = auto()
    TURTLE  = auto()
    CRAWL   = auto()
    WALK    = auto()
    TROT    = auto()
    GALLOP  = auto()
    SHUFFLE    = auto()   # short-stride crawl: safe on rough/narrow terrain
    DIAG_TROT  = auto()   # tunable diagonal trot — FR+RL in sync, FL+RR in sync
    STEP       = auto()   # sequential one-leg-at-a-time gait


# ── Physical speed at joystick magnitude 1.0 ──────────────────────────────────
MAX_SPEED_MS = 1.5        # m/s

# ═════════════════════════════════════════════════════════════════════════════
# DIAG_TROT tuning — edit these values to adjust the diagonal trot gait.
# FR+RL move together; FL+RR move together (classic dog trot).
#
#   DIAG_TROT_STRIDE_MM   Half-stroke per step (mm). Full foot travel is 2×.
#                         Larger = longer steps, faster at the same cadence.
#                         Safe range: 20 – 80 mm.
#
#   DIAG_TROT_LIFT_MM     How high each foot rises during swing (mm).
#                         Larger = more clearance but more joint extension.
#                         Safe range: 30 – 100 mm.
#
#   DIAG_TROT_SWING_S     Swing (air) duration in seconds.
#                         Shorter = snappier steps; longer = smoother arc.
#                         Safe range: 0.15 – 0.40 s.
#
#   DIAG_TROT_STANCE_S    Stance duration when the joystick is centred / slow.
#                         Determines cadence at low speed.
#                         Safe range: 0.30 – 0.80 s.
#
#   DIAG_TROT_SPEED_SCALE Fraction of MAX_SPEED_MS (1.5 m/s) at full stick.
#                         0.5 → top speed ≈ 0.75 m/s.
#                         Safe range: 0.2 – 1.0.
# ═════════════════════════════════════════════════════════════════════════════
DIAG_TROT_STRIDE_MM   =  35.0   # mm  — half-stride length
DIAG_TROT_LIFT_MM     =  40.0   # mm  — foot lift height during swing
DIAG_TROT_SWING_S     =  0.32   # s   — time each foot spends in the air
DIAG_TROT_STANCE_S    =  0.30   # s   — stance duration at low/zero speed
DIAG_TROT_SPEED_SCALE =  0.30   # —   — fraction of MAX_SPEED_MS at full stick

# ═════════════════════════════════════════════════════════════════════════════
# STEP gait tuning — one leg moves at a time, in order.
# This is the simplest possible walking gait.
#
#   STEP_STRIDE_MM   How far forward the foot travels per step (mm).
#                    The foot sweeps from -STRIDE to +STRIDE, so total
#                    travel = 2 × this value.
#                    Safe range: 15 – 60 mm.
#
#   STEP_LIFT_MM     Peak foot height above ground during swing (mm).
#                    Lower = less joint extension, safer on hardware.
#                    Safe range: 20 – 80 mm.
#
#   STEP_SWING_S     Time each foot spends in the air swinging forward (seconds).
#                    Slower = smoother arc; faster = snappier step.
#                    Safe range: 0.3 – 1.5 s.
#
#   STEP_RETURN_S    Time the foot spends sliding back to neutral after planting (seconds).
#                    This is the ground-contact phase that propels the body forward.
#                    Longer = more push per step.
#                    Safe range: 0.2 – 1.5 s.
#
#   STEP_LEG_ORDER   Which leg moves in which order.
#                    Indices: 0=FR, 1=FL, 2=RR, 3=RL
#                    Default [0, 2, 1, 3] = FR → RR → FL → RL
#                    Try [0, 3, 1, 2] for a diagonal pattern.
# ═════════════════════════════════════════════════════════════════════════════
STEP_STRIDE_MM  = 90.0       # mm — how far forward each foot steps
STEP_LIFT_MM    = 50.0       # mm — peak foot clearance during swing
STEP_SWING_S    = 0.6        # s  — time in the air (lift-and-forward arc)
STEP_RETURN_S   = 0.5        # s  — time sliding back to neutral (propulsion phase)
STEP_LEG_ORDER  = [0, 3, 1, 2]  # FR → RL → FL → RR

# SHUFFLE gait tuning.
# Each leg: quick kick from neutral (x=0) to +SHUFFLE_STRIDE_MM forward,
# then slow slide back to neutral along the ground (propulsion stroke).
SHUFFLE_STRIDE_MM   = 60.0   # mm — how far forward each foot kicks from neutral
SHUFFLE_LIFT_MM     = 150.0   # mm — peak foot clearance during the forward kick
SHUFFLE_SWING_S     = 0.50   # s  — time in the air (fast forward kick)
SHUFFLE_RETURN_S    = 1.20   # s  — time on the ground sliding back to neutral
SHUFFLE_LEVEL_GAIN  = 0.30   # scale IMU leveling influence during shuffle
SHUFFLE_LEVEL_DZ_MAX = 12.0  # mm — per-leg max leveling offset in shuffle

# ═════════════════════════════════════════════════════════════════════════════
# TROT rear-lift bias — raises the rear hips to tilt the body nose-down,
# shifting CoM toward the front feet for better forward traction.
#
#   TROT_REAR_LIFT_MM   Extra stand_z added to rear legs (RR, RL) only.
#                       Positive = rear of body higher = nose-down tilt.
#                       0 = symmetric (default level stance).
#                       Safe range: 0 – 60 mm.  Start at 30 and tune.
# ═════════════════════════════════════════════════════════════════════════════
TROT_REAR_LIFT_MM = 30.0   # mm — rear hip elevation above front during TROT

# ── Swing duration — constant per biology (Maes et al. 2008) ──────────────────
# Trot/gallop: 0.25 s  |  Walk: 0.35 s  |  Crawl: 0.40 s  |  Turtle: 0.80 s
_T_SW = {
    GaitType.TROT:      0.25,
    GaitType.GALLOP:    0.25,
    GaitType.WALK:      0.35,
    GaitType.CRAWL:     0.40,
    GaitType.TURTLE:    1.20,   # slow deliberate swing
    GaitType.SHUFFLE:   0.35,   # short swing — foot barely leaves the ground
    GaitType.DIAG_TROT: DIAG_TROT_SWING_S,
}

# ── Default stance duration for slow / stationary gaits ───────────────────────
_T_ST_SLOW = {
    GaitType.TROT:      0.50,
    GaitType.GALLOP:    0.40,
    GaitType.WALK:      0.50,
    GaitType.CRAWL:     0.60,
    GaitType.TURTLE:    6.00,   # long stance — very slow cadence
    GaitType.SHUFFLE:   0.80,   # long stance relative to swing for stability
    GaitType.DIAG_TROT: DIAG_TROT_STANCE_S,
}

# ── Per-gait top-speed fraction of MAX_SPEED_MS ───────────────────────────────
# Turtle is capped well below the other gaits so full joystick stays slow.
_SPEED_SCALE = {
    GaitType.TROT:      1.00,
    GaitType.GALLOP:    1.00,
    GaitType.WALK:      0.60,
    GaitType.CRAWL:     0.35,
    GaitType.TURTLE:    0.06,   # max ~0.09 m/s at full stick
    GaitType.SHUFFLE:   0.20,   # max ~0.30 m/s at full stick
    GaitType.DIAG_TROT: DIAG_TROT_SPEED_SCALE,
}

# ── Phase offsets ΔS_i relative to FR (leg 0) — Eq. (3.7) / (3.8) ────────────
# Order: [FR, FL, RR, RL]
_PHASE_OFFSETS = {
    GaitType.TROT:      [0.00, 0.50, 0.50, 0.00],  # diagonal pairs
    GaitType.GALLOP:    [0.00, 0.20, 0.55, 0.75],  # transverse gallop
    GaitType.WALK:      [0.00, 0.50, 0.25, 0.75],  # diagonal walk (duty ≈ 0.75)
    GaitType.CRAWL:     [0.00, 0.50, 0.75, 0.25],  # lateral-sequence (duty ≈ 0.80)
    GaitType.TURTLE:    [0.00, 0.50, 0.75, 0.25],  # same pattern as crawl, much slower
    GaitType.SHUFFLE:   [0.00, 0.50, 0.75, 0.25],  # lateral-sequence, short strides
    GaitType.DIAG_TROT: [0.00, 0.50, 0.50, 0.00],  # FR+RL in sync, FL+RR in sync
}

# ── Half-stroke length L_span (mm) ────────────────────────────────────────────
# The foot travels from +L_span to -L_span during stance.
# L_span is also the x-axis scaling reference for the Bezier curve.
_L_SPAN_DEFAULT = STEP_LENGTH / 2   # mm (STEP_LENGTH is full stroke)

# Per-gait overrides — gaits not listed here use _L_SPAN_DEFAULT.
_L_SPAN_GAIT = {
    GaitType.SHUFFLE:   38.0,
    GaitType.DIAG_TROT: DIAG_TROT_STRIDE_MM,
}

# Per-gait foot lift height overrides (mm).
_STEP_HEIGHT_GAIT = {
    GaitType.SHUFFLE:   60.0,
    GaitType.DIAG_TROT: DIAG_TROT_LIFT_MM,
}

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


def _smootherstep(t: float) -> float:
    """C2-smooth easing in [0,1] with zero vel/accel at both ends."""
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


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


def _nominal_foot_from_start_angles(
    upper_start_deg: float,
    lower_start_deg: float,
) -> tuple[float, float]:
    """Return nominal (x, z) from start-link angles.

    Angle convention:
      upper_start_deg: +down from forward horizontal (+x)
      lower_start_deg: +down from rearward horizontal (-x)
    """
    up = math.radians(upper_start_deg)
    lo = math.radians(lower_start_deg)

    x = UPPER_LENGTH * math.cos(up) - LOWER_LENGTH * math.cos(lo)
    z = UPPER_LENGTH * math.sin(up) + LOWER_LENGTH * math.sin(lo)
    z = max(1.0, z)  # keep IK denominator sane if params are misconfigured
    return x, z


def default_foot_positions(stand_z: float = STAND_HEIGHT) -> list[tuple[float, float, float]]:
    """Neutral standing foot positions in the hip frame (mm).
    foot_y = 0 keeps the hip (abduction) motor flat against the body.
    Lateral offset is added only when strafing (vy != 0).
    """
    return [
        (0.0, 0.0, stand_z),  # FR
        (0.0, 0.0, stand_z),  # FL
        (0.0, 0.0, stand_z),  # RR
        (0.0, 0.0, stand_z),  # RL
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

    def __init__(self,
                 upper_start_deg: float = GAIT_START_UPPER_ANGLE_DEG,
                 lower_start_deg: float = GAIT_START_LOWER_ANGLE_DEG):
        self.gait_type = GaitType.STAND
        self._upper_start_deg = float(upper_start_deg)
        self._lower_start_deg = float(lower_start_deg)
        self._nominal_foot_x, self._stand_z = _nominal_foot_from_start_angles(
            self._upper_start_deg,
            self._lower_start_deg,
        )
        self.foot_pos = default_foot_positions(self._stand_z)

        # Impedance gains (read-only by external nodes)
        self.kp_radial  = KP_RADIAL
        self.kd_radial  = KD_RADIAL
        self.kp_angular = KP_ANGULAR
        self.kd_angular = KD_ANGULAR

        # Internal state — phase-based gaits
        self._last_time    = time.time()
        self._phase_clock  = 0.0   # master clock elapsed since last reset (s)
        self._t_st         = _T_ST_SLOW[GaitType.TROT]
        self._l_span       = _L_SPAN_DEFAULT
        self._step_height  = STEP_HEIGHT
        self._height_ratio = self._stand_z / 500.0
        self._delta_front  = 36.0 * self._height_ratio
        self._delta_rear   = 10.0 * self._height_ratio

        # Internal state — STEP gait (sequential one-leg-at-a-time)
        self._seq_idx      = 0      # current index into STEP_LEG_ORDER
        self._seq_phase    = 0.0    # 0→1 within the current sub-phase
        self._seq_in_swing = True   # True = swinging forward; False = returning to neutral
        # Per-leg lift-up phase envelope for SHUFFLE (0..1); used by gait_node
        # for optional joint-space lift trim.
        self._shuffle_lift_alpha = [0.0, 0.0, 0.0, 0.0]

    @property
    def nominal_stand_z(self) -> float:
        return self._stand_z

    @property
    def nominal_stand_x(self) -> float:
        return self._nominal_foot_x

    @property
    def shuffle_lift_alpha(self) -> list[float]:
        return list(self._shuffle_lift_alpha)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_gait(self, gait_type: GaitType):
        """Switch to a new gait. Resets the phase clock."""
        self.gait_type    = gait_type
        self._phase_clock = 0.0
        self._l_span      = _L_SPAN_GAIT.get(gait_type, _L_SPAN_DEFAULT)
        self._step_height = _STEP_HEIGHT_GAIT.get(gait_type, STEP_HEIGHT)
        if gait_type == GaitType.STAND:
            self.foot_pos = default_foot_positions(self._stand_z)
        if gait_type == GaitType.STEP:
            self._seq_idx      = 0
            self._seq_phase    = 0.0
            self._seq_in_swing = True

    def update(self, vx: float, vy: float, yaw: float,
               body_roll: float = 0.0, body_pitch: float = 0.0,
               level_pitch: float = 0.0, level_roll: float = 0.0,
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
            self._shuffle_lift_alpha = [0.0, 0.0, 0.0, 0.0]
            positions = self._stand_pose(body_roll, body_pitch)
            return self._apply_level_correction(positions, level_pitch, level_roll)

        if self.gait_type == GaitType.STEP:
            self._shuffle_lift_alpha = [0.0, 0.0, 0.0, 0.0]
            positions = self._step_update(vx, vy, dt)
            positions = self._apply_body_pose(positions, body_roll, body_pitch)
            return self._apply_level_correction(positions, level_pitch, level_roll)

        if self.gait_type == GaitType.SHUFFLE:
            positions = self._shuffle_update(vx, yaw, dt)
            positions = self._apply_body_pose(positions, body_roll, body_pitch)
            return self._apply_level_correction(
                positions,
                level_pitch * SHUFFLE_LEVEL_GAIN,
                level_roll * SHUFFLE_LEVEL_GAIN,
                dz_limit_mm=SHUFFLE_LEVEL_DZ_MAX,
            )

        if self.gait_type not in _PHASE_OFFSETS:
            self._shuffle_lift_alpha = [0.0, 0.0, 0.0, 0.0]
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
            v_phys_mms = v_mag * MAX_SPEED_MS * _SPEED_SCALE.get(self.gait_type, 1.0) * 1000.0   # mm/s
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

        positions = self._apply_body_pose(positions, body_roll, body_pitch)
        return self._apply_level_correction(positions, level_pitch, level_roll)

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

        # Per-leg stand height: TROT raises rear hips to tilt CoM forward.
        is_rear = not is_front
        if self.gait_type == GaitType.TROT and is_rear:
            leg_stand_z = self._stand_z + TROT_REAR_LIFT_MM
        else:
            leg_stand_z = self._stand_z

        if is_front:
            delta = self._delta_front
        else:
            if self.gait_type == GaitType.GALLOP:
                delta = 25.0 * self._height_ratio
                p0_x = -50.0 if dir_x >= 0.0 else 0.0   # rear legs rearward only on forward motion
                gl   =  1.0 if dir_x >= 0.0 else 0.0    # gallop lean only on forward motion
            else:
                delta = self._delta_rear

        if t_i < t_st:
            s_st = t_i / t_st
            return self._stance_trajectory(leg, s_st, dir_x, dir_y, l_span, delta, p0_x, gl, leg_stand_z)
        else:
            s_sw = (t_i - t_st) / t_sw
            s_sw = max(0.0, min(1.0, s_sw))
            return self._swing_trajectory(leg, s_sw, dir_x, dir_y, l_span, p0_x, leg_stand_z)

    # ── STEP gait — sequential one-leg-at-a-time ─────────────────────────────

    def _step_update(self, vx: float, vy: float, dt: float
                     ) -> list[tuple[float, float, float]]:
        """
        Sequential gait: one leg is active at a time.

        Each step has two sub-phases for the active leg:
          Swing (in air):   foot lifts from neutral (x=0) and arcs forward to
                            +STEP_STRIDE_MM with a half-sine height profile.
          Return (on ground): foot slides back from +STEP_STRIDE_MM to neutral (x=0),
                            pushing the body forward.

        All other legs stay parked at neutral (x=0, z=nominal_stand_z) the whole time.
        When the return phase completes, the next leg in STEP_LEG_ORDER becomes active.
        """
        moving = math.hypot(vx, vy) > 0.05
        dir_x  = (vx / math.hypot(vx, vy)) if math.hypot(vx, vy) > 0.02 else 1.0
        stride  = STEP_STRIDE_MM
        active_leg = STEP_LEG_ORDER[self._seq_idx]

        # Advance phase
        if moving:
            duration = STEP_SWING_S if self._seq_in_swing else STEP_RETURN_S
            self._seq_phase += dt / duration

        # Transition between sub-phases / legs
        if self._seq_phase >= 1.0:
            self._seq_phase = 0.0
            if self._seq_in_swing:
                # Swing done → start return phase for the same leg
                self._seq_in_swing = False
            else:
                # Return done → advance to next leg, start its swing
                self._seq_idx      = (self._seq_idx + 1) % len(STEP_LEG_ORDER)
                self._seq_in_swing = True
                active_leg         = STEP_LEG_ORDER[self._seq_idx]

        s = max(0.0, min(1.0, self._seq_phase))

        # All legs at neutral by default
        positions = [(0.0, 0.0, self._stand_z)] * 4

        if self._seq_in_swing:
            # Arc from neutral forward to +stride, rising then falling
            foot_x = stride * dir_x * s
            foot_z = self._stand_z - STEP_LIFT_MM * math.sin(math.pi * s)
        else:
            # Slide from +stride back to neutral along the ground
            foot_x = stride * dir_x * (1.0 - s)
            foot_z = self._stand_z

        # Replace active leg position (all others remain at neutral)
        positions[active_leg] = (foot_x, 0.0, foot_z)
        return positions

    def _shuffle_update(self, vx: float, yaw: float, dt: float
                        ) -> list[tuple[float, float, float]]:
        """Ground shuffle — self-propelling, one leg at a time.

        Each leg cycles between neutral x (derived from start angles) and
        +SHUFFLE_STRIDE_MM forward from that neutral:

          Swing  (SHUFFLE_SWING_S, fast):
            Foot lifts from neutral and arcs forward to +stride in a
            parabolic arc — the quick forward kick shown in the drawing.

          Stance (SHUFFLE_RETURN_S, slow):
            Foot is planted at +stride and the commanded position slides
            back to neutral. The planted foot moving rearward
            relative to the body propels the dog forward.

        Sequence (FR → RL → FL → RR): phase gaps = 0.25 × T_stride =
        SHUFFLE_SWING_S exactly, so one leg is in swing at all times and
        the other three are in stance.
        Body speed ≈ SHUFFLE_STRIDE_MM / SHUFFLE_RETURN_S = 100 mm/s.
        """
        t_sw     = SHUFFLE_SWING_S
        t_st     = SHUFFLE_RETURN_S
        t_stride = t_sw + t_st
        self._phase_clock = (self._phase_clock + dt) % t_stride

        stride = SHUFFLE_STRIDE_MM
        x0 = self._nominal_foot_x
        self._shuffle_lift_alpha = [0.0, 0.0, 0.0, 0.0]
        # Phase offsets [FR=0.00, FL=0.50, RR=0.75, RL=0.25]
        # Each offset is 0.25 × t_stride = 0.20 s = T_sw apart → sequential.
        phase_offsets = _PHASE_OFFSETS[GaitType.SHUFFLE]

        positions = []
        for leg in range(4):
            ds  = phase_offsets[leg]
            t_i = (self._phase_clock - ds * t_stride) % t_stride

            if t_i < t_sw:
                # Swing: 3-phase lift / kick / plant.
                s_sw = t_i / t_sw
                if s_sw < 0.30:                          # Phase 1 — Lift straight up
                    s      = s_sw / 0.30
                    u = _smootherstep(s)
                    foot_x = x0
                    foot_z = self._stand_z - SHUFFLE_LIFT_MM * u
                elif s_sw < 0.70:                        # Phase 2 — Kick forward at peak height
                    s      = (s_sw - 0.30) / 0.40
                    u = _smootherstep(s)
                    foot_x = x0 + stride * u
                    foot_z = self._stand_z - SHUFFLE_LIFT_MM
                else:                                    # Phase 3 — Plant straight down
                    s      = (s_sw - 0.70) / 0.30
                    u = _smootherstep(s)
                    foot_x = x0 + stride
                    foot_z = self._stand_z - SHUFFLE_LIFT_MM * (1.0 - u)
            else:
                # Stance: slide from +stride back to neutral along the ground.
                s_st   = (t_i - t_sw) / t_st
                # C2 easing removes touchdown/return jerk and end-of-stance snap.
                u = _smootherstep(s_st)
                foot_x = x0 + stride * (1.0 - u)   # +stride → neutral
                foot_z = self._stand_z

            # Keep joint-space lift trim envelope continuous and tied to actual
            # foot height. This avoids trim snapping at swing sub-phase boundaries.
            if SHUFFLE_LIFT_MM > 1e-6:
                self._shuffle_lift_alpha[leg] = max(
                    0.0,
                    min(1.0, (self._stand_z - foot_z) / SHUFFLE_LIFT_MM),
                )
            else:
                self._shuffle_lift_alpha[leg] = 0.0

            positions.append((foot_x, 0.0, foot_z))

        return positions

    # ── Leg trajectory generator ──────────────────────────────────────────────

    def _stance_trajectory(self, leg: int, s_st: float, dir_x: float,
                           dir_y: float, l_span: float, delta: float,
                           p0_x: float, gl: float, stand_z: float
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
            foot_z = stand_z + delta * (
                math.cos(math.pi * stroke / (2.0 * l_span))
                - (gl / 2.0) * math.sin(math.pi * stroke / l_span)
            )
        else:
            foot_z = stand_z

        return foot_x, foot_y, foot_z

    def _swing_trajectory(self, leg: int, s_sw: float, dir_x: float,
                          dir_y: float, l_span: float, p0_x: float,
                          stand_z: float
                          ) -> tuple[float, float, float]:
        """
        Swing phase: 11th-degree Bezier curve. Eq. (3.9)

        Control points are scaled from the MIT reference geometry
        (L_span=200mm, stand=500mm) to our robot's dimensions.

        The Bezier x-axis (bx) represents the along-stride distance; it is
        rotated into the 2D (dir_x, dir_y) direction so the swing liftoff
        and touchdown positions match the start/end of the stance sweep.
        """
        ctrl = _scale_ctrl_pts(l_span, stand_z, self._step_height)
        bx, bz = _bezier(ctrl, s_sw)

        foot_x = bx * dir_x + p0_x
        foot_y = bx * _LEG_SIDE[leg] * dir_y

        return foot_x, foot_y, bz

    # ── IMU body leveling ─────────────────────────────────────────────────────

    def _apply_level_correction(self, positions: list,
                                pitch_deg: float, roll_deg: float,
                                dz_limit_mm: float | None = None
                                ) -> list[tuple[float, float, float]]:
        """Adjust foot-z per leg so the body stays horizontal.

        Uses the body-frame hip position of each leg — not the foot's
        hip-frame position — so the correction is correct at both
        neutral stand (foot_x = 0) and during gait (foot_x != 0).

        Sign convention (body leveling, not ground-following):
          pitch > 0 → nose up  → shorten front legs, extend rear legs
          roll  > 0 → right up → shorten right legs, extend left legs

          dz = -(hip_x[i]·sin(pitch) + hip_y[i]·sin(roll))

        Leg order: [FR, FL, RR, RL]
          hip_x: front = +BODY_LENGTH/2, rear = -BODY_LENGTH/2
          hip_y: right = +BODY_WIDTH/2,  left = -BODY_WIDTH/2
        """
        if abs(pitch_deg) < 0.05 and abs(roll_deg) < 0.05:
            return positions

        p = math.radians(pitch_deg)
        r = math.radians(roll_deg)

        half_len = BODY_LENGTH / 2.0
        half_wid = BODY_WIDTH  / 2.0

        # Body-frame hip positions for each leg [FR, FL, RR, RL]
        hip_x = [ half_len,  half_len, -half_len, -half_len]
        hip_y = [ half_wid, -half_wid,  half_wid, -half_wid]

        out = []
        for i, (x, y, z) in enumerate(positions):
            dz = -(hip_x[i] * math.sin(p) + hip_y[i] * math.sin(r))
            if dz_limit_mm is not None:
                dz = max(-dz_limit_mm, min(dz_limit_mm, dz))
            out.append((x, y, z + dz))
        return out

    # ── Body pose adjustment ──────────────────────────────────────────────────

    def _stand_pose(self, roll: float, pitch: float
                    ) -> list[tuple[float, float, float]]:
        """Static standing pose with optional body tilt compensation."""
        r = math.radians(roll)
        p = math.radians(pitch)
        out = []
        for x, y, z in default_foot_positions(self._stand_z):
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
