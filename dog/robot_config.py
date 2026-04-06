"""
robot_config.py
---------------
Physical constants and motor configuration for the Dog quadruped robot.

Hardware:
  - CubeMars AK45-36 brushless actuators (CAN bus, MIT mini cheetah protocol)
  - Teensy 4.1 running micro-ROS (CAN1 front legs, CAN2 rear legs)
  - Jetson Orin Nano running ROS2 Humble
  - mjbots power_dist r4.5b power distribution board

All joint angles are in RADIANS throughout this package.
The IK solver works internally in degrees but outputs are converted to radians.

Motor zeroing convention:
  Each motor is physically set to its mechanical zero using the calibration tool.
  Mechanical zero positions:
    Hip:      leg pointing straight out laterally from the body (abduction = 0)
    Shoulder: upper leg pointing straight down (vertical, no flex)
    Knee:     lower leg inline with upper leg (fully extended, no flex)
  After zeroing, the motor CAN position command 0.0 rad = that mechanical zero.
  NEUTRAL_ANGLES are the computed IK angles for the default standing pose.

Edit this file to match your physical build dimensions.
"""

# ─────────────────────────────────────────────
# LEG SEGMENT LENGTHS  (millimetres)
# Measure from pivot centre to pivot centre.
# ─────────────────────────────────────────────
HIP_LENGTH    = 130.0   # Distance from hip pivot to shoulder pivot (abduction link)
UPPER_LENGTH  = 320.0   # Upper leg (femur) length
LOWER_LENGTH  = 320.0   # Lower leg (tibia) length

# ─────────────────────────────────────────────
# BODY DIMENSIONS  (millimetres)
# ─────────────────────────────────────────────
BODY_LENGTH = 430.0   # Front-to-rear hip separation (along X)
BODY_WIDTH  = 150.0   # Left-to-right hip separation (along Y)

# ─────────────────────────────────────────────
# DEFAULT STANCE
# Mini Cheetah is fully symmetric — all four legs share the same stand height.
# ─────────────────────────────────────────────
STAND_HEIGHT  = 540.0   # Body height above ground (mm) — all four legs
STEP_HEIGHT   =  80.0   # Foot lift height per step (mm)
STEP_LENGTH   = 150.0   # Foot travel distance per step (mm)
STEP_DURATION =   0.3   # Seconds per half-cycle (~3.3 Hz trot cadence)

# ─────────────────────────────────────────────
# BODY POSE LIMITS  (degrees, used inside gait/state nodes)
# ─────────────────────────────────────────────
MAX_BODY_ROLL  = 15.0
MAX_BODY_PITCH = 15.0
MAX_BODY_YAW   = 20.0

# ─────────────────────────────────────────────
# CAN MOTOR CONFIGURATION
# ─────────────────────────────────────────────
# CAN bus → Teensy 4.1 pins:
#   CAN1: TX=22, RX=23  — front legs (FR + FL)
#   CAN3: TX=31, RX=30  — rear  legs (RR + RL)
#
# Each motor needs a unique CAN ID (1–127).
# Set IDs using the CubeMars R-Link USB adapter + CubeMars debugging software.
# ─────────────────────────────────────────────
# CAN_ID_MAP[leg_index][joint_index] = (can_bus, motor_id)
#   leg:   0=FR, 1=FL, 2=RR, 3=RL
#   joint: 1=shoulder, 2=knee  (hip removed — static dummy, 8DOF)
#   can_bus: 1=CAN1 (front), 2=CAN2 (rear)
CAN_ID_MAP = {
    # Front Right — CAN1
    (0, 1): (1, 0x79),  # FR Shoulder (121)
    (0, 2): (1, 0x7A),  # FR Knee     (122)

    # Front Left  — CAN1
    (1, 1): (1, 0x75),  # FL Shoulder (117)
    (1, 2): (1, 0x78),  # FL Knee     (120)

    # Rear Right  — CAN3
    (2, 1): (3, 0x73),  # RR Shoulder (115)
    (2, 2): (3, 0x74),  # RR Knee     (116)

    # Rear Left   — CAN3
    (3, 1): (3, 0x76),  # RL Shoulder (118)
    (3, 2): (3, 0x77),  # RL Knee     (119)
}

# Flat list: index = leg*2 + (joint-1) → motor_id (matches Teensy MOTOR_ID[8])
# Order: FR_sho, FR_kne, FL_sho, FL_kne, RR_sho, RR_kne, RL_sho, RL_kne
MOTOR_IDS = [CAN_ID_MAP[(leg, joint)][1]
             for leg in range(4) for joint in range(1, 3)]

# ─────────────────────────────────────────────
# MIT MINI CHEETAH CAN PROTOCOL LIMITS
# AK45-36 operating ranges
# ─────────────────────────────────────────────
CAN_POS_MIN  = -12.5    # rad
CAN_POS_MAX  =  12.5    # rad
CAN_VEL_MIN  = -45.0    # rad/s
CAN_VEL_MAX  =  45.0    # rad/s
CAN_KP_MIN   =   0.0    # N·m/rad
CAN_KP_MAX   = 500.0    # N·m/rad
CAN_KD_MIN   =   0.0    # N·m·s/rad
CAN_KD_MAX   =   5.0    # N·m·s/rad
CAN_TRQ_MIN  = -18.0    # N·m
CAN_TRQ_MAX  =  18.0    # N·m

# Default impedance gains for position control
CAN_KP_DEFAULT  = 150.0   # N·m/rad — stiff position hold
CAN_KD_DEFAULT  =   2.0   # N·m·s/rad — light damping

# ─────────────────────────────────────────────
# JOINT ANGLE LIMITS  (radians)
# Soft limits applied before sending CAN commands.
# Tune to match your physical joint stops.
# ─────────────────────────────────────────────
JOINT_ANGLE_MIN = -2.618   # -150°
JOINT_ANGLE_MAX =  2.618   #  150°

# ─────────────────────────────────────────────
# JOINT DIRECTION FLAGS
# Set to -1 if the motor rotates opposite to the IK convention.
# IK convention: hip=outward-positive, shoulder=forward-down-positive,
#                knee=bend-positive (negative IK angle = bent)
#
# How to verify each axis (motors zeroed, low Kp):
#   Hip:      command +0.2 rad → leg should swing OUTWARD. If it goes inward, flip to -1.
#   Shoulder: command +0.2 rad → leg should swing FORWARD-DOWN. If backward, flip to -1.
#   Knee:     command +0.2 rad → leg should extend (straighten). If it bends, flip to -1.
#             Note: knee motor is belt-driven from the body — verify after shoulder is confirmed.
# ─────────────────────────────────────────────
JOINT_DIRECTION = {
    (0, 1):  1, (0, 2):  1,   # FR shoulder, knee
    (1, 1): -1, (1, 2): -1,   # FL shoulder, knee
    (2, 1):  1, (2, 2):  1,   # RR shoulder, knee
    (3, 1):  1, (3, 2):  1,   # RL shoulder, knee
}

# ─────────────────────────────────────────────
# JOINT OFFSETS  (degrees)
# Fine-tune after zeroing if motors are not exactly at mechanical zero.
# Formula: motor_cmd_rad = direction * ik_angle_rad + radians(offset_deg)
# ─────────────────────────────────────────────
JOINT_OFFSETS = {
    (0, 1): 0.0, (0, 2): 0.0,   # FR shoulder, knee
    (1, 1): 0.0, (1, 2): 0.0,   # FL shoulder, knee
    (2, 1): 0.0, (2, 2): 0.0,   # RR shoulder, knee
    (3, 1): 0.0, (3, 2): 0.0,   # RL shoulder, knee
}

# ─────────────────────────────────────────────
# NEUTRAL STANDING ANGLES  (radians, motor frame)
# Computed from IK at (foot_x=0, foot_y=0, foot_z=STAND_HEIGHT).
# Hip motors removed (8DOF) — only shoulder and knee.
#
# Belt-drive coupling: knee motor = shoulder_ik + knee_ik (body-frame knee).
#   shoulder_ik = +35.02°, knee_ik = -70.04°, coupled = -35.02°
#   (UPPER=320 mm, LOWER=320 mm, HIP=130 mm, STAND_HEIGHT=540 mm)
# ─────────────────────────────────────────────
NEUTRAL_ANGLES = [
     0.6112, -0.6112,   # FR: shoulder, knee
     0.6112, -0.6112,   # FL
     0.6112, -0.6112,   # RR
     0.6112, -0.6112,   # RL
]

SIT_ANGLES = [
     1.1346, -1.1346,   # FR: knee bent (forward-tuck sit)
     1.1346, -1.1346,   # FL
     1.1346, -1.1346,   # RR
     1.1346, -1.1346,   # RL
]

# Deep sit: shoulder tilts backward, knee bends moderately.
# Body rests low; tune shoulder/knee magnitudes to match your physical geometry.
#   shoulder < 0  → upper leg tilts rearward
#   knee_coupled  → shoulder_ik + knee_ik; physical bend ≈ -0.4 rad
DEEP_SIT_ANGLES = [
    -0.6, -1.0,   # FR: shoulder back 34°, knee moderately bent
    -0.6, -1.0,   # FL
    -0.6, -1.0,   # RR
    -0.6, -1.0,   # RL
]

# ─────────────────────────────────────────────
# SELF-RIGHTING POSES  (motor frame, same convention as NEUTRAL_ANGLES)
#
# RIGHTING_TUCK_ANGLES — all legs folded tight against the body.
#   Used at the start and end of the righting sequence to reduce
#   rotational inertia and avoid ground obstruction.
#
# Righting push poses are built dynamically in state_manager based on
# the IMU roll at the moment righting is triggered:
#   push leg  : hip=+0.60 (full outward), shoulder/knee near-extended
#   tuck leg  : hip=−0.30 (inward),       shoulder/knee tightly folded
# ─────────────────────────────────────────────
RIGHTING_TUCK_ANGLES = [
     1.30, -1.1346,   # FR
     1.30, -1.1346,   # FL
     1.30, -1.1346,   # RR
     1.30, -1.1346,   # RL
]

# ─────────────────────────────────────────────
# BNO085 9-DOF IMU  (connected directly to Jetson I2C)
# ─────────────────────────────────────────────
BNO085_ADDRESS   = 0x4A  # default; 0x4B if PS1 pin is pulled high
IMU_PUBLISH_RATE = 50    # Hz

# ─────────────────────────────────────────────
# u-blox SAM-M10Q GPS  (connected to Jetson UART)
# ─────────────────────────────────────────────
GPS_PORT      = '/dev/ttyUSB0'  # adjust to your port (ttyUSB0, ttyTHS0, ttyACM0…)
GPS_BAUD_RATE = 9600            # SAM-M10Q factory default; raise after reconfiguring

# ─────────────────────────────────────────────
# CONTROLLER AXES  (identical for Xbox and PS4)
# ─────────────────────────────────────────────
AXIS_LEFT_X  = 0   # Strafe left/right
AXIS_LEFT_Y  = 1   # Forward / backward
# PS4 (ds4drv/hid-sony): axes 2=RX, 3=RY, 4=L2, 5=R2
# Xbox / PS4 (xpadneo):  axes 2=L2, 3=RX, 4=RY, 5=R2
# Set these to match your driver's axis layout (verify with: ros2 topic echo /joy)
AXIS_RIGHT_X = 3   # Right stick X  (Xbox: 3, PS4 ds4drv: 2)
AXIS_RIGHT_Y = 4   # Right stick Y  (Xbox: 4, PS4 ds4drv: 3)
AXIS_LT      = 2   # L2/LT analog trigger  (Xbox: 2, PS4 ds4drv: 4)
AXIS_RT      = 5   # R2/RT analog trigger

# ─────────────────────────────────────────────
# XBOX CONTROLLER BUTTONS
# ─────────────────────────────────────────────
BTN_A        = 0   # Stand / sit toggle
BTN_B        = 1   # Reserved
BTN_X        = 2   # Change gait
BTN_Y        = 3   # Reserved
BTN_LB       = 4   # Deadman enable
BTN_RB       = 5   # Turbo speed
BTN_BACK     = 6   # E-stop        (View button)
BTN_START    = 7   # Reset E-stop  (Menu button)

# ─────────────────────────────────────────────
# PS4 DUALSHOCK 4 BUTTONS
# Axes 0-5 are identical to Xbox above.
# Linux hid-sony driver button order (USB or ds4drv Bluetooth):
#   0: Square(□)  1: Cross(×)  2: Circle(○)  3: Triangle(△)
#   4: L1         5: R1        6: L2(digital) 7: R2(digital)
#   8: Share      9: Options   10: L3         11: R3  12: PS  13: Touchpad
# ─────────────────────────────────────────────
PS4_BTN_SQUARE  = 0   # Stand / sit toggle   (equivalent to Xbox A)
PS4_BTN_CROSS   = 1   # Reserved             (equivalent to Xbox B)
PS4_BTN_CIRCLE  = 2   # Change gait          (equivalent to Xbox X)
PS4_BTN_TRI     = 3   # Reserved             (equivalent to Xbox Y)
PS4_BTN_L1      = 4   # Deadman enable       (same index as Xbox LB)
PS4_BTN_R1      = 5   # Turbo speed          (same index as Xbox RB)
PS4_BTN_BACK    = 8   # E-stop               (Share button)
PS4_BTN_START   = 9   # Reset E-stop         (Options button)

DEADMAN_BUTTON    = BTN_LB
JOYSTICK_SCALE    = 0.7
TURN_SCALE        = 1.5
TURBO_MULTIPLIER  = 2.0
JOYSTICK_DEADZONE = 0.1

# ─────────────────────────────────────────────
# JUMP FORWARD KEYFRAME ANGLES  (motor frame)
#
# SASSA body is rear-heavy. Two fixes applied:
#
#   1. Asymmetric crouch — front legs deeper than rear so the body tilts
#      nose-down, loading the front feet and preventing the rear-heavy
#      body from rearing up before launch.
#
#   2. Straight-leg forward launch — when motor_shoulder = motor_knee,
#      geo_kne = motor_kne − motor_sho = 0.00 (fully straight leg).
#      A straight leg angled at S rad from vertical launches the body at
#      exactly S rad from vertical (= 90°−S from horizontal).
#
#      At S = 0.80 rad (≈ 46°):
#        foot_x = 640 · sin(0.80) ≈ 459 mm forward of hip
#        foot_z = 640 · cos(0.80) ≈ 446 mm below hip
#        launch angle ≈ 44° from horizontal  ← near-optimal for range
#
#      Previous design (shoulder=0.35, knee=0.00) gave only ~10° from
#      vertical = 80° from horizontal = nearly straight up, minimal range.
#
# geometric_knee = motor_knee − motor_shoulder  (belt-drive decoupling)
# Crouch front: geo_kne = −1.15 − 1.15 = −2.30 rad
# Crouch rear:  geo_kne = −1.00 − 1.00 = −2.00 rad
# Launch:       geo_kne =  0.80 − 0.80 =  0.00 rad  (fully straight, 46° forward)
#
# To increase distance further: deepen the crouch (larger magnitude = more
# stored energy = higher launch velocity).  The launch angle is already at
# the theoretical optimum (~44°), so crouch depth is the only remaining lever.
# ─────────────────────────────────────────────
JUMP_CROUCH_ANGLES = [
     1.30, -1.30,   # FR — deeper (geo=−2.60): more stored energy + nose-down
     1.30, -1.30,   # FL
     1.00, -1.00,   # RR — asymmetry (0.30 rad) keeps CoM forward
     1.00, -1.00,   # RL
]
JUMP_LAUNCH_ANGLES = [
     0.80,  0.80,   # FR — straight leg (geo=0.00) at 46° → foot 459 mm forward
     0.80,  0.80,   # FL   near-optimal 44° launch angle for maximum range
     0.80,  0.80,   # RR
     0.80,  0.80,   # RL
]
JUMP_TUCK_ANGLES = [
     1.25, -1.25,   # FR — tight tuck while airborne (geo_kne = −2.50 rad)
     1.25, -1.25,   # FL
     1.25, -1.25,   # RR
     1.25, -1.25,   # RL
]
JUMP_LAND_ANGLES = [
     1.20, -1.20,   # FR — deeper absorption (geo=−2.40)
     1.20, -1.20,   # FL
     0.50, -0.50,   # RR — very extended (geo=−1.00): resists nose-down pitch on landing
     0.50, -0.50,   # RL
]

# ─────────────────────────────────────────────
# BACKFLIP KEYFRAME ANGLES  (motor frame)
#
# Two-phase launch — avoids the "T-rex balance" failure mode:
#
#   Symmetric launch (all legs S=K=0.80), held for 0.40 s:
#     This is the MIT Mini Cheetah approach.  All four feet push off together
#     at 46° forward.  Because the SASSA body is rear-heavy, the front feet
#     carry less load and leave the ground first — the body then rotates
#     nose-up around the rear feet that are still in contact.  Holding the
#     command for 0.40 s gives ~20 ticks at 50 Hz for the legs to fully
#     extend and transfer maximum nose-up angular momentum before the rear
#     feet also leave.
#
#     Asymmetric (front push + rear tuck) failed because the rear legs
#     barely moved from the crouch position (1.30 → 1.25, only 0.05 rad),
#     so the rear feet stayed on the ground and counteracted the nose-up
#     torque from the front legs — producing only a partial rear-up, not a
#     full flip.
#
# Launch: geo_kne = 0.80 − 0.80 = 0.00 (straight leg, 46° forward, all legs)
# ─────────────────────────────────────────────
BACKFLIP_CROUCH_ANGLES = [
     1.40, -1.40,   # FR — deep crouch (geo_kne = −2.80 rad)
     1.40, -1.40,   # FL
     1.40, -1.40,   # RR
     1.40, -1.40,   # RL
]
# Phase 1 of launch — front lifts, rear stays planted (pivot):
#   Front legs extend to 46° forward, lifting the front feet off the ground.
#   Rear stays crouched so it remains on the ground as a fixed pivot.
#   The body rotates nose-up around the rear feet until ~90°.
BACKFLIP_FRONT_LIFT_ANGLES = [
     0.80,  0.80,   # FR — straight leg 46° fwd: front feet leave ground
     0.80,  0.80,   # FL   body pivots nose-up around grounded rear feet
     1.40, -1.40,   # RR — stay crouched: rear feet remain the pivot point
     1.40, -1.40,   # RL
]
# Phase 2 of launch — rear explodes, front immediately tucks:
BACKFLIP_LAUNCH_ANGLES = [
     1.50, -1.50,   # FR — tuck immediately: front is airborne, reduce I now
     1.50, -1.50,   # FL
     0.80,  0.80,   # RR — explosive rear push: launches body fully airborne
     0.80,  0.80,   # RL
]
BACKFLIP_TUCK_ANGLES = [
     1.50, -1.50,   # FR — maximum tuck: I drops ~8× vs extended → 3× faster spin
     1.50, -1.50,   # FL
     1.50, -1.50,   # RR
     1.50, -1.50,   # RL
]
BACKFLIP_LAND_ANGLES = [
     0.70, -0.70,   # FR — extend to catch landing
     0.70, -0.70,   # FL
     0.70, -0.70,   # RR
     0.70, -0.70,   # RL
]
