"""
robot_config.py
---------------
All physical constants and servo configuration for the Spot Micro robot.
Edit these values to match your specific build.
"""

import math

# ─────────────────────────────────────────────
# LEG SEGMENT LENGTHS  (millimetres)
# ─────────────────────────────────────────────
HIP_LENGTH    = 50.0   # Distance from hip pivot to upper-leg pivot
UPPER_LENGTH  = 100.0  # Upper leg (femur) length
LOWER_LENGTH  = 100.0  # Lower leg (tibia) length

# ─────────────────────────────────────────────
# BODY DIMENSIONS  (millimetres)
# ─────────────────────────────────────────────
BODY_LENGTH = 200.0   # Front-to-rear hip separation
BODY_WIDTH  = 80.0    # Left-to-right hip separation

# ─────────────────────────────────────────────
# DEFAULT STANCE
# ─────────────────────────────────────────────
STAND_HEIGHT     = 185.0   # Height of body above ground (mm) — front legs
REAR_STAND_HEIGHT = 185.0  # Rear legs reach further to level the body (tune if still tilted)
STEP_HEIGHT      = 25.0    # How high each foot lifts per step (mm)
STEP_LENGTH      = 55.0    # How far each foot travels per step (mm)
STEP_DURATION    = 0.5     # Seconds per step cycle

# ─────────────────────────────────────────────
# BODY POSE LIMITS  (degrees)
# ─────────────────────────────────────────────
MAX_BODY_ROLL    = 15.0
MAX_BODY_PITCH   = 15.0
MAX_BODY_YAW     = 20.0

# Forward lean applied when walking to keep CoM over the support polygon.
# Positive vx → negative pitch bias (nose down). Tune if robot still tips.
WALK_FORWARD_LEAN = 0.0   # degrees of forward pitch at full forward speed

# ─────────────────────────────────────────────
# PCA9685 CONFIGURATION
# ─────────────────────────────────────────────
PCA9685_ADDRESS_0 = 0x40   # Left  side (FL + RL legs)
PCA9685_ADDRESS_1 = 0x41   # Right side (FR + RR legs)
PCA9685_FREQUENCY = 50     # Hz  — standard for MG996R servos
I2C_BUS           = 1

# ─────────────────────────────────────────────
# MG996R SERVO PULSE WIDTH  (microseconds)
# ─────────────────────────────────────────────
SERVO_MIN_PULSE = 500    # 0°
SERVO_MAX_PULSE = 2500   # 180°
SERVO_ANGLE_MIN = 0.0
SERVO_ANGLE_MAX = 180.0

# ─────────────────────────────────────────────
# SERVO CHANNEL MAP
# key  = (leg_index, joint_index)
# leg  : 0=FR, 1=FL, 2=RR, 3=RL
# joint: 0=hip, 1=shoulder, 2=knee
# value = (pca_board_index, channel)
#
# Physical wiring:
#   Board 0 (0x40) = LEFT  side  │  Board 1 (0x41) = RIGHT side
#   Ch  0 = front knee            │  Ch  0 = front knee
#   Ch  1 = front shoulder        │  Ch  1 = front shoulder
#   Ch  2 = front hip             │  Ch  2 = front hip
#   Ch 13 = rear hip              │  Ch 13 = rear hip
#   Ch 14 = rear shoulder         │  Ch 14 = rear shoulder
#   Ch 15 = rear knee             │  Ch 15 = rear knee
# ─────────────────────────────────────────────
SERVO_CHANNEL_MAP = {
    # Front Right — Board 1 (0x41), channels 0-2
    (0, 0): (1,  2),   # FR Hip
    (0, 1): (1,  1),   # FR Shoulder
    (0, 2): (1,  0),   # FR Knee

    # Front Left  — Board 0 (0x40), channels 0-2
    (1, 0): (0,  2),   # FL Hip
    (1, 1): (0,  1),   # FL Shoulder
    (1, 2): (0,  0),   # FL Knee

    # Rear Right  — Board 1 (0x41), channels 13-15
    (2, 0): (1, 13),   # RR Hip
    (2, 1): (1, 14),   # RR Shoulder
    (2, 2): (1, 15),   # RR Knee

    # Rear Left   — Board 0 (0x40), channels 13-15
    (3, 0): (0, 13),   # RL Hip
    (3, 1): (0, 14),   # RL Shoulder
    (3, 2): (0, 15),   # RL Knee
}

# ─────────────────────────────────────────────
# SERVO CALIBRATION OFFSETS  (degrees)
# Add/subtract from computed angle before sending to servo.
# Tune these so that 0° IK output = mechanically neutral.
# ─────────────────────────────────────────────
SERVO_OFFSETS = {
    # Computed from NEUTRAL_ANGLES so that IK at the default foot position
    # (STAND_HEIGHT below hip, BODY_WIDTH/2 outward) maps to the calibrated
    # neutral servo angles.  Formula: neutral_servo - 90 - direction*ik_neutral
    # IK neutral at z=185mm: hip=12.2°, shoulder=24.2°, knee=-48.2°
    (0, 0): -27.2, (0, 1): -30.8, (0, 2): -60.2,   # FR
    (1, 0): -27.2, (1, 1):  20.8, (1, 2):  55.2,   # FL
    (2, 0): -27.2, (2, 1): -30.8, (2, 2): -63.2,   # RR
    (3, 0): -27.2, (3, 1):  20.8, (3, 2):  55.2,   # RL
}

# ─────────────────────────────────────────────
# SERVO DIRECTION FLAGS
# Some servos are mirrored. Set to -1 to invert.
# ─────────────────────────────────────────────
SERVO_DIRECTION = {
    (0, 0): 1,  (0, 1): -1, (0, 2): -1,    # FR: shoulder inverted, knee same as original
    (1, 0): 1, (1, 1): 1, (1, 2): 1,   # FL: all mirrored
    (2, 0): 1,  (2, 1): -1, (2, 2): -1,    # RR: shoulder inverted, knee same as original
    (3, 0): 1, (3, 1): 1, (3, 2): 1,   # RL: all mirrored
}

# ─────────────────────────────────────────────
# NEUTRAL STANDING ANGLES  (degrees, servo frame)
# Applied when the robot first stands up.
# ─────────────────────────────────────────────
NEUTRAL_ANGLES = [
    75.0, 35.0, 78.0,    # FR: hip, shoulder, knee
    75.0, 135.0, 97.0,   # FL: hip, shoulder, knee
    75.0, 35.0, 75.0,    # RR: hip, shoulder, knee  (raised 10 mm via REAR_STAND_HEIGHT)
    75.0, 135.0, 97.0,  # RL: hip, shoulder, knee  (raised 10 mm via REAR_STAND_HEIGHT)
]

SIT_ANGLES = [
    # Hip and shoulder match NEUTRAL exactly — only the knee bends ~24° more.
    # Right-side knees (FR, RR): direction=-1, so more bend = higher servo.
    # Left-side knees (FL, RL):  direction=+1, so more bend = lower servo.
    75.0,  35.0, 102.0,   # FR: knee +24° from neutral 78°
    75.0, 135.0,  73.0,   # FL: knee -24° from neutral 97°
    75.0,  35.0,  99.0,   # RR: knee +24° from neutral 75°
    75.0, 135.0,  73.0,   # RL: knee -24° from neutral 97°
]

# ─────────────────────────────────────────────
# MPU-6050
# ─────────────────────────────────────────────
MPU6050_ADDRESS  = 0x68
IMU_PUBLISH_RATE = 50   # Hz

# ─────────────────────────────────────────────
# XBOX CONTROLLER AXES / BUTTONS
# ─────────────────────────────────────────────
AXIS_LEFT_X  = 0   # Strafe left/right
AXIS_LEFT_Y  = 1   # Forward / backward
AXIS_RIGHT_X = 3   # Yaw (turn)
AXIS_RIGHT_Y = 4   # Body pitch
AXIS_LT      = 2   # Body roll left
AXIS_RT      = 5   # Body roll right

BTN_A        = 0   # Stand / sit toggle
BTN_B        = 1   # Reserved
BTN_X        = 2   # Change gait
BTN_Y        = 3   # Reserved
BTN_LB       = 4   # Deadman enable
BTN_RB       = 5   # Turbo speed
BTN_BACK     = 6   # E-stop
BTN_START    = 7   # Reset pose

DEADMAN_BUTTON  = BTN_LB
JOYSTICK_SCALE  = 0.7    # Max linear velocity  (m/s equivalent)
TURN_SCALE      = 1.5    # Max yaw rate
TURBO_MULTIPLIER = 2.0
JOYSTICK_DEADZONE = 0.1
