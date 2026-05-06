"""
gait_node.py
------------
ROS2 node that runs the gait generator and inverse kinematics.

Subscribed topics:
  /gait_command   (geometry_msgs/Twist)   — velocity command
  /body_pose      (geometry_msgs/Vector3) — roll, pitch, yaw
  /robot_state    (std_msgs/String)       — current robot state
  /gait_type      (std_msgs/String)       — gait type name

Published topics:
  /joint_angles   (std_msgs/Float32MultiArray)
    8 motor position commands in RADIANS. Hip motors removed (8DOF).
    Order: FR_sho, FR_kne,
           FL_sho, FL_kne,
           RR_sho, RR_kne,
           RL_sho, RL_kne
  /joint_gains    (std_msgs/Float32MultiArray) — PD gains for the active gait
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Vector3

from dog.gait_generator import GaitGenerator, GaitType, default_foot_positions
from dog.kinematics import compute_all_legs
from dog.state_manager import RobotState
from dog.robot_config import (
    NEUTRAL_ANGLES,
    GAIT_START_UPPER_ANGLE_DEG, GAIT_START_LOWER_ANGLE_DEG,
    SHUFFLE_LIFT_UPPER_TRIM_DEG, SHUFFLE_LIFT_LOWER_TRIM_DEG,
    JOINT_DIRECTION, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX,
)

_N_MOTORS = 8


CONTROL_RATE_HZ = 100

# Seconds to smoothly blend from the standing position into the first gait cycle.
# Prevents the jolt caused by the phase clock starting at 0 (some legs immediately
# enter swing) and any mismatch between the standing pose and the gait's neutral.
_TRANSITION_DURATION = 0.40

# PD gains per gait.  Softer gains during dynamic gaits reduce impact loads on
# the PLA frame and let the 36:1 motors swing freely without fighting themselves.
# Stiffer gains during stand/turtle give solid position hold.
_GAIT_GAINS = {
    GaitType.STAND:     (45.0, 2.0),
    GaitType.STEP:      (40.0, 2.0),  # sequential one-leg gait — stiff for clean steps
    GaitType.TURTLE:    (40.0, 2.0),
    GaitType.SHUFFLE:   (38.0, 1.8),
    GaitType.CRAWL:     (35.0, 1.5),
    GaitType.WALK:      (30.0, 1.3),
    GaitType.DIAG_TROT: (25.0, 1.3),
    GaitType.TROT:      (22.0, 1.2),
    GaitType.GALLOP:    (18.0, 1.0),
}


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')

        self.declare_parameter('upper_start_angle_deg', GAIT_START_UPPER_ANGLE_DEG)
        self.declare_parameter('lower_start_angle_deg', GAIT_START_LOWER_ANGLE_DEG)
        self.declare_parameter('shuffle_lift_upper_trim_deg', SHUFFLE_LIFT_UPPER_TRIM_DEG)
        self.declare_parameter('shuffle_lift_lower_trim_deg', SHUFFLE_LIFT_LOWER_TRIM_DEG)
        upper_start_deg = float(self.get_parameter('upper_start_angle_deg').value)
        lower_start_deg = float(self.get_parameter('lower_start_angle_deg').value)
        self._shuffle_lift_upper_trim_deg = float(
            self.get_parameter('shuffle_lift_upper_trim_deg').value
        )
        self._shuffle_lift_lower_trim_deg = float(
            self.get_parameter('shuffle_lift_lower_trim_deg').value
        )

        self.gait = GaitGenerator(
            upper_start_deg=upper_start_deg,
            lower_start_deg=lower_start_deg,
        )
        self.vx           = 0.0
        self.vy           = 0.0
        self.yaw          = 0.0
        self.roll         = 0.0
        self.pitch        = 0.0
        self._level_pitch = 0.0
        self._level_roll  = 0.0
        self.robot_state     = RobotState.ESTOP
        self.walk_gait_type  = GaitType.SHUFFLE

        # Joint-space offset so gait trajectories are relative to the operator's
        # physical standing position rather than the hardcoded NEUTRAL_ANGLES.
        self._home_offset = [0.0] * _N_MOTORS
        self._neutral_motor = compute_all_legs(
            default_foot_positions(self.gait.nominal_stand_z)
        )

        # Latest motor-frame feedback from Teensy — used to start the blend
        # from wherever the legs actually are when walking begins.
        self._fb_angles         = [0.0] * _N_MOTORS
        self._transition_active = False
        self._transition_start  = 0.0
        self._transition_from   = [0.0] * _N_MOTORS
        self._last_trim_clip_log_t = 0.0

        self.create_subscription(Twist,            'gait_command',    self._cmd_callback,       10)
        self.create_subscription(Vector3,          'body_pose',       self._pose_callback,      10)
        self.create_subscription(Vector3,          'level_correction',self._level_callback,     10)
        self.create_subscription(String,           'robot_state',     self._state_callback,     10)
        self.create_subscription(String,           'gait_type',       self._gait_type_callback, 10)
        self.create_subscription(Float32MultiArray,'standing_home',   self._home_callback,      10)
        self.create_subscription(Float32MultiArray,'/joint_states',   self._fb_callback,        10)

        self.joint_pub = self.create_publisher(
            Float32MultiArray, 'joint_angles', 10
        )
        self.gains_pub = self.create_publisher(
            Float32MultiArray, 'joint_gains', 10
        )

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)

        self.get_logger().info(f'Gait node running at {CONTROL_RATE_HZ} Hz')
        self.get_logger().info(
            'Start-angle setup: upper=%.1f deg, lower=%.1f deg, stand_x=%.1f mm, stand_z=%.1f mm'
            % (
                upper_start_deg,
                lower_start_deg,
                self.gait.nominal_stand_x,
                self.gait.nominal_stand_z,
            )
        )
        self.get_logger().info(
            'Shuffle lift trim: upper=%+.2f deg, lower=%+.2f deg (+lower=tuck)'
            % (self._shuffle_lift_upper_trim_deg, self._shuffle_lift_lower_trim_deg)
        )
        self._publish_gains(GaitType.STAND)

    # ────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Twist):
        self.vx  = msg.linear.x
        self.vy  = msg.linear.y
        self.yaw = msg.angular.z

    def _pose_callback(self, msg: Vector3):
        self.roll  = msg.x
        self.pitch = msg.y

    def _level_callback(self, msg: Vector3):
        self._level_roll  = msg.x
        self._level_pitch = msg.y

    def _gait_type_callback(self, msg: String):
        try:
            gt = GaitType[msg.data]
        except KeyError:
            self.get_logger().warn(f'Unknown gait type: {msg.data}')
            return

        if self.robot_state == RobotState.AUTONOMOUS:
            # Autonomous locomotion is constrained to SHUFFLE.
            # STAND is still allowed for obstacle-hold and autonomous exit flow.
            if gt not in (GaitType.SHUFFLE, GaitType.STAND):
                self.get_logger().warn(
                    f'Ignoring gait request {gt.name} in AUTONOMOUS; allowed: SHUFFLE/STAND.'
                )
                return

        self.walk_gait_type = gt
        if self.robot_state in (RobotState.WALKING, RobotState.AUTONOMOUS):
            self.gait.set_gait(gt)
            self._publish_gains(gt)
            self.get_logger().info(f'Switched to {gt.name} gait')

    def _state_callback(self, msg: String):
        new_state = msg.data
        if new_state == self.robot_state:
            return
        self.robot_state = new_state

        if new_state == RobotState.STANDING:
            self.gait.set_gait(GaitType.STAND)
            self._publish_gains(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0
        elif new_state == RobotState.WALKING:
            self.gait.set_gait(self.walk_gait_type)
            self._publish_gains(self.walk_gait_type)
            # Capture current leg positions so we can blend smoothly into the
            # gait instead of jumping to phase=0 immediately.
            self._transition_from   = list(self._fb_angles)
            self._transition_start  = time.time()
            self._transition_active = True
        elif new_state == RobotState.AUTONOMOUS:
            # Autonomous locomotion is fixed to shuffle for stability/safety.
            self.walk_gait_type = GaitType.SHUFFLE
            self.gait.set_gait(self.walk_gait_type)
            self._publish_gains(self.walk_gait_type)
            # Capture current leg positions so we can blend smoothly into the
            # gait instead of jumping to phase=0 immediately.
            self._transition_from   = list(self._fb_angles)
            self._transition_start  = time.time()
            self._transition_active = True
        elif new_state in (RobotState.ESTOP, RobotState.IDLE,
                           RobotState.JUMPING, RobotState.BACKFLIP):
            self.gait.set_gait(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0
            # Do NOT publish gains on ESTOP — state_manager is simultaneously
            # zeroing KP/KD and sending can_enable=False.  Publishing high gains
            # here races against that and can leave the motor drives holding
            # position at high gain even after the Teensy is disconnected.
            if new_state != RobotState.ESTOP:
                self._publish_gains(GaitType.STAND)

    def _fb_callback(self, msg: Float32MultiArray):
        """Track latest motor-frame positions from the Teensy for transition blending."""
        if len(msg.data) >= _N_MOTORS:
            self._fb_angles = list(msg.data[:_N_MOTORS])

    def _home_callback(self, msg: Float32MultiArray):
        """Update the motor-frame offset between the neutral pose and the
        operator-set physical standing position.

        msg.data is motor-frame (JOINT_DIRECTION already applied by state_manager).
        self._neutral_motor is also motor-frame, so the subtraction is frame-consistent.
        """
        if len(msg.data) < _N_MOTORS:
            return
        self._home_offset = [
            float(msg.data[i]) - self._neutral_motor[i]
            for i in range(_N_MOTORS)
        ]
        self.get_logger().info(
            'Standing home updated — motor-frame offsets (rad): '
            + '  '.join(f'{o:+.3f}' for o in self._home_offset)
        )

    def _publish_gains(self, gait_type: GaitType):
        kp, kd = _GAIT_GAINS.get(gait_type, (35.0, 1.5))
        msg = Float32MultiArray()
        msg.data = [kp, kd]
        self.gains_pub.publish(msg)

    def _apply_shuffle_lift_trim(self, angles: list[float]) -> list[float]:
        """Apply optional joint-space trim during SHUFFLE lift-up only.

        User params are specified as IK-space lift trims (degrees):
          - upper trim adjusts shoulder/upper-link angle
          - lower trim adjusts knee/lower-link angle
            Positive lower trim means MORE knee tuck during lift-up
            (i.e. more knee bend in IK convention).

        Conversion to motor frame respects left/right JOINT_DIRECTION and the
        belt-coupled knee motor (knee motor tracks shoulder+knee in IK space).
        """
        if self.gait.gait_type != GaitType.SHUFFLE:
            return angles

        upper_trim_rad = math.radians(self._shuffle_lift_upper_trim_deg)
        lower_trim_rad = math.radians(self._shuffle_lift_lower_trim_deg)
        if abs(upper_trim_rad) < 1e-9 and abs(lower_trim_rad) < 1e-9:
            return angles

        out = list(angles)
        lift_alpha = self.gait.shuffle_lift_alpha  # per-leg 0..1
        any_clip = False
        for leg in range(4):
            a = lift_alpha[leg]
            if a <= 1e-6:
                continue

            # Lower IK angle is negative when bent; make positive user trim
            # mean "more tuck" by applying a negative IK delta.
            d_upper = upper_trim_rad * a
            d_lower = -lower_trim_rad * a

            sho_idx = leg * 2
            kne_idx = sho_idx + 1
            dir_sho = JOINT_DIRECTION.get((leg, 1), 1)
            dir_kne = JOINT_DIRECTION.get((leg, 2), 1)

            # Shoulder motor follows shoulder IK directly.
            out[sho_idx] += dir_sho * d_upper
            # Knee motor follows (shoulder + knee) IK due belt coupling.
            out[kne_idx] += dir_kne * (d_upper + d_lower)

            sho_pre = out[sho_idx]
            kne_pre = out[kne_idx]
            out[sho_idx] = max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, out[sho_idx]))
            out[kne_idx] = max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, out[kne_idx]))
            if abs(out[sho_idx] - sho_pre) > 1e-9 or abs(out[kne_idx] - kne_pre) > 1e-9:
                any_clip = True

        if any_clip:
            now = time.time()
            if now - self._last_trim_clip_log_t > 1.0:
                self._last_trim_clip_log_t = now
                self.get_logger().warn(
                    'Shuffle lift trim hit joint limits; reducing trim or '
                    're-centering standing pose may improve effect.'
                )

        return out

    # ────────────────────────────────────────────────────────────────
    def _control_loop(self):
        if self.robot_state in (RobotState.POSITIONING, RobotState.ESTOP,
                                RobotState.IDLE, RobotState.JUMPING,
                                RobotState.BACKFLIP):
            return

        if self.robot_state == RobotState.STANDING:
            no_move = (abs(self.vx) < 0.001 and abs(self.vy) < 0.001
                       and abs(self.yaw) < 0.001)
            no_tilt = abs(self.roll) < 0.5 and abs(self.pitch) < 0.5
            if no_move and no_tilt:
                return

        foot_positions = self.gait.update(
            vx=self.vx, vy=self.vy, yaw=self.yaw,
            body_roll=self.roll, body_pitch=self.pitch,
            level_pitch=self._level_pitch, level_roll=self._level_roll,
        )

        # Hip is static (8DOF): zero lateral offset so IK geometry is correct
        foot_positions = [(x, 0.0, z) for x, _, z in foot_positions]

        try:
            angles = compute_all_legs(foot_positions)
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            angles = list(NEUTRAL_ANGLES)

        # Shift all angles by the operator's physical standing offset so the
        # gait trajectories are centred on the real standing position, not the
        # hardcoded NEUTRAL_ANGLES.
        angles = [a + o for a, o in zip(angles, self._home_offset)]
        angles = self._apply_shuffle_lift_trim(angles)

        # Blend from the pre-walk standing position into the live gait output.
        # This eliminates the jolt at gait start caused by phase_clock=0 and
        # the mismatch between the standing pose and the gait's first target.
        if self._transition_active:
            elapsed = time.time() - self._transition_start
            if elapsed < _TRANSITION_DURATION:
                t = elapsed / _TRANSITION_DURATION
                t = t * t * (3.0 - 2.0 * t)   # smooth-step ease-in
                angles = [f + (a - f) * t
                          for f, a in zip(self._transition_from, angles)]
            else:
                self._transition_active = False

        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
