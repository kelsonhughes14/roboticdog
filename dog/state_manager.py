"""
state_manager.py
----------------
Manages the high-level robot state machine.

States
------
  IDLE        Robot is powered but not initialised.
  SITTING     All legs folded, safe resting position.
  STANDING    Robot is up and holding still.
  WALKING     Gait controller is active.
  AUTONOMOUS  Nav2 autonomous navigation is active (cmd_vel from autonomous_bridge_node).
  ESTOP       Emergency stop — all motors enter safe (torque-free) state.
  RIGHTING    Self-righting recovery sequence (triggered from ESTOP via B button).

Subscribed topics:
  /joy                        (sensor_msgs/Joy)
  /imu/euler                  (geometry_msgs/Vector3)
  /estop                      (std_msgs/Bool)  — true=trigger ESTOP, false=clear ESTOP
Published topics:
  /robot_state                (std_msgs/String)
  /estop_state                (std_msgs/Bool)   — true if robot is in ESTOP state
  /gait_command               (geometry_msgs/Twist)
  /body_pose                  (geometry_msgs/Vector3)
  /joint_angles               (std_msgs/Float32MultiArray)
  /can_enable                 (std_msgs/Bool)   — true=enter motor mode, false=exit
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

from dog.robot_config import (
    BTN_A, BTN_BACK, BTN_START, BTN_LB, BTN_RB, BTN_X, BTN_Y,
    PS4_BTN_SQUARE, PS4_BTN_CIRCLE, PS4_BTN_BACK, PS4_BTN_START,
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y,
    AXIS_LT, AXIS_RT,
    JOYSTICK_SCALE, TURN_SCALE, TURBO_MULTIPLIER, JOYSTICK_DEADZONE,
    MAX_BODY_ROLL, MAX_BODY_PITCH, JOINT_DIRECTION,
    NEUTRAL_ANGLES, SIT_ANGLES, DEEP_SIT_ANGLES, RIGHTING_TUCK_ANGLES,
    JUMP_CROUCH_ANGLES, JUMP_LAUNCH_ANGLES, JUMP_TUCK_ANGLES, JUMP_LAND_ANGLES,
    BACKFLIP_CROUCH_ANGLES, BACKFLIP_FRONT_LIFT_ANGLES, BACKFLIP_LAUNCH_ANGLES,
    BACKFLIP_TUCK_ANGLES, BACKFLIP_LAND_ANGLES,
)
from dog.gait_generator import GaitType

_WALK_GAIT_CYCLE = [GaitType.TROT, GaitType.GALLOP, GaitType.WALK, GaitType.CRAWL, GaitType.TURTLE]

_SIT_RAMP_DURATION = 2.0   # seconds — time to ease into the sit position


class RobotState:
    IDLE         = 'IDLE'
    POSITIONING  = 'POSITIONING'   # compliant leg-by-leg capture on platform
    DEEP_SITTING = 'DEEP_SITTING'
    SITTING      = 'SITTING'
    STANDING     = 'STANDING'
    WALKING      = 'WALKING'
    AUTONOMOUS   = 'AUTONOMOUS'
    ESTOP        = 'ESTOP'
    RIGHTING     = 'RIGHTING'
    JUMPING      = 'JUMPING'
    BACKFLIP     = 'BACKFLIP'


class StateManagerNode(Node):

    def __init__(self):
        super().__init__('state_manager')

        self.declare_parameter('controller_type', 'xbox')
        ctrl_type = self.get_parameter('controller_type').get_parameter_value().string_value
        if ctrl_type == 'ps4':
            self._btn_a     = PS4_BTN_SQUARE
            self._btn_x     = PS4_BTN_CIRCLE
            self._btn_back  = PS4_BTN_BACK
            self._btn_start = PS4_BTN_START
        else:
            self._btn_a     = BTN_A
            self._btn_x     = BTN_X
            self._btn_back  = BTN_BACK
            self._btn_start = BTN_START
        self.get_logger().info(f'Controller type: {ctrl_type}')

        self.state          = RobotState.IDLE
        self.prev_a         = 0
        self.prev_b         = 0
        self.prev_back      = 0
        self.prev_start     = 0
        self.prev_x         = 0
        self.prev_y         = 0
        self._walk_gait_idx    = 0
        self._active_walk_gait = _WALK_GAIT_CYCLE[0]

        # Self-righting state
        self._last_roll           = 0.0   # most recent IMU roll (degrees)
        self._righting_start      = 0.0   # time.time() when righting began
        self._righting_push_right = True  # which side pushes off the ground
        self._righting_inverted   = False # True when |roll| >= 120° (upside-down)

        # Jump / backflip state
        self._jump_start     = 0.0
        self._backflip_start = 0.0

        # Ramps — smoothly interpolate between poses
        self._current_angles       = [0.0] * 8   # last published angles (motor frame)
        self._sit_ramp_start       = 0.0
        self._sit_ramp_from        = [0.0] * 8
        self._deep_sit_ramp_start  = 0.0
        self._deep_sit_ramp_from   = [0.0] * 8

        # Positioning state — leg-by-leg capture on platform
        self._fb_pos              = [0.0] * 8   # latest motor-frame positions from Teensy
        self._leg_captured        = [False] * 4  # FR, FL, RR, RL
        self._positioning_angles  = [0.0] * 8   # IK-frame captured standing positions

        self.create_subscription(Joy,              'joy',          self._joy_callback,    10)
        self.create_subscription(Vector3,          'imu/euler',    self._imu_callback,    10)
        self.create_subscription(Bool,             'estop',        self._estop_callback,  10)
        self.create_subscription(Float32MultiArray,'/joint_states', self._fb_callback,    10)

        self.state_pub     = self.create_publisher(String,           'robot_state',  10)
        self.gait_pub      = self.create_publisher(Twist,            'gait_command', 10)
        self.pose_pub      = self.create_publisher(Vector3,          'body_pose',    10)
        self.joint_pub     = self.create_publisher(Float32MultiArray,'joint_angles', 10)
        self.enable_pub    = self.create_publisher(Bool,             'can_enable',   10)
        self.gait_type_pub = self.create_publisher(String,           'gait_type',    10)
        self.estop_pub     = self.create_publisher(Bool,             'estop_state',  10)

        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.02, self._timed_actions_tick)   # 50 Hz — matches gait loop

        self.get_logger().info(
            'State manager ready. E-STOP active.\n'
            '  Place robot on platform, then press Start to enter POSITIONING.\n'
            '  A=capture FR  B=capture FL  X=capture RR  Y=capture RL\n'
            '  Start (again) = confirm all legs and stand.'
        )
        self._set_state(RobotState.ESTOP)

    # ────────────────────────────────────────────────────────────────
    def _set_state(self, new_state: str):
        if new_state == self.state:
            return
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        self._publish_state()

        # Publish estop status
        estop_msg = Bool()
        estop_msg.data = (new_state == RobotState.ESTOP)
        self.estop_pub.publish(estop_msg)

        if new_state == RobotState.POSITIONING:
            self._leg_captured       = [False] * 4
            self._positioning_angles = [0.0] * 8
        elif new_state == RobotState.DEEP_SITTING:
            self._deep_sit_ramp_from  = list(self._current_angles)
            self._deep_sit_ramp_start = time.time()
        elif new_state == RobotState.SITTING:
            self._sit_ramp_from  = list(self._current_angles)
            self._sit_ramp_start = time.time()

        if new_state == RobotState.ESTOP:
            # Tell Teensy to exit motor mode → motors become torque-free
            msg = Bool()
            msg.data = False
            self.enable_pub.publish(msg)
        elif new_state in (RobotState.POSITIONING, RobotState.DEEP_SITTING,
                           RobotState.SITTING, RobotState.STANDING,
                           RobotState.RIGHTING, RobotState.AUTONOMOUS):
            # Re-enter motor mode if recovering from E-stop
            msg = Bool()
            msg.data = True
            self.enable_pub.publish(msg)

    def _joy_callback(self, msg: Joy):
        if not msg.buttons or not msg.axes:
            return

        # Read all one-shot buttons at the top so their prev_ trackers are
        # always updated on every callback, regardless of current state.
        # Without this, prev_b stays 0 while sitting, and the first callback
        # after transitioning to STANDING fires a jump if B was ever pressed.
        b_btn = msg.buttons[1]           if 1           < len(msg.buttons) else 0
        y_btn = msg.buttons[BTN_Y]       if BTN_Y       < len(msg.buttons) else 0
        rb    = msg.buttons[BTN_RB]      if BTN_RB      < len(msg.buttons) else 0

        back = msg.buttons[self._btn_back] if self._btn_back < len(msg.buttons) else 0
        if back == 1 and self.prev_back == 0:
            self._set_state(RobotState.ESTOP)
            self.get_logger().warn('E-STOP activated!')
        self.prev_back = back

        if self.state == RobotState.ESTOP:
            start = msg.buttons[self._btn_start] if self._btn_start < len(msg.buttons) else 0
            if start == 1 and self.prev_start == 0:
                self.get_logger().info('E-STOP cleared — entering POSITIONING.')
                self._set_state(RobotState.POSITIONING)
            self.prev_start = start

            # B button triggers self-righting from fallen position
            if b_btn == 1 and self.prev_b == 0:
                self._start_righting()
            self.prev_b = b_btn
            return

        if self.state == RobotState.POSITIONING:
            _LEG_BUTTONS = [
                (msg.buttons[self._btn_a] if self._btn_a < len(msg.buttons) else 0),  # FR
                (b_btn),                                                                 # FL
                (msg.buttons[self._btn_x] if self._btn_x < len(msg.buttons) else 0),  # RR
                (y_btn),                                                                 # RL
            ]
            _LEG_PREV = [self.prev_a, self.prev_b, self.prev_x, self.prev_y]
            _LEG_NAMES = ['FR', 'FL', 'RR', 'RL']
            for leg, (btn, prev) in enumerate(zip(_LEG_BUTTONS, _LEG_PREV)):
                if btn == 1 and prev == 0 and not self._leg_captured[leg]:
                    self._capture_leg(leg)
            self.prev_a = _LEG_BUTTONS[0]
            self.prev_b = _LEG_BUTTONS[1]
            self.prev_x = _LEG_BUTTONS[2]
            self.prev_y = _LEG_BUTTONS[3]

            start = msg.buttons[self._btn_start] if self._btn_start < len(msg.buttons) else 0
            if start == 1 and self.prev_start == 0:
                self._confirm_positioning()
            self.prev_start = start
            return

        a = msg.buttons[self._btn_a] if self._btn_a < len(msg.buttons) else 0
        if a == 1 and self.prev_a == 0:
            if self.state == RobotState.DEEP_SITTING:
                # Deep sit → sit: slow ramp (same duration as sit → deep sit)
                self._set_state(RobotState.SITTING)
            elif self.state in (RobotState.IDLE, RobotState.SITTING):
                self._set_state(RobotState.STANDING)
                self._cancel_front_stand_timer()
                self._publish_joint_angles(NEUTRAL_ANGLES)
            elif self.state in (RobotState.STANDING, RobotState.WALKING,
                                RobotState.AUTONOMOUS):
                self._cancel_front_stand_timer()
                self._exit_autonomous_if_needed()
                self._set_state(RobotState.SITTING)
        self.prev_a = a

        # Y button: toggle AUTONOMOUS mode on/off.
        if y_btn == 1 and self.prev_y == 0:
            if self.state in (RobotState.STANDING, RobotState.WALKING):
                self._enter_autonomous()
            elif self.state == RobotState.AUTONOMOUS:
                self._exit_autonomous()
        self.prev_y = y_btn

        # In AUTONOMOUS state joystick movement is handled by Nav2.
        # Always update prev_b before returning so it stays in sync.
        if self.state == RobotState.AUTONOMOUS:
            self.prev_b = b_btn
            return

        if self.state in (RobotState.STANDING, RobotState.WALKING):
            x_btn = msg.buttons[self._btn_x] if self._btn_x < len(msg.buttons) else 0
            if x_btn == 1 and self.prev_x == 0:
                self._walk_gait_idx = (self._walk_gait_idx + 1) % len(_WALK_GAIT_CYCLE)
                self._active_walk_gait = _WALK_GAIT_CYCLE[self._walk_gait_idx]
                self._publish_gait_type(self._active_walk_gait)
                self.get_logger().info(f'Walk gait → {self._active_walk_gait.name}')
            self.prev_x = x_btn

            # B button: jump forward.  B + RB: backflip.
            # (Y button is now used for autonomous toggle.)
            if b_btn == 1 and self.prev_b == 0:
                if rb:
                    self._start_backflip()
                else:
                    self._start_jump()

        # Always update prev_b — must happen for every state, not just STANDING/WALKING,
        # to prevent a stale prev_b=0 from firing jump on the next state transition.
        self.prev_b = b_btn

        if self.state not in (RobotState.STANDING, RobotState.WALKING):
            return

        lb = msg.buttons[BTN_LB] if BTN_LB < len(msg.buttons) else 0
        rb = msg.buttons[BTN_RB] if BTN_RB < len(msg.buttons) else 0

        def axis(i):
            return float(msg.axes[i]) if i < len(msg.axes) else 0.0

        def dz(v):
            return v if abs(v) > JOYSTICK_DEADZONE else 0.0

        vx  = -dz(axis(AXIS_LEFT_Y))
        vy  = -dz(axis(AXIS_LEFT_X))
        yaw = -dz(axis(AXIS_RIGHT_X))

        body_pitch = dz(axis(AXIS_RIGHT_Y)) * MAX_BODY_PITCH
        lt         = (1.0 - axis(AXIS_LT)) / 2.0
        rt         = (1.0 - axis(AXIS_RT)) / 2.0
        body_roll  = (rt - lt) * MAX_BODY_ROLL

        speed_mult = TURBO_MULTIPLIER if rb else 1.0
        moving = abs(vx) > 0 or abs(vy) > 0 or abs(yaw) > 0

        if lb and moving:
            if self.state == RobotState.STANDING:
                self._set_state(RobotState.WALKING)
                self._publish_gait_type(self._active_walk_gait)

            twist = Twist()
            twist.linear.x  = vx  * JOYSTICK_SCALE * speed_mult
            twist.linear.y  = vy  * JOYSTICK_SCALE * speed_mult
            twist.angular.z = yaw * TURN_SCALE      * speed_mult
            self.gait_pub.publish(twist)

        elif not moving and self.state == RobotState.WALKING:
            self._set_state(RobotState.STANDING)
            self.gait_pub.publish(Twist())
            self._publish_joint_angles(NEUTRAL_ANGLES)

        pose = Vector3()
        pose.x = body_roll
        pose.y = body_pitch
        self.pose_pub.publish(pose)

    # ── Positioning helpers ───────────────────────────────────────────

    def _fb_callback(self, msg: Float32MultiArray):
        """Track latest motor-frame positions from Teensy feedback."""
        if len(msg.data) >= 8:
            self._fb_pos = list(msg.data[0:8])

    def _capture_leg(self, leg: int):
        """Lock one leg at its current physical position."""
        i = leg * 2
        # fb_pos is in motor frame; convert to IK frame by applying direction
        d_sho = JOINT_DIRECTION.get((leg, 1), 1)
        d_kne = JOINT_DIRECTION.get((leg, 2), 1)
        self._positioning_angles[i]     = d_sho * self._fb_pos[i]
        self._positioning_angles[i + 1] = d_kne * self._fb_pos[i + 1]
        self._leg_captured[leg] = True
        names = ['FR', 'FL', 'RR', 'RL']
        self.get_logger().info(
            f'POSITIONING: {names[leg]} captured — '
            f'sho={self._positioning_angles[i]:.3f} rad  '
            f'kne={self._positioning_angles[i+1]:.3f} rad  '
            f'({sum(self._leg_captured)}/4 locked)'
        )

    def _confirm_positioning(self):
        """Capture any remaining free legs and transition to STANDING."""
        for leg in range(4):
            if not self._leg_captured[leg]:
                self._capture_leg(leg)
        self.get_logger().info('POSITIONING complete — transitioning to STANDING.')
        self._set_state(RobotState.STANDING)
        self._publish_joint_angles(self._positioning_angles)

    # ────────────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Vector3):
        self._last_roll = msg.x   # always track for righting direction
        if self.state not in (RobotState.STANDING, RobotState.WALKING,
                              RobotState.AUTONOMOUS):
            return
        if abs(msg.x) > 45.0 or abs(msg.y) > 45.0:
            self.get_logger().warn(
                f'Fall detected! roll={msg.x:.1f}° pitch={msg.y:.1f}° — self-righting'
            )
            self._start_righting()

    def _estop_callback(self, msg: Bool):
        """Callback for remote E-STOP commands."""
        if msg.data:
            if self.state != RobotState.ESTOP:
                self._set_state(RobotState.ESTOP)
                self.get_logger().warn('E-STOP activated via /estop topic!')
        else:
            # As per docstring, false on /estop clears the E-STOP state.
            # This provides a remote way to recover, same as the Start button.
            if self.state == RobotState.ESTOP:
                self.get_logger().info(
                    'E-STOP cleared via /estop topic — entering deep sit.'
                )
                self._set_state(RobotState.DEEP_SITTING)

    # ── Self-righting ─────────────────────────────────────────────────

    def _start_righting(self):
        """Begin the self-righting sequence.

        Determines push side from the most recent IMU roll:
          roll ≥ 0  → right side is lower → push from right (FR / RR)
          roll < 0  → left side is lower  → push from left  (FL / RL)
        Upside-down (|roll| ≈ 180°) defaults to pushing from right.
        """
        self._righting_push_right = (self._last_roll >= 0.0)
        self._righting_inverted   = (abs(self._last_roll) >= 120.0)
        self._righting_start = time.time()
        self._set_state(RobotState.RIGHTING)
        side = 'right' if self._righting_push_right else 'left'
        orient = 'inverted' if self._righting_inverted else 'on side'
        self.get_logger().info(
            f'Self-righting: {orient}, pushing {side} side (roll={self._last_roll:.1f}°)'
        )

    def _timed_actions_tick(self):
        """Phase sequencer called at 10 Hz for all timed motion states."""
        if self.state == RobotState.POSITIONING:
            # Build command: locked legs hold captured position, free legs track fb_pos
            angles = list(self._positioning_angles)
            for leg in range(4):
                if not self._leg_captured[leg]:
                    i = leg * 2
                    d_sho = JOINT_DIRECTION.get((leg, 1), 1)
                    d_kne = JOINT_DIRECTION.get((leg, 2), 1)
                    angles[i]     = d_sho * self._fb_pos[i]
                    angles[i + 1] = d_kne * self._fb_pos[i + 1]
            self._publish_joint_angles(angles)
        elif self.state == RobotState.DEEP_SITTING:
            elapsed = time.time() - self._deep_sit_ramp_start
            if elapsed < _SIT_RAMP_DURATION:
                t = elapsed / _SIT_RAMP_DURATION
                t = t * t * (3.0 - 2.0 * t)
                angles = [a + (b - a) * t
                          for a, b in zip(self._deep_sit_ramp_from, DEEP_SIT_ANGLES)]
                self._publish_joint_angles(angles)
            else:
                self._publish_joint_angles(DEEP_SIT_ANGLES)
        elif self.state == RobotState.SITTING:
            elapsed = time.time() - self._sit_ramp_start
            if elapsed < _SIT_RAMP_DURATION:
                # Smooth-step ease-in-out: 3t² − 2t³
                t = elapsed / _SIT_RAMP_DURATION
                t = t * t * (3.0 - 2.0 * t)
                angles = [a + (b - a) * t
                          for a, b in zip(self._sit_ramp_from, SIT_ANGLES)]
                self._publish_joint_angles(angles)
            else:
                self._publish_joint_angles(SIT_ANGLES)
        elif self.state == RobotState.RIGHTING:
            self._righting_phase()
        elif self.state == RobotState.JUMPING:
            self._jump_phase()
        elif self.state == RobotState.BACKFLIP:
            self._backflip_phase()

    def _righting_phase(self):
        """Self-righting timeline.

        0.0 – 0.7 s  tuck  : all legs pulled tight (reduce inertia)
        0.7 – 2.0 s  push  : down-side legs extend outward to lever the body
        2.0 – 2.5 s  tuck  : legs retuck as body rolls upright
        ≥ 2.5 s      done  : transition to SITTING
        """
        elapsed = time.time() - self._righting_start

        if elapsed < 0.7:
            self._publish_joint_angles(RIGHTING_TUCK_ANGLES)
        elif elapsed < 2.0:
            self._publish_joint_angles(self._make_push_angles(self._righting_push_right))
        elif elapsed < 2.5:
            self._publish_joint_angles(RIGHTING_TUCK_ANGLES)
        else:
            self._set_state(RobotState.SITTING)

    def _jump_phase(self):
        """Jump-forward timeline.

        0.00 – 0.30 s  crouch  : front legs deeper than rear (nose-down lean loads
                                  front feet, stops rear-heavy robot from rearing up)
        0.30 – 0.65 s  launch  : 0.35 s window — longer than needed for liftoff,
                                  so the position controller keeps pushing the joints
                                  toward the 46° target throughout ground contact,
                                  maximising the impulse delivered before feet leave
        0.65 – 0.95 s  tuck    : fold tight while airborne
        0.95 – 1.95 s  absorb  : asymmetric landing — front deep (absorbs
                                  forward momentum), rear extended (resists
                                  nose-down pitch that lifts the hind legs);
                                  1.00 s gives time to stabilise before
                                  transitioning to neutral
        ≥ 1.95 s       done    : return to STANDING
        """
        elapsed = time.time() - self._jump_start

        if elapsed < 0.30:
            self._publish_joint_angles(JUMP_CROUCH_ANGLES)
        elif elapsed < 0.65:
            self._publish_joint_angles(JUMP_LAUNCH_ANGLES)
        elif elapsed < 0.95:
            self._publish_joint_angles(JUMP_TUCK_ANGLES)
        elif elapsed < 1.95:
            self._publish_joint_angles(JUMP_LAND_ANGLES)
        else:
            self._set_state(RobotState.STANDING)
            self._publish_joint_angles(NEUTRAL_ANGLES)

    def _backflip_phase(self):
        """Backflip timeline.

        0.00 – 0.35 s  crouch    : all legs deep squat (geo_kne = −2.80)
        0.35 – 0.55 s  frontlift : front extends to 46°, rear stays crouched —
                                   front feet leave ground, body pivots nose-up
                                   around rear feet as fixed pivot (~90°)
        0.55 – 0.80 s  rearpush  : rear extends to 46°, front folds to tuck —
                                   explosive rear push launches body airborne,
                                   front I drops immediately
        0.80 – 1.70 s  tuck      : all legs 1.50, minimum I for full 360°
        1.70 – 1.95 s  reach     : extend to catch landing
        ≥ 1.95 s       done      : return to STANDING
        """
        elapsed = time.time() - self._backflip_start

        if elapsed < 0.35:
            self._publish_joint_angles(BACKFLIP_CROUCH_ANGLES)
        elif elapsed < 0.55:
            self._publish_joint_angles(BACKFLIP_FRONT_LIFT_ANGLES)
        elif elapsed < 0.80:
            self._publish_joint_angles(BACKFLIP_LAUNCH_ANGLES)
        elif elapsed < 1.70:
            self._publish_joint_angles(BACKFLIP_TUCK_ANGLES)
        elif elapsed < 1.95:
            self._publish_joint_angles(BACKFLIP_LAND_ANGLES)
        else:
            self._set_state(RobotState.STANDING)
            self._publish_joint_angles(NEUTRAL_ANGLES)

    def _start_jump(self):
        self._jump_start = time.time()
        self._set_state(RobotState.JUMPING)
        self.get_logger().info('Jump forward initiated')

    def _start_backflip(self):
        self._backflip_start = time.time()
        self._set_state(RobotState.BACKFLIP)
        self.get_logger().info('Backflip initiated')

    def _make_push_angles(self, push_right: bool) -> list:
        """Return 8 motor-command angles for the righting push phase (8DOF, no hips).

        push leg  – shoulder/knee angled toward the ground.
        tuck leg  – shoulder/knee tightly folded.

        When inverted (|roll| >= 120°) the legs must reach over the body to
        touch the ground, requiring a large shoulder angle:
          shoulder_motor=2.40 → geo≈2.40, knee_motor=1.40 → geo_kne≈-1.00
          foot_z_body = -L*(cos(2.40) + cos(1.40)) ≈ +0.18 m  (toward ground)
        When on side, all hips rotate the same world direction so inertia
        of the tuck legs assists the roll.  Push legs use shoulder ~52°
        back with a sprung knee:  shoulder_motor=0.90, knee_motor=-0.90
        """
        # push legs always get motor +0.6, tuck legs always -0.6.
        # sim_bridge negates FL/RL hips, so:
        #   push_right: FR/RR +0.6 → geo rightward; FL/RL -0.6 → geo rightward ✓
        #   push_left:  FL/RL +0.6 → geo leftward;  FR/RR -0.6 → geo leftward ✓
        # All four hips move in the same world direction regardless of push side.
        if self._righting_inverted:
            push = [-2.70, -1.1346]   # legs reach over body to ground
            tuck = [ 1.30, -1.1346]   # folded, knee at sit position
        else:
            push = [ 2.70, -1.1346]   # shoulder sweeps above body (~155°)
            tuck = [ 1.30, -1.1346]   # folded, knee at sit position
        if push_right:
            return push + tuck + push + tuck   # FR push, FL tuck, RR push, RL tuck
        return tuck + push + tuck + push       # FR tuck, FL push, RR tuck, RL push

    # ── Autonomous mode ───────────────────────────────────────────────

    def _enter_autonomous(self):
        """Switch to AUTONOMOUS: Nav2 / autonomous_bridge_node drives the robot."""
        self.get_logger().info(
            'Entering AUTONOMOUS mode — Nav2 is now in control. '
            'Press Y again to return to manual, or A to sit down.'
        )
        self._set_state(RobotState.AUTONOMOUS)
        # Force TROT gait so gait_node is ready to execute Nav2 velocities.
        self._publish_gait_type(GaitType.TROT)

    def _exit_autonomous(self):
        """Return from AUTONOMOUS to manual STANDING."""
        self.get_logger().info('Exiting AUTONOMOUS mode — returning manual control.')
        # Stop any ongoing motion before handing back to the operator.
        self.gait_pub.publish(Twist())
        self._set_state(RobotState.STANDING)
        self._publish_joint_angles(NEUTRAL_ANGLES)

    def _exit_autonomous_if_needed(self):
        """Call before any state transition that should implicitly leave AUTONOMOUS."""
        if self.state == RobotState.AUTONOMOUS:
            self.gait_pub.publish(Twist())

    # ────────────────────────────────────────────────────────────────
    def _cancel_front_stand_timer(self):
        pass  # retained for any in-flight timer cancellation at sit

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_gait_type(self, gait_type: GaitType):
        msg = String()
        msg.data = gait_type.name
        self.gait_type_pub.publish(msg)

    def _publish_joint_angles(self, angles):
        self._current_angles = list(angles)
        msg = Float32MultiArray()
        motor = []
        for leg in range(4):
            for joint in [1, 2]:   # shoulder=1, knee=2 (hip removed, 8DOF)
                i = leg * 2 + (joint - 1)
                d = JOINT_DIRECTION.get((leg, joint), 1)
                motor.append(float(d * angles[i]))
        msg.data = motor
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
