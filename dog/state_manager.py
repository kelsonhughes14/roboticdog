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

Subscribed topics:
  /joy                        (sensor_msgs/Joy)
  /estop                      (std_msgs/Bool)  — true=trigger ESTOP, false=clear ESTOP
Published topics:
  /robot_state                (std_msgs/String)
  /estop_state                (std_msgs/Bool)   — true if robot is in ESTOP state
  /gait_command               (geometry_msgs/Twist)
  /body_pose                  (geometry_msgs/Vector3)
  /joint_angles               (std_msgs/Float32MultiArray)
  /can_enable                 (std_msgs/Bool)   — true=enter motor mode, false=exit
  /gait_type                  (std_msgs/String) — current gait type name
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
    NEUTRAL_ANGLES, SIT_ANGLES,
    JUMP_CROUCH_ANGLES, JUMP_LAUNCH_ANGLES, JUMP_TUCK_ANGLES, JUMP_LAND_ANGLES,
    BACKFLIP_CROUCH_ANGLES, BACKFLIP_FRONT_LIFT_ANGLES, BACKFLIP_LAUNCH_ANGLES,
    BACKFLIP_TUCK_ANGLES, BACKFLIP_LAND_ANGLES,
)
from dog.gait_generator import GaitType

_WALK_GAIT_CYCLE = [GaitType.TROT, GaitType.GALLOP, GaitType.WALK, GaitType.CRAWL, GaitType.TURTLE]

_SIT_RAMP_DURATION          = 2.0   # seconds — time to ease into the sit position
_STOP_RAMP_DURATION         = 0.40  # seconds — time to ease back to standing after walking

# Three-phase stand-up durations
_STANDUP_PHASE1_DURATION = 1.0   # lean: shoulders swing flush with body (forward lean)
_STANDUP_PHASE2_DURATION = 1.0   # push: knees extend to lift body off ground
_STANDUP_PHASE3_DURATION = 1.0   # normalize: shoulders settle to neutral standing angle
_STANDUP_FLUSH_ANGLE     = 0.85    # ~74° — upper leg leans forward enough to shift CoM without over-rotating

# Gains published to Teensy during POSITIONING (torque-free) and STANDING
_POSITIONING_KP = 0.0
_POSITIONING_KD = 0.0
_STANDING_KP    = 45.0
_STANDING_KD    =  2.0


class RobotState:
    IDLE         = 'IDLE'
    POSITIONING  = 'POSITIONING'   # motors off; operator manually positions legs, then presses Start
    SITTING      = 'SITTING'
    STANDING     = 'STANDING'
    WALKING      = 'WALKING'
    AUTONOMOUS   = 'AUTONOMOUS'
    ESTOP        = 'ESTOP'
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
        self.prev_rb        = 0
        self._positioning_target = 'sit'   # 'sit' or 'stand' — toggled by RB in ESTOP
        self._walk_gait_idx    = 0
        self._active_walk_gait = _WALK_GAIT_CYCLE[0]

        # Jump / backflip state
        self._jump_start     = 0.0
        self._backflip_start = 0.0

        # Ramps — smoothly interpolate between poses
        self._current_angles       = [0.0] * 8   # last published angles (motor frame)
        self._sit_ramp_start       = 0.0
        self._sit_ramp_from        = [0.0] * 8

        self._fb_pos              = [0.0] * 8   # latest motor-frame positions from Teensy
        self._fb_received         = False        # True once any /joint_states arrives
        self._fb_last_time        = 0.0          # wall time of last /joint_states msg
        self._fb_was_absent       = False        # True while no feedback for >3 s
        self._leg_captured        = [False] * 4  # FR, FL, RR, RL
        self._positioning_angles  = [0.0]  * 8  # captured IK-frame angles per leg
        self._standing_target     = list(NEUTRAL_ANGLES)  # updated on POSITIONING confirm
        self._sit_target          = list(SIT_ANGLES)      # overridden by POSITIONING capture
        self._stop_ramp_start     = 0.0
        self._stop_ramp_from      = [0.0] * 8
        self._stop_ramp_active    = False
        self._standup_phase       = 0       # 0=inactive, 1=lean, 2=push, 3=normalize
        self._standup_phase_start = 0.0
        self._standup_phase_from  = [0.0] * 8
        self._standup_phase_to    = [0.0] * 8

        self.create_subscription(Joy,              'joy',          self._joy_callback,    10)
        self.create_subscription(Bool,             'estop',        self._estop_callback,  10)
        self.create_subscription(Float32MultiArray,'/joint_states', self._fb_callback,    10)

        self.state_pub     = self.create_publisher(String,           'robot_state',  10)
        self.gait_pub      = self.create_publisher(Twist,            'gait_command', 10)
        self.pose_pub      = self.create_publisher(Vector3,          'body_pose',    10)
        self.joint_pub      = self.create_publisher(Float32MultiArray,'joint_angles',   10)
        self.gains_pub      = self.create_publisher(Float32MultiArray,'joint_gains',    10)
        self.home_pub       = self.create_publisher(Float32MultiArray,'standing_home',  10)
        self.enable_pub     = self.create_publisher(Bool,             'can_enable',     10)
        self.gait_type_pub  = self.create_publisher(String,           'gait_type',      10)
        self.estop_pub      = self.create_publisher(Bool,             'estop_state',    10)

        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.01, self._timed_actions_tick)   # 100 Hz — matches gait loop

        self.get_logger().info(
            'State manager ready. E-STOP active.\n'
            '  Place robot down in natural sitting position, then press Start.\n'
            '  A=capture FR  B=capture FL  X=capture RR  Y=capture RL\n'
            '  Start (again) = confirm all legs and sit (A to stand up).'
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
            self._positioning_angles = [0.0]  * 8
            self._fb_received        = False
        elif new_state == RobotState.SITTING:
            self._sit_ramp_from  = list(self._current_angles)
            self._sit_ramp_start = time.time()

        if new_state == RobotState.ESTOP:
            # Tell Teensy to exit motor mode → motors become torque-free
            msg = Bool()
            msg.data = False
            self.enable_pub.publish(msg)
        elif new_state == RobotState.POSITIONING:
            # Enable MIT mode with zero gains — legs are completely torque-free
            # so the operator can move them freely, but CAN feedback is live.
            enable_msg = Bool()
            enable_msg.data = True
            self.enable_pub.publish(enable_msg)
            self._publish_gains(_POSITIONING_KP, _POSITIONING_KD)
        elif new_state in (RobotState.SITTING, RobotState.STANDING,
                           RobotState.AUTONOMOUS):
            # Re-enter motor mode with standing gains
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
            if rb == 1 and self.prev_rb == 0:
                self._positioning_target = 'stand' if self._positioning_target == 'sit' else 'sit'
                self.get_logger().info(
                    f'POSITIONING target → {self._positioning_target.upper()} '
                    f'(RB to cycle; press Start to enter POSITIONING)'
                )
            self.prev_rb = rb
            start = msg.buttons[self._btn_start] if self._btn_start < len(msg.buttons) else 0
            if start == 1 and self.prev_start == 0:
                self.get_logger().info(
                    f'E-STOP cleared — entering POSITIONING (target: {self._positioning_target.upper()}).'
                )
                self._set_state(RobotState.POSITIONING)
            self.prev_start = start
            self.prev_b = b_btn
            return

        if self.state == RobotState.POSITIONING:
            leg_buttons = [
                msg.buttons[self._btn_a] if self._btn_a < len(msg.buttons) else 0,  # FR
                b_btn,                                                                # FL
                msg.buttons[self._btn_x] if self._btn_x < len(msg.buttons) else 0,  # RR
                y_btn,                                                                # RL
            ]
            leg_prev = [self.prev_a, self.prev_b, self.prev_x, self.prev_y]
            for leg, (btn, prev) in enumerate(zip(leg_buttons, leg_prev)):
                if btn == 1 and prev == 0 and not self._leg_captured[leg]:
                    self._capture_leg(leg)
            self.prev_a = leg_buttons[0]
            self.prev_b = leg_buttons[1]
            self.prev_x = leg_buttons[2]
            self.prev_y = leg_buttons[3]

            start = msg.buttons[self._btn_start] if self._btn_start < len(msg.buttons) else 0
            if start == 1 and self.prev_start == 0:
                self._confirm_positioning()
            self.prev_start = start
            return

        a = msg.buttons[self._btn_a] if self._btn_a < len(msg.buttons) else 0
        if a == 1 and self.prev_a == 0:
            if self.state in (RobotState.IDLE, RobotState.SITTING):
                # Phase 1: swing shoulders flush with body (π/2) while keeping
                # geometric knee constant so feet stay planted.  This shifts
                # weight forward off the knee joints before we push up.
                self._standup_phase       = 1
                self._standup_phase_start = time.time()
                self._standup_phase_from  = list(self._current_angles)
                p1_to = list(self._current_angles)
                for leg in range(4):
                    sho = leg * 2
                    kne = leg * 2 + 1
                    geo_kne = self._current_angles[kne] - self._current_angles[sho]
                    p1_to[sho] = _STANDUP_FLUSH_ANGLE
                    p1_to[kne] = _STANDUP_FLUSH_ANGLE + geo_kne
                self._standup_phase_to = p1_to
                self._set_state(RobotState.STANDING)
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
            # Smoothly ramp back from current leg position to the confirmed
            # standing position instead of snapping there instantly.
            if self._fb_received:
                self._stop_ramp_from   = self._to_motor_frame(list(self._fb_pos[:8]))
                self._stop_ramp_start  = time.time()
                self._stop_ramp_active = True
                self._current_angles   = list(self._stop_ramp_from)

        pose = Vector3()
        pose.x = body_roll
        pose.y = body_pitch
        self.pose_pub.publish(pose)

    # ── Positioning helpers ───────────────────────────────────────────

    def _fb_callback(self, msg: Float32MultiArray):
        """Track latest motor-frame positions from Teensy feedback."""
        if len(msg.data) >= 8:
            self._fb_pos = list(msg.data[0:8])
            self._fb_received = True
            now = time.time()
            if self._fb_was_absent:
                # Teensy just reconnected — re-enable motors if we're in an active state
                self._fb_was_absent = False
                if self.state in (RobotState.STANDING, RobotState.WALKING,
                                  RobotState.SITTING, RobotState.AUTONOMOUS):
                    self.get_logger().warn(
                        'Teensy reconnected — re-enabling motors.'
                    )
                    msg_en = Bool()
                    msg_en.data = True
                    self.enable_pub.publish(msg_en)
            self._fb_last_time = now

    def _capture_leg(self, leg: int):
        """Record the current physical position of one leg."""
        if not self._fb_received:
            self.get_logger().warn('POSITIONING: no motor feedback yet — waiting for Teensy connection')
            return
        i = leg * 2
        d_sho = JOINT_DIRECTION.get((leg, 1), 1)
        d_kne = JOINT_DIRECTION.get((leg, 2), 1)
        self._positioning_angles[i]     = d_sho * self._fb_pos[i]
        self._positioning_angles[i + 1] = d_kne * self._fb_pos[i + 1]
        self._leg_captured[leg] = True
        names = ['FR', 'FL', 'RR', 'RL']
        self.get_logger().info(
            f'POSITIONING: {names[leg]} locked — '
            f'sho={self._positioning_angles[i]:.3f} rad  '
            f'kne={self._positioning_angles[i+1]:.3f} rad  '
            f'({sum(self._leg_captured)}/4 locked)'
        )

    def _confirm_positioning(self):
        """Capture any remaining free legs and transition based on _positioning_target."""
        for leg in range(4):
            if not self._leg_captured[leg]:
                self._capture_leg(leg)

        self._publish_gains(_STANDING_KP, _STANDING_KD)

        if self._positioning_target == 'stand':
            # Captured position IS the standing zero — motors hold it in place now.
            self.get_logger().info(
                'POSITIONING complete (STAND mode) — captured angles are the standing zero.'
            )
            self._standing_target = list(self._positioning_angles)
            self._sit_target      = list(SIT_ANGLES)
            self._current_angles  = list(self._positioning_angles)
            # Tell gait_node the standing home in motor frame (captured angles).
            home_msg = Float32MultiArray()
            home_msg.data = self._to_motor_frame(list(self._positioning_angles))
            self.home_pub.publish(home_msg)
            self._standup_phase   = 0   # already at target — no standup sequence needed
            self._set_state(RobotState.STANDING)
            self._publish_joint_angles(self._current_angles)
        else:
            # Default: captured position is the sitting position.
            self.get_logger().info('POSITIONING complete — transitioning to SITTING.')
            self._sit_target      = list(self._positioning_angles)
            self._standing_target = list(NEUTRAL_ANGLES)
            # Tell gait_node the standing home in motor frame (NEUTRAL_ANGLES).
            home_msg = Float32MultiArray()
            home_msg.data = self._to_motor_frame(list(NEUTRAL_ANGLES))
            self.home_pub.publish(home_msg)
            self._set_state(RobotState.SITTING)
            # Already at the sitting position — hold it.
            self._publish_joint_angles(self._sit_target)

        # Reset mode to default for next time.
        self._positioning_target = 'sit'

    # ────────────────────────────────────────────────────────────────
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
                    'E-STOP cleared via /estop topic — entering POSITIONING.'
                )
                self._set_state(RobotState.POSITIONING)

    def _publish_gains(self, kp: float, kd: float):
        msg = Float32MultiArray()
        msg.data = [kp, kd]
        self.gains_pub.publish(msg)

    def _timed_actions_tick(self):
        """Phase sequencer called at 100 Hz for all timed motion states."""
        # Detect when Teensy feedback goes silent (agent disconnected / watchdog fired)
        if (self._fb_received and not self._fb_was_absent
                and (time.time() - self._fb_last_time) > 3.0):
            self._fb_was_absent = True

        if self.state == RobotState.POSITIONING:
            if not self._fb_received:
                # Teensy not yet connected or enable message was dropped — retry
                enable_msg = Bool()
                enable_msg.data = True
                self.enable_pub.publish(enable_msg)
                self._publish_gains(_POSITIONING_KP, _POSITIONING_KD)
                return
            # Send current fb_pos back as the command — zero gains mean zero torque,
            # so legs are free to move. This keeps CAN feedback flowing.
            angles = list(self._positioning_angles)
            for leg in range(4):
                if not self._leg_captured[leg]:
                    i = leg * 2
                    d_sho = JOINT_DIRECTION.get((leg, 1), 1)
                    d_kne = JOINT_DIRECTION.get((leg, 2), 1)
                    angles[i]     = d_sho * self._fb_pos[i]
                    angles[i + 1] = d_kne * self._fb_pos[i + 1]
            self._publish_joint_angles(angles)
        elif self.state == RobotState.STANDING:
            if self._standup_phase > 0:
                durations = [0.0, _STANDUP_PHASE1_DURATION,
                             _STANDUP_PHASE2_DURATION, _STANDUP_PHASE3_DURATION]
                dur     = durations[self._standup_phase]
                elapsed = time.time() - self._standup_phase_start
                if elapsed < dur:
                    t = elapsed / dur
                    t = t * t * (3.0 - 2.0 * t)   # smooth-step ease-in-out
                    angles = [a + (b - a) * t
                              for a, b in zip(self._standup_phase_from, self._standup_phase_to)]
                    self._publish_joint_angles(angles)
                else:
                    self._standup_phase_from = list(self._standup_phase_to)
                    if self._standup_phase == 1:
                        # Phase 2: knees extend to push body up; shoulders stay flush
                        self._standup_phase       = 2
                        self._standup_phase_start = time.time()
                        p2_to = list(self._standup_phase_from)
                        for leg in range(4):
                            sho = leg * 2
                            kne = leg * 2 + 1
                            geo_kne_stand = NEUTRAL_ANGLES[kne] - NEUTRAL_ANGLES[sho]
                            p2_to[kne] = _STANDUP_FLUSH_ANGLE + geo_kne_stand
                        self._standup_phase_to = p2_to
                        self._publish_joint_angles(self._standup_phase_from)
                    elif self._standup_phase == 2:
                        # Phase 3: shoulders return to mechanical zero (0.0 rad);
                        # knees hold their phase-2 position unchanged.
                        self._standup_phase       = 3
                        self._standup_phase_start = time.time()
                        p3_to = list(self._standup_phase_from)
                        for leg in range(4):
                            p3_to[leg * 2] = 0.0   # shoulder → 0; knee index untouched
                        self._standup_phase_to = p3_to
                        self._publish_joint_angles(self._standup_phase_from)
                    else:
                        # Phase 3 done: standing complete
                        self._standup_phase   = 0
                        self._current_angles  = list(self._standup_phase_to)
                        self._standing_target = list(self._standup_phase_to)
                        self._publish_joint_angles(self._current_angles)
                        home_msg = Float32MultiArray()
                        home_msg.data = self._to_motor_frame(self._standup_phase_to)
                        self.home_pub.publish(home_msg)
            elif self._stop_ramp_active:
                elapsed = time.time() - self._stop_ramp_start
                if elapsed < _STOP_RAMP_DURATION:
                    t = elapsed / _STOP_RAMP_DURATION
                    t = t * t * (3.0 - 2.0 * t)   # smooth-step ease-in-out
                    angles = [a + (b - a) * t
                              for a, b in zip(self._stop_ramp_from, self._standing_target)]
                    self._publish_joint_angles(angles)
                else:
                    self._stop_ramp_active = False
                    self._current_angles   = list(self._standing_target)
                    self._publish_joint_angles(self._current_angles)
            else:
                # Keep sending current commanded angles so motors stay active.
                # gait_node overrides this when walking.
                self._publish_joint_angles(self._current_angles)
        elif self.state == RobotState.SITTING:
            elapsed = time.time() - self._sit_ramp_start
            if elapsed < _SIT_RAMP_DURATION:
                # Smooth-step ease-in-out: 3t² − 2t³
                t = elapsed / _SIT_RAMP_DURATION
                t = t * t * (3.0 - 2.0 * t)
                angles = [a + (b - a) * t
                          for a, b in zip(self._sit_ramp_from, self._sit_target)]
                self._publish_joint_angles(angles)
            else:
                self._publish_joint_angles(self._sit_target)
        elif self.state == RobotState.JUMPING:
            self._jump_phase()
        elif self.state == RobotState.BACKFLIP:
            self._backflip_phase()

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
        self._standup_phase    = 0
        self._stop_ramp_active = False

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_gait_type(self, gait_type: GaitType):
        msg = String()
        msg.data = gait_type.name
        self.gait_type_pub.publish(msg)

    def _to_motor_frame(self, ik_angles) -> list:
        """Convert IK-frame angles to motor-frame (applies JOINT_DIRECTION)."""
        motor = []
        for leg in range(4):
            for joint in [1, 2]:
                i = leg * 2 + (joint - 1)
                d = JOINT_DIRECTION.get((leg, joint), 1)
                motor.append(float(d * ik_angles[i]))
        return motor

    def _publish_joint_angles(self, angles):
        self._current_angles = list(angles)
        msg = Float32MultiArray()
        motor = self._to_motor_frame(angles)
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
