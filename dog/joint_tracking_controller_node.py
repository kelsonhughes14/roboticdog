"""
joint_tracking_controller_node.py
---------------------------------
ROS-side outer-loop joint tracking controller.

Purpose
-------
Closes an additional control loop around the motor driver's MIT position loop:

  desired (/joint_angles_desired) + correction(error) -> /joint_angles

where:
  error = desired - encoder_feedback

This helps when actuator friction/load/compliance causes steady-state tracking
error between what gait/state logic commands and where the joints actually are.

Topics
------
Subscribed:
  /joint_angles_desired   (std_msgs/Float32MultiArray, 8)  motor-frame desired
  /joint_states           (std_msgs/Float32MultiArray, 32) motor-frame feedback
  /robot_state            (std_msgs/String)
  /can_enable             (std_msgs/Bool)

Published:
  /joint_angles           (std_msgs/Float32MultiArray, 8) corrected motor command
  /joint_position_error   (std_msgs/Float32MultiArray, 8) signed error rad
  /joint_tracking_ok      (std_msgs/Bool) true when within tolerance
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from dog.robot_config import JOINT_ANGLE_MIN, JOINT_ANGLE_MAX

N_MOTORS = 8

_ACTIVE_STATES = {'STANDING', 'WALKING', 'AUTONOMOUS'}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class JointTrackingControllerNode(Node):
    def __init__(self):
        super().__init__('joint_tracking_controller')

        # Conservative defaults to avoid fighting the motor's internal MIT loop.
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('outer_kp', 0.25)
        self.declare_parameter('outer_ki', 0.10)
        self.declare_parameter('error_deadband_rad', 0.02)
        self.declare_parameter('ok_error_rad', 0.08)
        self.declare_parameter('max_integral_rad', 0.20)
        self.declare_parameter('max_correction_rad', 0.35)
        self.declare_parameter('max_cmd_step_rad', 0.05)
        # Set to <=0 to disable freshness timeout checks after first sample.
        self.declare_parameter('feedback_timeout_s', 0.0)
        self.declare_parameter('desired_timeout_s', 0.0)

        self._rate_hz = float(self.get_parameter('control_rate_hz').value)
        self._kp = float(self.get_parameter('outer_kp').value)
        self._ki = float(self.get_parameter('outer_ki').value)
        self._deadband = float(self.get_parameter('error_deadband_rad').value)
        self._ok_err = float(self.get_parameter('ok_error_rad').value)
        self._i_lim = float(self.get_parameter('max_integral_rad').value)
        self._u_lim = float(self.get_parameter('max_correction_rad').value)
        self._step_lim = float(self.get_parameter('max_cmd_step_rad').value)
        self._fb_timeout = float(self.get_parameter('feedback_timeout_s').value)
        self._des_timeout = float(self.get_parameter('desired_timeout_s').value)

        self._desired = [0.0] * N_MOTORS
        self._fb = [0.0] * N_MOTORS
        self._integral = [0.0] * N_MOTORS
        self._last_out = [0.0] * N_MOTORS
        self._last_error = [0.0] * N_MOTORS
        self._last_desired_t = 0.0
        self._last_fb_t = 0.0
        self._have_desired = False
        self._have_fb = False
        self._last_tick_t = time.time()
        self._robot_state = 'ESTOP'
        self._can_enabled = False

        self.create_subscription(
            Float32MultiArray, 'joint_angles_desired', self._desired_cb, 20
        )
        self.create_subscription(
            Float32MultiArray, '/joint_states', self._fb_cb, 20
        )
        self.create_subscription(
            String, 'robot_state', self._state_cb, 20
        )
        self.create_subscription(
            Bool, 'can_enable', self._enable_cb, 20
        )

        self._joint_pub = self.create_publisher(Float32MultiArray, 'joint_angles', 20)
        self._err_pub = self.create_publisher(Float32MultiArray, 'joint_position_error', 20)
        self._ok_pub = self.create_publisher(Bool, 'joint_tracking_ok', 20)

        self.create_timer(1.0 / max(self._rate_hz, 1.0), self._tick)

        self.get_logger().info(
            'joint_tracking_controller ready '
            f'(rate={self._rate_hz:.1f}Hz kp={self._kp:.3f} ki={self._ki:.3f} '
            f'deadband={self._deadband:.3f}rad max_u={self._u_lim:.3f}rad)'
        )

    def _desired_cb(self, msg: Float32MultiArray):
        if len(msg.data) < N_MOTORS:
            return
        self._desired = [float(v) for v in msg.data[:N_MOTORS]]
        self._last_desired_t = time.time()
        self._have_desired = True

    def _fb_cb(self, msg: Float32MultiArray):
        # /joint_states layout from Teensy: pos[0:8], vel[8:16], torque[16:24], temp[24:32]
        if len(msg.data) < N_MOTORS:
            return
        self._fb = [float(v) for v in msg.data[:N_MOTORS]]
        self._last_fb_t = time.time()
        self._have_fb = True

    def _state_cb(self, msg: String):
        self._robot_state = msg.data

    def _enable_cb(self, msg: Bool):
        self._can_enabled = bool(msg.data)

    def _tracking_active(self) -> bool:
        return self._can_enabled and (self._robot_state in _ACTIVE_STATES)

    @staticmethod
    def _is_fresh(last_t: float, timeout_s: float, now_t: float) -> bool:
        if last_t <= 0.0:
            return False
        if timeout_s <= 0.0:
            return True
        return (now_t - last_t) <= timeout_s

    def _tick(self):
        now = time.time()
        dt = max(1e-4, now - self._last_tick_t)
        self._last_tick_t = now

        fresh_des = self._is_fresh(self._last_desired_t, self._des_timeout, now)
        fresh_fb = self._is_fresh(self._last_fb_t, self._fb_timeout, now)
        active = (
            self._tracking_active()
            and self._have_desired
            and self._have_fb
            and fresh_des
            and fresh_fb
        )

        # If inactive or stale, pass desired through and reset controller memory.
        if not active:
            self._integral = [0.0] * N_MOTORS
            out = list(self._desired)
            self._last_error = [0.0] * N_MOTORS
        else:
            out = [0.0] * N_MOTORS
            for i in range(N_MOTORS):
                err = self._desired[i] - self._fb[i]
                self._last_error[i] = err

                if abs(err) <= self._deadband:
                    # Bleed integrator near zero so old bias does not linger.
                    self._integral[i] *= 0.98
                    p = 0.0
                else:
                    p = self._kp * err
                    self._integral[i] += self._ki * err * dt
                    self._integral[i] = _clamp(self._integral[i], -self._i_lim, self._i_lim)

                u = _clamp(p + self._integral[i], -self._u_lim, self._u_lim)
                raw = self._desired[i] + u

                # Per-step limiter to keep ROS outer-loop smooth.
                step = raw - self._last_out[i]
                step = _clamp(step, -self._step_lim, self._step_lim)
                out[i] = _clamp(self._last_out[i] + step, JOINT_ANGLE_MIN, JOINT_ANGLE_MAX)

        self._last_out = list(out)

        cmd_msg = Float32MultiArray()
        cmd_msg.data = out
        self._joint_pub.publish(cmd_msg)

        err_msg = Float32MultiArray()
        err_msg.data = list(self._last_error)
        self._err_pub.publish(err_msg)

        max_err = max(abs(e) for e in self._last_error) if self._last_error else 0.0
        ok_msg = Bool()
        ok_msg.data = bool(active and (max_err <= self._ok_err))
        self._ok_pub.publish(ok_msg)

        # if active and max_err > self._ok_err:
        #     self.get_logger().warn(
        #         f'joint tracking error high: max |e|={max_err:.3f} rad',
        #         throttle_duration_sec=0.5,
        #     )


def main(args=None):
    rclpy.init(args=args)
    node = JointTrackingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
