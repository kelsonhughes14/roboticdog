"""
gait_node.py
------------
ROS2 node that runs the gait generator and inverse kinematics.

Subscribed topics:
  /gait_command   (geometry_msgs/Twist)   — velocity command from state manager
  /body_pose      (geometry_msgs/Vector3) — roll, pitch, yaw from controller
  /robot_state    (std_msgs/String)       — current robot state

Published topics:
  /servo_angles   (std_msgs/Float32MultiArray) — 12 servo angles to servo_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Vector3

from spotmicro.gait_generator import GaitGenerator, GaitType
from spotmicro.kinematics import compute_all_legs
from spotmicro.state_manager import RobotState
from spotmicro.robot_config import NEUTRAL_ANGLES, WALK_FORWARD_LEAN


CONTROL_RATE_HZ = 50   # 50 Hz control loop


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')

        self.gait      = GaitGenerator()
        self.vx        = 0.0
        self.vy        = 0.0
        self.yaw       = 0.0
        self.roll      = 0.0
        self.pitch     = 0.0
        self.robot_state    = RobotState.SITTING
        self.walk_gait_type = GaitType.WALK   # default walk gait

        # ── Subscriptions ────────────────────────────────────────────
        self.create_subscription(Twist,  'gait_command', self._cmd_callback,      10)
        self.create_subscription(Vector3,'body_pose',    self._pose_callback,     10)
        self.create_subscription(String, 'robot_state',  self._state_callback,    10)
        self.create_subscription(String, 'gait_type',    self._gait_type_callback, 10)

        # ── Publisher ─────────────────────────────────────────────────
        self.servo_pub = self.create_publisher(
            Float32MultiArray, 'servo_angles', 10
        )

        # ── Control loop ─────────────────────────────────────────────
        period = 1.0 / CONTROL_RATE_HZ
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'Gait node running at {CONTROL_RATE_HZ} Hz'
        )

    # ────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Twist):
        self.vx  = msg.linear.x
        self.vy  = msg.linear.y
        self.yaw = msg.angular.z

    def _pose_callback(self, msg: Vector3):
        self.roll  = msg.x
        self.pitch = msg.y

    def _gait_type_callback(self, msg: String):
        try:
            gt = GaitType[msg.data]
        except KeyError:
            self.get_logger().warn(f'Unknown gait type: {msg.data}')
            return
        self.walk_gait_type = gt
        if self.robot_state == RobotState.WALKING:
            self.gait.set_gait(gt)
            self.get_logger().info(f'Switched to {gt.name} gait')

    def _state_callback(self, msg: String):
        new_state = msg.data

        if new_state == self.robot_state:
            return

        self.robot_state = new_state

        if new_state == RobotState.STANDING:
            self.gait.set_gait(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0

        elif new_state == RobotState.WALKING:
            self.gait.set_gait(self.walk_gait_type)

        elif new_state in (RobotState.SITTING, RobotState.ESTOP,
                           RobotState.IDLE):
            self.gait.set_gait(GaitType.STAND)
            self.vx = self.vy = self.yaw = 0.0

    # ────────────────────────────────────────────────────────────────
    def _control_loop(self):
        # Don't run IK when sitting or in E-stop
        if self.robot_state in (RobotState.SITTING, RobotState.ESTOP,
                                RobotState.IDLE):
            return

        # When standing still with no body tilt, hold the pose set by
        # state_manager (NEUTRAL_ANGLES) rather than overwriting with IK.
        if self.robot_state == RobotState.STANDING:
            no_move = abs(self.vx) < 0.001 and abs(self.vy) < 0.001 and abs(self.yaw) < 0.001
            no_tilt = abs(self.roll) < 0.5 and abs(self.pitch) < 0.5
            if no_move and no_tilt:
                return

        # Lean nose-down proportional to forward velocity to prevent backward tipping.
        # Negative pitch = nose down (positive pitch = nose up in this convention).
        lean = -self.vx * WALK_FORWARD_LEAN if self.robot_state == RobotState.WALKING else 0.0

        # Get foot positions from gait generator
        foot_positions = self.gait.update(
            vx=self.vx,
            vy=self.vy,
            yaw=self.yaw,
            body_roll=self.roll,
            body_pitch=self.pitch + lean,
        )

        # Run IK to convert foot positions to servo angles
        try:
            angles = compute_all_legs(foot_positions)
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            angles = NEUTRAL_ANGLES

        # Publish
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.servo_pub.publish(msg)


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
