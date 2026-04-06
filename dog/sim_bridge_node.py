"""
sim_bridge_node.py
------------------
Bridges between the dog control stack and Gazebo Fortress simulation.

In hardware mode the Teensy 4.1 firmware handles:
  /joint_angles  (Float32MultiArray) → CAN motors
  CAN feedback   → /joint_states  (Float32MultiArray [pos*12, vel*12, cur*12])
  MPU-6050       → /imu/data  (Imu)  and  /imu/euler (Vector3, degrees)

In sim mode this node replaces the Teensy:

Subscriptions:
  /joint_angles         (std_msgs/Float32MultiArray[8])   from gait_node (8DOF, no hips)
  /joint_states         (sensor_msgs/JointState)          from joint_state_broadcaster
  /imu/data             (sensor_msgs/Imu)                 from Gazebo IMU sensor
  /odom                 (nav_msgs/Odometry)                from Gazebo OdometryPublisher
                                                           (only active in autonomous mode)

Publications:
  /joint_group_position_controller/commands
                        (std_msgs/Float64MultiArray[12])  to ign_ros2_control
  /imu/euler            (geometry_msgs/Vector3)           roll, pitch in DEGREES
                                                          (matches Teensy output)
  /tf                   (odom → base_link transform)       when /odom is received;
                                                           required by Nav2 and SLAM
"""

import math
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Empty
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

# Ignition Gazebo world name (must match worlds/empty.sdf)
_WORLD_NAME = 'dog_world'


# Joint order expected by /joint_group_position_controller/commands
# Must match joint order in ros2_controllers.yaml (hip joints removed, 8DOF)
JOINT_NAMES = [
    'fr_shoulder_joint', 'fr_knee_joint',
    'fl_shoulder_joint', 'fl_knee_joint',
    'rr_shoulder_joint', 'rr_knee_joint',
    'rl_shoulder_joint', 'rl_knee_joint',
]
NUM_JOINTS = len(JOINT_NAMES)

# Map joint name → index in JOINT_NAMES (for reordering JointState feedback)
_JOINT_INDEX = {name: i for i, name in enumerate(JOINT_NAMES)}

# ── Hardware → geometric joint conversion ──────────────────────────────────
# The real robot uses a belt-drive on the knee: the motor encoder measures
#   motor_knee = geometric_shoulder + geometric_knee   (coupled)
# so to recover the geometric knee angle for the Gazebo position controller:
#   geometric_knee = motor_knee_cmd - motor_shoulder_cmd
#
# Indices within the 8-element command vector:
#   [FR_sho(0), FR_kne(1), FL_sho(2), FL_kne(3),
#    RR_sho(4), RR_kne(5), RL_sho(6), RL_kne(7)]
_SHOULDER_IDX = (0, 2, 4, 6)
_KNEE_IDX     = (1, 3, 5, 7)


def _motor_to_geometric(motor: list) -> list:
    """Convert 8 hardware motor commands to 8 geometric joint angles."""
    geo = list(motor)
    # Decouple belt-drive knee: geometric_knee = motor_knee - motor_shoulder
    for sho, kne in zip(_SHOULDER_IDX, _KNEE_IDX):
        geo[kne] = motor[kne] - motor[sho]
    return geo


def _quat_to_roll_pitch_deg(q):
    """Convert geometry_msgs/Quaternion to (roll_deg, pitch_deg)."""
    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (rotation around x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    return math.degrees(roll_rad), math.degrees(pitch_rad)


class SimBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_bridge_node')

        # Publish joint commands to ign_ros2_control position controller
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10,
        )

        # Publish euler angles to match Teensy /imu/euler output (degrees)
        self._euler_pub = self.create_publisher(
            Vector3,
            '/imu/euler',
            10,
        )

        # Last received joint command — republished at fixed rate so the
        # Gazebo controller catches it even if it activates after the command
        # was first sent (state_manager only publishes on state transitions).
        self._last_cmd: Float64MultiArray | None = None

        # Subscribe to joint angle commands from gait_node / state_manager
        self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self._joint_angles_cb,
            10,
        )

        # Subscribe to Gazebo IMU (bridged via ros_gz_bridge)
        self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_cb,
            10,
        )

        # Subscribe to sim reset trigger from keyboard_node ('r' key)
        self.create_subscription(
            Empty,
            '/sim_reset',
            self._sim_reset_cb,
            10,
        )

        # Broadcast odom→base_link TF from Gazebo ground-truth odometry.
        # This is only published when autonomous:=true (gz_odom_bridge runs).
        # SLAM Toolbox and Nav2 require this TF to function.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10,
        )

        # Republish the last command at 20 Hz so the position controller
        # receives it as soon as it becomes active.
        self.create_timer(0.05, self._republish_cmd)

        self.get_logger().info('sim_bridge_node started — bridging to Gazebo Fortress')

    # ─────────────────────────────────────────────────────────────────
    def _joint_angles_cb(self, msg: Float32MultiArray):
        """Cache and forward /joint_angles (Float32[8]) → position controller (Float64[8])."""
        data = msg.data
        if len(data) != NUM_JOINTS:
            self.get_logger().warn(
                f'Expected {NUM_JOINTS} joint angles, got {len(data)} — dropping'
            )
            return

        cmd = Float64MultiArray()
        cmd.data = _motor_to_geometric([float(v) for v in data])
        self._last_cmd = cmd
        self._cmd_pub.publish(cmd)

    def _republish_cmd(self):
        """Republish last joint command so controller picks it up once active."""
        if self._last_cmd is not None:
            self._cmd_pub.publish(self._last_cmd)

    def _sim_reset_cb(self, _msg: Empty):
        """Reset the Gazebo world (resets model poses and sim time)."""
        self.get_logger().info('Resetting Gazebo world…')
        self._last_cmd = None  # stop republishing stale commands
        subprocess.Popen([
            'ign', 'service',
            '-s', f'/world/{_WORLD_NAME}/control',
            '--reqtype',  'ignition.msgs.WorldControl',
            '--reptype',  'ignition.msgs.Boolean',
            '--timeout',  '2000',
            '--req',      'reset: {model_only: true}',
        ])

    def _odom_cb(self, msg: Odometry):
        """Broadcast odom→base_link TF from Gazebo ground-truth odometry.

        Stamps the transform with the node's current clock (sim time when
        use_sim_time:=true) rather than msg.header.stamp.  This prevents
        TF_OLD_DATA warnings caused by DDS reordering /odom messages slightly
        out of order — the node clock is always monotonically increasing.
        """
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()  # monotonic sim clock
        t.header.frame_id = msg.header.frame_id              # 'odom'
        t.child_frame_id  = msg.child_frame_id               # 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _imu_cb(self, msg: Imu):
        """Convert Imu orientation quaternion → /imu/euler (degrees, matches Teensy)."""
        roll_deg, pitch_deg = _quat_to_roll_pitch_deg(msg.orientation)

        euler = Vector3()
        euler.x = roll_deg
        euler.y = pitch_deg
        euler.z = 0.0  # yaw not used by state_manager fall-detection
        self._euler_pub.publish(euler)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
