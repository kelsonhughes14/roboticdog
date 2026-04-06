"""
torque_monitor_node.py
----------------------
Subscribes to /joint_states (Float32MultiArray[32] from Teensy) and republishes
motor data as a sensor_msgs/JointState on /motor_states with proper joint names,
plus temperature on /motor_temps (Float32MultiArray[8]).

Also subscribes to /motor_fault (std_msgs/UInt8 bitmask) and forwards any
firmware-detected faults to /estop (std_msgs/Bool) to trigger a safe shutdown.

/joint_states layout (from Teensy firmware, 8DOF):
  [0:8]    position  (rad)
  [8:16]   velocity  (rad/s)
  [16:24]  current   (A) — proportional to torque; AK45-36 peak ≈ 18 N·m
  [24:32]  temperature (°C)

The `effort` field of /motor_states carries the raw current (A).
View with:  ros2 topic echo /motor_states
            ros2 topic echo /motor_temps
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool, UInt8

N_MOTORS = 8

JOINT_NAMES = [
    'FR_shoulder', 'FR_knee',
    'FL_shoulder', 'FL_knee',
    'RR_shoulder', 'RR_knee',
    'RL_shoulder', 'RL_knee',
]


class TorqueMonitorNode(Node):
    def __init__(self):
        super().__init__('torque_monitor')

        self._states_pub = self.create_publisher(JointState,         '/motor_states', 10)
        self._temps_pub  = self.create_publisher(Float32MultiArray,  '/motor_temps',  10)
        self._estop_pub  = self.create_publisher(Bool,               '/estop',        10)

        self.create_subscription(
            Float32MultiArray, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            UInt8, '/motor_fault', self._on_motor_fault, 10)

        self._log_counter = 0
        self.get_logger().info(
            'torque_monitor ready — publishing on /motor_states and /motor_temps'
        )

    def _on_joint_states(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 32:
            self.get_logger().warn(
                f'Expected 32 floats in /joint_states, got {len(data)} — skipping',
                throttle_duration_sec=5.0,
            )
            return

        positions  = list(data[0:8])
        velocities = list(data[8:16])
        currents   = list(data[16:24])
        temps      = list(data[24:32])

        stamp = self.get_clock().now().to_msg()

        out = JointState()
        out.header.stamp = stamp
        out.name     = JOINT_NAMES
        out.position = positions
        out.velocity = velocities
        out.effort   = currents
        self._states_pub.publish(out)

        temp_msg = Float32MultiArray()
        temp_msg.data = temps
        self._temps_pub.publish(temp_msg)

        self._log_counter += 1
        if self._log_counter >= 50:
            self._log_counter = 0
            lines = ['Motor current (A) | temp (°C):']
            for i, name in enumerate(JOINT_NAMES):
                lines.append(
                    f'  {name:<16s}  {currents[i]:+.3f} A   {temps[i]:.1f} °C'
                )
            self.get_logger().info('\n'.join(lines))

    def _on_motor_fault(self, msg: UInt8):
        bitmask = msg.data
        if bitmask == 0:
            return
        faulted = [JOINT_NAMES[i] for i in range(N_MOTORS) if bitmask & (1 << i)]
        self.get_logger().error(
            f'MOTOR FAULT (0x{bitmask:02X}): {", ".join(faulted)} — triggering E-STOP'
        )
        estop = Bool()
        estop.data = True
        self._estop_pub.publish(estop)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
