"""
torque_monitor_node.py
----------------------
Subscribes to /joint_states (Float32MultiArray[48] from Teensy) and republishes
motor data as a sensor_msgs/JointState on /motor_states with proper joint names,
plus temperature on /motor_temps (Float32MultiArray[12]).

/joint_states layout (from Teensy firmware):
  [0:12]   position  (rad)
  [12:24]  velocity  (rad/s)
  [24:36]  current   (A) — proportional to torque; AK45-36 peak ≈ 18 N·m
  [36:48]  temperature (°C)

The `effort` field of /motor_states carries the raw current (A).
View with:  ros2 topic echo /motor_states
            ros2 topic echo /motor_temps
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

JOINT_NAMES = [
    'FR_hip', 'FR_shoulder', 'FR_knee',
    'FL_hip', 'FL_shoulder', 'FL_knee',
    'RR_hip', 'RR_shoulder', 'RR_knee',
    'RL_hip', 'RL_shoulder', 'RL_knee',
]


class TorqueMonitorNode(Node):
    def __init__(self):
        super().__init__('torque_monitor')

        self._states_pub = self.create_publisher(JointState, '/motor_states', 10)
        self._temps_pub  = self.create_publisher(Float32MultiArray, '/motor_temps', 10)
        self._sub = self.create_subscription(
            Float32MultiArray,
            '/joint_states',
            self._on_joint_states,
            10,
        )

        self._log_counter = 0
        self.get_logger().info(
            'torque_monitor ready — publishing on /motor_states and /motor_temps'
        )

    def _on_joint_states(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 48:
            self.get_logger().warn(
                f'Expected 48 floats in /joint_states, got {len(data)} — skipping',
                throttle_duration_sec=5.0,
            )
            return

        positions  = list(data[0:12])
        velocities = list(data[12:24])
        currents   = list(data[24:36])  # Amperes ∝ torque
        temps      = list(data[36:48])  # °C

        stamp = self.get_clock().now().to_msg()

        # /motor_states — full named JointState (effort = current in A)
        out = JointState()
        out.header.stamp = stamp
        out.name     = JOINT_NAMES
        out.position = positions
        out.velocity = velocities
        out.effort   = currents

        self._states_pub.publish(out)

        # /motor_temps — 12 temperatures in °C
        temp_msg = Float32MultiArray()
        temp_msg.data = temps
        self._temps_pub.publish(temp_msg)

        # Log a human-readable summary once per second (~every 50 messages at 50 Hz)
        self._log_counter += 1
        if self._log_counter >= 50:
            self._log_counter = 0
            lines = ['Motor current (A) | temp (°C):']
            for i, name in enumerate(JOINT_NAMES):
                lines.append(
                    f'  {name:<16s}  {currents[i]:+.3f} A   {temps[i]:.1f} °C'
                )
            self.get_logger().info('\n'.join(lines))


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
