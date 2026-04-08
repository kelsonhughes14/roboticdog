"""
torque_monitor_node.py
----------------------
Subscribes to /joint_states (Float32MultiArray[32] from Teensy) and republishes
motor data as a sensor_msgs/JointState on /motor_states with proper joint names,
plus temperature on /motor_temps (Float32MultiArray[8]).

Also subscribes to /motor_fault (std_msgs/UInt8 bitmask) and forwards any
firmware-detected faults to /estop (std_msgs/Bool) to trigger a safe shutdown.

/joint_states layout (from Teensy firmware, 8DOF):
  [0:8]    position   (rad)
  [8:16]   velocity   (rad/s)
  [16:24]  torque     (N·m)  — decoded by firmware with MIT_T_MAX = 18 N·m
  [24:32]  temperature (°C)

Phase current (A) is derived from torque using MOTOR_KT_EFF (N·m/A).
The `effort` field of /motor_states carries torque (N·m).

CSV log (one row per motor per sample):
  timestamp_s, motor, position_rad, velocity_rad_s, torque_nm, current_a, temp_c

Parameters:
  csv_path  (string) — path for the CSV log file (default: ~/motor_log.csv)

View live data with:
  ros2 topic echo /motor_states
  ros2 topic echo /motor_temps
"""

import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool, UInt8

from dog.robot_config import MOTOR_KT_EFF

N_MOTORS = 8

JOINT_NAMES = [
    'FR_shoulder', 'FR_knee',
    'FL_shoulder', 'FL_knee',
    'RR_shoulder', 'RR_knee',
    'RL_shoulder', 'RL_knee',
]

_CSV_HEADER = ['timestamp_s', 'motor', 'position_rad', 'velocity_rad_s', 'torque_nm', 'current_a', 'temp_c']


class TorqueMonitorNode(Node):
    def __init__(self):
        super().__init__('torque_monitor')

        _logs_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'Logs',
        )
        os.makedirs(_logs_dir, exist_ok=True)
        _session_name = datetime.now().strftime('session_%Y-%m-%d_%H-%M-%S.csv')
        _default_path = os.path.join(_logs_dir, _session_name)

        self.declare_parameter('csv_path', _default_path)
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        self._states_pub = self.create_publisher(JointState,        '/motor_states', 10)
        self._temps_pub  = self.create_publisher(Float32MultiArray, '/motor_temps',  10)
        self._estop_pub  = self.create_publisher(Bool,              '/estop',        10)

        self.create_subscription(
            Float32MultiArray, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            UInt8, '/motor_fault', self._on_motor_fault, 10)

        self._log_counter = 0

        # Each launch creates a new file — no append, always write fresh header
        self._csv_file   = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(_CSV_HEADER)

        self.get_logger().info(
            f'torque_monitor ready — logging to {csv_path}'
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
        torques    = list(data[16:24])
        temps      = list(data[24:32])
        currents   = [t / MOTOR_KT_EFF for t in torques]   # N·m → A

        stamp     = self.get_clock().now().to_msg()
        timestamp = stamp.sec + stamp.nanosec * 1e-9

        # ── Publish /motor_states ──────────────────────────────────────────────
        out = JointState()
        out.header.stamp = stamp
        out.name     = JOINT_NAMES
        out.position = positions
        out.velocity = velocities
        out.effort   = torques        # N·m
        self._states_pub.publish(out)

        # ── Publish /motor_temps ───────────────────────────────────────────────
        temp_msg = Float32MultiArray()
        temp_msg.data = temps
        self._temps_pub.publish(temp_msg)

        # ── CSV log — every sample ─────────────────────────────────────────────
        for i, name in enumerate(JOINT_NAMES):
            self._csv_writer.writerow([
                f'{timestamp:.6f}',
                name,
                f'{positions[i]:.6f}',
                f'{velocities[i]:.6f}',
                f'{torques[i]:.6f}',
                f'{currents[i]:.4f}',
                f'{temps[i]:.2f}',
            ])
        self._csv_file.flush()

        # ── Console log — every ~1 s (50 samples at typical callback rate) ─────
        self._log_counter += 1
        if self._log_counter >= 50:
            self._log_counter = 0
            lines = ['Motor state — position (rad) | velocity (rad/s) | torque (N·m) | current (A) | temp (°C):']
            for i, name in enumerate(JOINT_NAMES):
                lines.append(
                    f'  {name:<16s}'
                    f'  pos={positions[i]:+.3f}'
                    f'  vel={velocities[i]:+.3f}'
                    f'  trq={torques[i]:+.3f} N·m'
                    f'  cur={currents[i]:+.3f} A'
                    f'  {temps[i]:.1f}°C'
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

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


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
