"""
calibration_launch.py
---------------------
Starts the micro-ROS agent so the Teensy 4.1 is reachable as a ROS2 node.

The Teensy subscribes to:
  /joint_angles    — motor position commands (radians)
  /can_calibrate   — motor ID to send "set zero position" CAN frame
  /can_enable      — enter / exit motor mode for all motors

Usage:
  ros2 launch dog calibration_launch.py
  ros2 launch dog calibration_launch.py serial_port:=/dev/ttyACM1

Then in another terminal, run the interactive calibration tool:
  ros2 run dog calibration_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for Teensy 4.1 (micro-ROS)'
    )
    serial_port = LaunchConfiguration('serial_port')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port,
        ],
        output='screen',
        name='micro_ros_agent',
    )

    return LaunchDescription([
        serial_port_arg,
        micro_ros_agent,
    ])
