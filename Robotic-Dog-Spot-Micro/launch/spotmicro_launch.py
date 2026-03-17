"""
spotmicro_launch.py
-------------------
Main launch file for Spot Micro.

Starts all nodes:
  - micro_ros_agent   (bridges Teensy USB serial ↔ ROS2)
  - joy_node          (Xbox controller input — USB or Bluetooth)
  - controller_node   (watchdog / remapper)
  - state_manager     (robot state machine)
  - gait_node         (gait generator + IK)

The Teensy 4.1 handles servos (PCA9685) and IMU (MPU-6050) directly
and exposes them as ROS2 topics via micro-ROS over USB serial.
The Pi's imu_node and servo_node are no longer needed.

Usage:
  ros2 launch spotmicro spotmicro_launch.py
  ros2 launch spotmicro spotmicro_launch.py serial_port:=/dev/ttyACM1

The controller is found by name ("Xbox Wireless Controller") so it works
for both USB and Bluetooth connections without any extra arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('spotmicro')
    robot_params = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    xbox_config  = os.path.join(pkg_share, 'config', 'xbox_controller.yaml')

    # ── Launch arguments ─────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for the Teensy 4.1 (micro-ROS)'
    )
    serial_port = LaunchConfiguration('serial_port')

    # ── micro-ROS agent — bridges Teensy ↔ ROS2 ─────────────────────
    # Runs the micro-ROS agent that connects to the Teensy over USB serial.
    # The Teensy publishes /imu/data, /imu/euler and subscribes to /servo_angles.
    # Install: sudo apt install ros-$ROS_DISTRO-micro-ros-agent
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port, '-b', '1000000',
        ],
        output='screen',
        name='micro_ros_agent',
    )

    # ── joy_node — reads Xbox controller (USB or Bluetooth) ─────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[xbox_config],
        remappings=[('/joy', '/joy_raw')],  # raw → controller_node filters it
    )

    # ── controller_node — watchdog + republish ───────────────────────
    controller_node = Node(
        package='spotmicro',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )

    # ── state_manager — robot state machine ─────────────────────────
    state_manager = Node(
        package='spotmicro',
        executable='state_manager',
        name='state_manager',
        parameters=[robot_params],
        output='screen',
    )

    # ── gait_node — gait + IK ────────────────────────────────────────
    gait_node = Node(
        package='spotmicro',
        executable='gait_node',
        name='gait_node',
        parameters=[robot_params],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        micro_ros_agent,
        joy_node,
        controller_node,
        state_manager,
        gait_node,
    ])
