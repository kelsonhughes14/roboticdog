"""
leg_test_launch.py
------------------
Control a single leg with the Xbox controller while all others hold neutral.

Controls:
  RB (press)       next leg   (FR → FL → RR → RL → FR)
  LB (press)       previous leg
  Left stick X     hip angle
  Left stick Y     shoulder angle
  Right stick Y    knee angle
  A button         reset selected leg to neutral
  BACK button      reset all legs to neutral

Usage:
  ros2 launch spotmicro leg_test_launch.py
  ros2 launch spotmicro leg_test_launch.py serial_port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for the Teensy 4.1 (micro-ROS)'
    )
    serial_port = LaunchConfiguration('serial_port')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port, '-b', '1000000',
        ],
        output='screen',
        name='micro_ros_agent',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    leg_joy_node = Node(
        package='spotmicro',
        executable='leg_joy_node',
        name='leg_joy_node',
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        micro_ros_agent,
        joy_node,
        leg_joy_node,
    ])
