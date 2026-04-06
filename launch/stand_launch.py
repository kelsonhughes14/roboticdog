"""
stand_launch.py
---------------
Minimal hardware launch for static standing and motor verification.

Brings up only what is needed to stand the robot and monitor motor health:
  - micro_ros_agent     (bridges Teensy USB serial <-> ROS2)
  - joy_node            (Xbox/PS4 controller)
  - controller_node     (watchdog / republish)
  - state_manager       (state machine — handles sit/stand, E-STOP, righting)
  - torque_monitor_node (motor current, temperature, fault forwarding to /estop)

Gait generation and autonomous navigation are NOT started.
The robot can only sit, stand, and self-right — no walking.

Usage:
  ros2 launch dog stand_launch.py
  ros2 launch dog stand_launch.py ctrl:=ps4
  ros2 launch dog stand_launch.py serial_port:=/dev/ttyACM1
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share   = get_package_share_directory('dog')
    robot_params = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    xbox_config  = os.path.join(pkg_share, 'config', 'xbox_controller.yaml')
    ps4_config   = os.path.join(pkg_share, 'config', 'ps4_controller.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for Teensy 4.1',
    )
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='xbox',
        description='Input device: xbox | ps4',
    )

    serial_port = LaunchConfiguration('serial_port')
    ctrl        = LaunchConfiguration('ctrl')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port,
        ],
        output='screen',
        name='micro_ros_agent',
    )

    joy_node_xbox = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[xbox_config],
        remappings=[('/joy', '/joy_raw')],
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'xbox'"])),
    )

    joy_node_ps4 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[ps4_config],
        remappings=[('/joy', '/joy_raw')],
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'ps4'"])),
    )

    controller_node = Node(
        package='dog',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )

    state_manager = Node(
        package='dog',
        executable='state_manager',
        name='state_manager',
        parameters=[robot_params, {'controller_type': ctrl}],
        output='screen',
    )

    torque_monitor_node = Node(
        package='dog',
        executable='torque_monitor_node',
        name='torque_monitor',
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        ctrl_arg,
        micro_ros_agent,
        joy_node_xbox,
        joy_node_ps4,
        controller_node,
        state_manager,
        torque_monitor_node,
    ])
