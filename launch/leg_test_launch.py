"""
leg_test_launch.py
------------------
Starts the micro-ROS agent and (optionally) the controller nodes.
The keyboard UI must be run in a separate terminal:

  Terminal 1:  ros2 launch dog leg_test_launch.py
  Terminal 2:  ros2 run dog keyboard_leg_node

Xbox controller (optional):
  ros2 launch dog leg_test_launch.py ctrl:=xbox

PS4 DualShock 4 controller:
  ros2 launch dog leg_test_launch.py ctrl:=ps4

Controller mapping (leg_joy_node):
  LB / L1 (press)   cycle primary leg  (FR → FL → RR → RL)
  RB / R1 (press)   add / cycle secondary leg
  L2 (trigger)      remove secondary leg
  Left stick X      hip angle
  Left stick Y      shoulder angle
  Right stick Y     knee angle
  Cross / A button  reset active leg(s) to neutral
  Share / BACK      reset all legs to neutral
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share   = get_package_share_directory('dog')
    xbox_config = os.path.join(pkg_share, 'config', 'xbox_controller.yaml')
    ps4_config  = os.path.join(pkg_share, 'config', 'ps4_controller.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for Teensy 4.1 (micro-ROS)'
    )

    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='keyboard',
        description='Controller type: keyboard (run separately), xbox, or ps4'
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
        output='screen',
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'xbox'"])),
    )

    joy_node_ps4 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[ps4_config],
        output='screen',
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'ps4'"])),
    )

    leg_joy_node = Node(
        package='dog',
        executable='leg_joy_node',
        name='leg_joy_node',
        parameters=[{'controller_type': ctrl}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", ctrl, "' in ['xbox', 'ps4']"])),
    )

    return LaunchDescription([
        serial_port_arg,
        ctrl_arg,
        micro_ros_agent,
        joy_node_xbox,
        joy_node_ps4,
        leg_joy_node,
    ])
