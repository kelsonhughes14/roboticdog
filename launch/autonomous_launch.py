"""
autonomous_launch.py
---------------------
Convenience alias for:  ros2 launch dog dog_launch.py autonomous:=true

Defaults to nav_test.sdf (obstacle world) when sim:=true, so you get a
useful environment for testing Nav2 path planning out of the box.
Override the world with:  ros2 launch dog autonomous_launch.py world:=<path>

Usage:
  ros2 launch dog autonomous_launch.py                          # hardware
  ros2 launch dog autonomous_launch.py sim:=true                # sim + obstacles
  ros2 launch dog autonomous_launch.py sim:=true world:=$(ros2 pkg prefix dog)/share/dog/worlds/empty.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share    = get_package_share_directory('dog')
    nav_world    = os.path.join(pkg_share, 'worlds', 'nav_test.sdf')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Teensy 4.1 USB serial port',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyACM1',
        description='Hokuyo UTM-30LX USB serial port',
    )
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', default_value='xbox',
        description='Controller type: xbox | ps4 | keyboard',
    )
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Set true for Gazebo simulation',
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=nav_world,
        description='Gazebo world SDF (sim mode only). Defaults to nav_test.sdf with obstacles.',
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        ctrl_arg,
        sim_arg,
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'dog_launch.py')
            ),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'lidar_port':  LaunchConfiguration('lidar_port'),
                'ctrl':        LaunchConfiguration('ctrl'),
                'sim':         LaunchConfiguration('sim'),
                'world':       LaunchConfiguration('world'),
                'autonomous':  'true',
            }.items(),
        ),
    ])
