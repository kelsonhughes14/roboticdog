"""
dog_launch.py
-------------
Main launch file for the Dog quadruped robot.

Hardware mode (default, sim:=false):
  - micro_ros_agent     (bridges Teensy USB serial <-> ROS2)
  - joy_node            (Xbox/PS4 controller — skipped when ctrl:=keyboard)
  - controller_node     (watchdog / republish)
  - state_manager       (robot state machine + motor enable/disable)
  - gait_node           (gait generator + IK, publishes /joint_angles in radians)
  - torque_monitor_node (logs per-motor current from /joint_states)

Simulation mode (sim:=true):
  - ign gazebo        (Gazebo Fortress with flat-ground world)
  - robot_state_publisher
  - ros_gz_bridge     (Gazebo IMU -> /imu/data)
  - sim_bridge_node   (/joint_angles -> Gazebo controllers, /imu/data -> /imu/euler)
  - joy_node / controller_node / state_manager / gait_node  (unchanged)

Autonomous mode (autonomous:=true, works with both sim and hardware):
  - autonomous_bridge_node  (/cmd_vel -> /gait_command when in AUTONOMOUS state)
  - slam_toolbox            (online async mapping, map->odom TF)
  - nav2 stack              (planner + controller + costmaps, publishes /cmd_vel)
  - rviz2                   (map, scan, paths, Nav2 goal panel)
  - requires odom->base_link TF on /odom (from your odometry/localization source)
  Hardware only:
    - robot_state_publisher  (URDF TF for sensor frames)
    - urg_node2 / hokuyo     (/scan from physical Hokuyo UTM-30LX)
  Sim only:
    - ros_gz_bridge for /scan and /odom (from Gazebo lidar + odometry plugins)

Usage:
  ros2 launch dog dog_launch.py                                    # hardware, Xbox
  ros2 launch dog dog_launch.py ctrl:=ps4                         # hardware, PS4
  ros2 launch dog dog_launch.py ctrl:=keyboard                    # hardware, keyboard
  ros2 launch dog dog_launch.py sim:=true                         # Gazebo only
  ros2 launch dog dog_launch.py autonomous:=true                  # hardware + Nav2
  ros2 launch dog dog_launch.py sim:=true autonomous:=true        # Gazebo + Nav2
  ros2 launch dog dog_launch.py autonomous:=true lidar_port:=/dev/ttyACM1
  ros2 launch dog dog_launch.py autonomous:=true fallback_odom_tf:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    pkg_share    = get_package_share_directory('dog')
    robot_params = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    xbox_config  = os.path.join(pkg_share, 'config', 'xbox_controller.yaml')
    ps4_config   = os.path.join(pkg_share, 'config', 'ps4_controller.yaml')
    nav2_params  = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'config', 'rviz_nav.rviz')
    xacro_file   = os.path.join(pkg_share, 'urdf',   'dog.urdf.xacro')
    world_file   = os.path.join(pkg_share, 'worlds',  'empty.sdf')

    # ── Launch arguments ──────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='USB serial port for Teensy 4.1 (hardware mode only)',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for Hokuyo UTM-30LX (hardware + autonomous only)',
    )
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='xbox',
        description='Input device: xbox | ps4 | keyboard',
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Set true to run in Gazebo Fortress simulation',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to Gazebo world SDF file (sim mode only)',
    )
    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='false',
        description='Set true to launch Nav2 + SLAM Toolbox + sensor nodes',
    )
    fallback_odom_tf_arg = DeclareLaunchArgument(
        'fallback_odom_tf',
        default_value='true',
        description=(
            'Hardware autonomous only: publish static odom->base_link when no odometry source exists. '
            'Set false if an external odometry/localization node already publishes odom->base_link.'
        ),
    )

    serial_port = LaunchConfiguration('serial_port')
    lidar_port  = LaunchConfiguration('lidar_port')
    ctrl        = LaunchConfiguration('ctrl')
    sim         = LaunchConfiguration('sim')
    autonomous  = LaunchConfiguration('autonomous')
    fallback_odom_tf = LaunchConfiguration('fallback_odom_tf')
    world       = LaunchConfiguration('world')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    # ── Helper expressions ─────────────────────────────────────────
    # PythonExpression conditions: compare string values of LaunchConfigurations.
    is_sim      = IfCondition(sim)
    is_hw       = UnlessCondition(sim)
    is_auto     = IfCondition(autonomous)
    is_hw_auto  = IfCondition(PythonExpression(
        ["'", sim, "' == 'false' and '", autonomous, "' == 'true'"]
    ))
    is_sim_auto = IfCondition(PythonExpression(
        ["'", sim, "' == 'true'  and '", autonomous, "' == 'true'"]
    ))
    use_fallback_odom_tf = IfCondition(PythonExpression(
        ["'", sim, "' == 'false' and '", autonomous, "' == 'true' and '", fallback_odom_tf, "' == 'true'"]
    ))

    # ═══════════════════════════════════════════════════════════════
    # HARDWARE-ONLY  (sim:=false)
    # ═══════════════════════════════════════════════════════════════

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port,
        ],
        output='screen',
        name='micro_ros_agent',
        condition=is_hw,
    )

    # ═══════════════════════════════════════════════════════════════
    # SIMULATION-ONLY  (sim:=true)
    # ═══════════════════════════════════════════════════════════════

    sim_group = GroupAction(
        condition=is_sim,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description_content,
                             'use_sim_time': True}],
                output='screen',
            ),

            ExecuteProcess(
                cmd=['ign', 'gazebo', '-r', world],
                additional_env={
                    'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
                        '/opt/ros/humble/lib:'
                        + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
                    'GZ_SIM_SYSTEM_PLUGIN_PATH':
                        '/opt/ros/humble/lib:'
                        + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                },
                output='screen',
            ),

            # Bridge Gazebo IMU and clock → ROS2
            # /clock is required for use_sim_time nodes (SLAM, Nav2) to advance.
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_imu_bridge',
                arguments=[
                    '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                ],
                output='screen',
            ),

            # /joint_angles → Gazebo controllers; /imu/data → /imu/euler;
            # /odom → odom→base_link TF (autonomous mode).
            # use_sim_time=True so get_clock().now() matches Gazebo sim time,
            # ensuring TF stamps are monotonically increasing even if /odom
            # messages arrive slightly reordered through the DDS bridge.
            Node(
                package='dog',
                executable='sim_bridge_node',
                name='sim_bridge_node',
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),

            # Spawn robot after Gazebo has loaded the world
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        name='spawn_dog',
                        arguments=[
                            '-name', 'dog',
                            '-topic', 'robot_description',
                            '-z', '0.65',
                        ],
                        output='screen',
                    ),
                ],
            ),

            # Load ros2_control controllers with retry loop
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'bash', '-c',
                            'until ros2 control load_controller --set-state active '
                            'joint_state_broadcaster; '
                            'do echo "[dog_launch] controller_manager not ready, retrying..."; '
                            'sleep 1; done && '
                            'sleep 1 && '
                            'until ros2 control load_controller --set-state active '
                            'joint_group_position_controller; '
                            'do echo "[dog_launch] position controller not ready, retrying..."; '
                            'sleep 1; done',
                        ],
                        output='screen',
                    ),
                ],
            ),
        ],
    )

    # ═══════════════════════════════════════════════════════════════
    # SIMULATION + AUTONOMOUS  (sim:=true autonomous:=true)
    # Bridges Gazebo lidar and ground-truth odometry into ROS2 topics
    # that SLAM Toolbox and Nav2 consume (/scan, /odom).
    # ═══════════════════════════════════════════════════════════════

    sim_auto_group = GroupAction(
        condition=is_sim_auto,
        actions=[
            # Bridge Gazebo lidar sensor → /scan (LaserScan)
            # The sensor plugin in the URDF publishes to /scan in Ignition topic space.
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_scan_bridge',
                arguments=[
                    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                ],
                output='screen',
            ),
            # Bridge Gazebo ground-truth odometry → /odom (Odometry)
            # The OdometryPublisher plugin publishes to /model/dog/odometry.
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_odom_bridge',
                arguments=[
                    '/model/dog/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                ],
                remappings=[('/model/dog/odometry', '/odom')],
                output='screen',
            ),
        ],
    )

    # ═══════════════════════════════════════════════════════════════
    # HARDWARE + AUTONOMOUS  (sim:=false autonomous:=true)
    # Launches physical sensors and robot_state_publisher for TF.
    # ═══════════════════════════════════════════════════════════════

    # ── Hokuyo UTM-30LX lifecycle node (defined here so event handlers can reference it) ──
    hokuyo_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='hokuyo',
        namespace='',
        parameters=[{
            'serial_port':       lidar_port,
            'frame_id':          'lidar_link',
            'angle_min':         -2.356,
            'angle_max':          2.356,
            'publish_intensity':  False,
        }],
        output='screen',
    )

    # Unconfigured → Inactive on process start
    hokuyo_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(hokuyo_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
    )

    # Inactive → Active once configure completes
    hokuyo_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(hokuyo_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    hw_auto_group = GroupAction(
        condition=is_hw_auto,
        actions=[
            # robot_state_publisher broadcasts URDF joint TFs (lidar_link, imu_link)
            # so Nav2 and SLAM can locate sensor frames relative to base_link.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description_content}],
                output='screen',
            ),

            # Hokuyo UTM-30LX — lifecycle node, auto-activates on startup
            hokuyo_node,
            hokuyo_configure,
            hokuyo_activate,

            # Fallback for hardware setups without a live odometry source.
            # This prevents Nav2/SLAM message_filters from dropping /scan
            # due to missing odom->base_link TF at startup.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fallback_odom_tf_pub',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                output='screen',
                condition=use_fallback_odom_tf,
            ),
        ],
    )

    # ═══════════════════════════════════════════════════════════════
    # AUTONOMOUS (common to both sim and hardware, autonomous:=true)
    # SLAM Toolbox, Nav2, autonomous bridge node, RViz.
    # Nav2 is delayed 5 s to give SLAM time to emit its first map→odom TF.
    # ═══════════════════════════════════════════════════════════════

    auto_group = GroupAction(
        condition=is_auto,
        actions=[
            # SLAM Toolbox (online async mapping) — builds /map, broadcasts map→odom TF
            # use_sim_time mirrors the sim argument so SLAM timestamps match Gazebo
            # sensor data in simulation and wall-clock on hardware.
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_params, {
                    'use_sim_time': PythonExpression(["'", sim, "' == 'true'"])
                }],
                output='screen',
            ),

            # Nav2 navigation stack (planner + controller + costmaps → /cmd_vel)
            # 15 s delay: Gazebo needs ~3 s to spawn, ~5 s for controllers to load,
            # then SLAM needs a few scans before it emits its first map→odom TF.
            # Starting Nav2 before that TF exists causes "extrapolation into past" errors.
            TimerAction(
                period=15.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py',
                            )
                        ),
                        launch_arguments={
                            'params_file':  nav2_params,
                            'use_sim_time': sim,
                        }.items(),
                    ),
                ],
            ),

            # Routes /cmd_vel → /gait_command while robot is in AUTONOMOUS state
            Node(
                package='dog',
                executable='autonomous_bridge_node',
                name='autonomous_bridge',
                output='screen',
            ),

            # RViz with Nav2 goal panel — use "Nav2 Goal" tool to set goal poses
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ],
    )

    # ═══════════════════════════════════════════════════════════════
    # COMMON  (hardware and sim, with or without autonomous)
    # ═══════════════════════════════════════════════════════════════

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

    gait_node = Node(
        package='dog',
        executable='gait_node',
        name='gait_node',
        parameters=[robot_params],
        output='screen',
    )

    torque_monitor_node = Node(
        package='dog',
        executable='torque_monitor_node',
        name='torque_monitor',
        output='screen',
    )

    imu_leveling_node = Node(
        package='dog',
        executable='imu_leveling_node',
        name='imu_leveling',
        parameters=[{
            'alpha':        0.10,   # EMA smoothing — lower = slower/smoother
            'max_angle':   10.0,   # max correction in degrees
            'invert_roll':  False,  # flip if roll reads backwards
            'invert_pitch': False,  # flip if pitch reads backwards
        }],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        ctrl_arg,
        sim_arg,
        world_arg,
        autonomous_arg,
        fallback_odom_tf_arg,
        micro_ros_agent,
        sim_group,
        sim_auto_group,
        hw_auto_group,
        auto_group,
        joy_node_xbox,
        joy_node_ps4,
        controller_node,
        state_manager,
        gait_node,
        imu_leveling_node,
        torque_monitor_node,
    ])
