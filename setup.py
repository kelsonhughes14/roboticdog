from setuptools import setup
import os
from glob import glob

package_name = 'dog'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'meshes'),
            glob('urdf/meshes/*.dae')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dog Robot User',
    maintainer_email='user@example.com',
    description='Large-scale quadruped robot with CubeMars AK45-36 CAN motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node          = dog.imu_node:main',
            'gps_node          = dog.gps_node:main',
            'controller_node   = dog.controller_node:main',
            'gait_node         = dog.gait_node:main',
            'state_manager     = dog.state_manager:main',
            'calibration_node  = dog.calibration_node:main',
            'leg_test_node     = dog.leg_test_node:main',
            'leg_joy_node      = dog.leg_joy_node:main',
            'keyboard_node     = dog.keyboard_node:main',
            'keyboard_leg_node = dog.keyboard_leg_node:main',
            'sim_bridge_node        = dog.sim_bridge_node:main',
            'autonomous_bridge_node = dog.autonomous_bridge_node:main',
            'torque_monitor_node    = dog.torque_monitor_node:main',
        ],
    },
)
