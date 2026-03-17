from setuptools import setup
import os
from glob import glob

package_name = 'spotmicro'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpotMicro User',
    maintainer_email='user@example.com',
    description='Full control package for Spot Micro quadruped robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node        = spotmicro.servo_node:main',
            'imu_node          = spotmicro.imu_node:main',
            'controller_node   = spotmicro.controller_node:main',
            'gait_node         = spotmicro.gait_node:main',
            'state_manager     = spotmicro.state_manager:main',
            'calibration_node  = spotmicro.calibration_node:main',
            'leg_test_node     = spotmicro.leg_test_node:main',
            'leg_joy_node      = spotmicro.leg_joy_node:main',
            'keyboard_node     = spotmicro.keyboard_node:main',
        ],
    },
)
