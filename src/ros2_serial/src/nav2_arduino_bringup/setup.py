from setuptools import setup
import os
from glob import glob

package_name = 'nav2_arduino_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi025',
    maintainer_email='your@email.com',
    description='Arduino + Nav2 integration package for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = nav2_arduino_bringup.serial_bridge_node:main',
            'arduino_bridge = nav2_arduino_bringup.arduino_bridge:main',
            'arduino_odometry_node = nav2_arduino_bringup.arduino_odometry_node:main',
            'goal_stop_controller = nav2_arduino_bringup.goal_stop_controller:main',
            'fake_odom_publisher = nav2_arduino_bringup.fake_odom_publisher:main',
        ],
    },
)

