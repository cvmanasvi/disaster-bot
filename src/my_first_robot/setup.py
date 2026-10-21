from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'my_first_robot'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            'share/' + package_name + '/launch',
            glob('launch/*.py')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Manasvi',
    maintainer_email='184147642+cvmanasvi@users.noreply.github.com',

    description='My first ROS2 package',
    license='All rights reserved',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'mission_control = my_first_robot.mission_control:main',
            'mission_monitor = my_first_robot.mission_monitor:main',

            'battery_server = my_first_robot.battery_server:main',
            'battery_client = my_first_robot.battery_client:main',

            'obstacle_detector = my_first_robot.obstacle_detector:main',
            'safety_controller = my_first_robot.safety_controller:main',

            'autonomous_search = my_first_robot.autonomous_search:main',

            'camera_viewer = my_first_robot.camera_viewer:main',
            'yolo_detector = my_first_robot.yolo_detector:main',

            'victim_distance_estimator = my_first_robot.victim_distance_estimator:main',
            'victim_rescue = my_first_robot.victim_rescue:main',

            'keyboard_controller = my_first_robot.keyboard_controller:main',

            'disaster_bot_monitor = my_first_robot.disaster_bot_monitor:main',
        ],
    },
)


