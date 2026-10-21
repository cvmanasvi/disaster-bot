from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='my_first_robot',
            executable='mission_control',
            name='mission_control'
        ),

        Node(
            package='my_first_robot',
            executable='mission_monitor',
            name='mission_monitor'
        ),

        Node(
            package='my_first_robot',
            executable='battery_server',
            name='battery_server'
        )

    ])
