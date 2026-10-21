"""Disaster Bot: complete validated stack in the disaster-site world, one command.

    ros2 launch my_first_robot disaster_bot_full.launch.py [gui:=false] [x_pose:=0.0]
        [y_pose:=-1.2] [yaw:=0.0] [explore_max_steps:=30] [record_dir:=/path/to/scan_record]

Starts (packaging only; no algorithm or parameter changes):
  * the validated disaster-site simulation  (~/ros2_ws/disaster_site/launch/disaster_site.launch.py:
    Gazebo, robot_state_publisher, burger_cam spawn, ros_gz_bridge, camera image bridge)
  * safety_controller          /cmd_vel_raw -> /cmd_vel   (the ONLY /cmd_vel publisher)
  * yolo_detector              run with ~/ros2_ws/yolo_env/bin/python (ultralytics lives there),
                               cwd ~/ros2_ws so 'yolo11n.pt' resolves exactly as validated
  * victim_distance_estimator  /victim_detections + /scan -> /victim_distance
  * autonomous_search          -> /cmd_vel_raw, with the validated mission configuration
                               victim_approach_enabled, victim_prioritization_enabled,
                               explore_max_steps 30 (started after the others, as in validation)

Deliberately NOT started: obstacle_detector (legacy, publishes /cmd_vel directly), victim_rescue,
mission_control, mission_monitor, battery_server / battery_client, keyboard_controller.

All nodes use sim time. record_dir (default '' = off, the node default) only sets
autonomous_search's scan_record_dir logging. yolo_detector opens an OpenCV window (needs DISPLAY).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# autonomous_search starts this long after the rest (wall seconds), mirroring the validated
# start order: simulation, SafetyController and perception first. Its STARTUP state also waits
# for fresh /scan and /odom, so the exact value is not critical.
SEARCH_START_DELAY_S = 10.0


def _workspace():
    """~/ros2_ws, from <ws>/install/my_first_robot/share/my_first_robot."""
    share = get_package_share_directory('my_first_robot')
    return os.path.abspath(os.path.join(share, '..', '..', '..', '..'))


def generate_launch_description():
    ws = _workspace()
    site_launch = os.path.join(ws, 'disaster_site', 'launch', 'disaster_site.launch.py')
    yolo_python = os.path.join(ws, 'yolo_env', 'bin', 'python')
    yolo_script = os.path.join('src', 'my_first_robot', 'my_first_robot', 'yolo_detector.py')
    for path in (site_launch, yolo_python, os.path.join(ws, yolo_script),
                 os.path.join(ws, 'yolo11n.pt')):
        if not os.path.exists(path):
            raise RuntimeError(f'disaster_bot_full.launch.py: required file not found: {path}')

    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI client'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn x (world)'),
        DeclareLaunchArgument('y_pose', default_value='-1.2', description='Spawn y (world)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('explore_max_steps', default_value='30',
                              description='autonomous_search exploration move budget'),
        DeclareLaunchArgument('record_dir', default_value='',
                              description="autonomous_search scan_record_dir ('' = no records)"),

        # The validated robot model (the disaster-site launch reads it from the environment).
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(site_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'yaw': LaunchConfiguration('yaw'),
            }.items()),

        Node(package='my_first_robot', executable='safety_controller',
             parameters=[sim_time], output='screen'),

        ExecuteProcess(
            name='yolo_detector',
            cmd=[yolo_python, yolo_script, '--ros-args', '-p', 'use_sim_time:=true'],
            cwd=ws, output='screen', sigterm_timeout='5', sigkill_timeout='5'),

        Node(package='my_first_robot', executable='victim_distance_estimator',
             parameters=[sim_time], output='screen'),

        TimerAction(period=SEARCH_START_DELAY_S, actions=[
            Node(package='my_first_robot', executable='autonomous_search', output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'victim_approach_enabled': True,
                     'victim_prioritization_enabled': True,
                     'explore_max_steps': ParameterValue(
                         LaunchConfiguration('explore_max_steps'), value_type=int),
                     'scan_record_dir': ParameterValue(
                         LaunchConfiguration('record_dir'), value_type=str),
                 }]),
        ]),
    ])
