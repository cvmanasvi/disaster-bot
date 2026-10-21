"""Earthquake disaster-site world with the existing TurtleBot3 (TURTLEBOT3_MODEL=burger_cam).

Same robot / bridges / topics as turtlebot3_house.launch.py; only the world and spawn pose differ.

  export TURTLEBOT3_MODEL=burger_cam
  ros2 launch ~/ros2_ws/disaster_site/launch/disaster_site.launch.py [gui:=false] [x_pose:=0.0 y_pose:=-1.2 yaw:=0.0]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SITE = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger_cam')
    world = os.path.join(SITE, 'world', 'earthquake_disaster.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='-1.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(SITE, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', model,
                        '-file', os.path.join(tb3, 'models', 'turtlebot3_' + model, 'model.sdf'),
                        '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p',
                        f'config_file:={os.path.join(tb3, "params", "turtlebot3_" + model + "_bridge.yaml")}']),
        Node(package='ros_gz_image', executable='image_bridge', arguments=['/camera/image_raw'], output='screen'),
    ])
