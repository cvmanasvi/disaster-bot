"""
Disaster Bot full stack PLUS the optional victim posture node.

    ros2 launch my_first_robot disaster_bot_posture.launch.py [posture_enabled:=true]
        [posture_model:=/path/to/yolo11m-pose.pt] [posture_max_rate_hz:=1.0]
        [localizer_enabled:=true] [localizer_rvc:=false] [nav_guard:=false]
        [nav_guard_close_pair:=false]
        [gui:=false] [x_pose:=0.0] [y_pose:=-1.2] [yaw:=0.0] [explore_max_steps:=30]
        [record_dir:=...]

This file does not change the validated stack. It includes
disaster_bot_full.launch.py unchanged, passing every argument through, and
adds victim_posture_estimator (YOLO11m-pose) when posture_enabled is true.
It also adds the advisory per-person victim_localizer (system Python) when
localizer_enabled is true: it localizes the fused /victim_posture candidates
(or the YOLO11n boxes if the pose stream is off or unhealthy) with /scan and
publishes /victim_localizations. Nothing in the mission subscribes to it.
localizer_rvc:=true enables the localizer's RVC withhold checks (default
false = v3 behaviour); they only withhold readings, never change distances.
nav_guard:=true (default false) protects the navigation-facing path: it remaps
the frozen victim_distance_estimator's input to /victim_detections_nav (launch_ros
SetRemap; ExecuteProcess children are not remapped) and starts victim_detection_guard,
which forwards only boxes whose single-box LiDAR association passes the RVC checks
(fresh scan, image no older than 1 s). Without the guard running the estimator
gets no detections, so it can never publish an unchecked distance.
nav_guard_close_pair:=true (default false; needs nav_guard:=true) also withholds a
box whose single-box LiDAR cluster differs from its multi-box exclusive cluster
(close pairs). It removes mixed close-pair readings but loses 4-21 % of correct
readings in the recorded evaluations (2026-10-17). The legacy
obstacle_detector.py (Twist on /cmd_vel) must never run with this stack.
With posture_enabled:=false localizer_enabled:=false it starts exactly the
validated stack. Launching disaster_bot_full.launch.py directly also keeps the
validated pipeline.

The posture node publishes only /victim_posture. No node of the validated
mission subscribes to it, so it cannot change robot motion. It shares the CPU
with the validated detector, though: 1.4-2.6 s per frame under Gazebo was
measured in the 2026-10-12 tests. The default rate limit is 1 Hz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

# The YOLO11m-pose weights evaluated on 2026-10-12 (sha256 29b17eaf...). They
# are not in the workspace; pass posture_model:= to use another copy.
DEFAULT_POSTURE_MODEL = os.path.expanduser(
    '~/disaster_bot_pose_eval_20261012/model_comparison/models/yolo11m-pose.pt')

FULL_ARGS = ('gui', 'x_pose', 'y_pose', 'yaw', 'explore_max_steps', 'record_dir')


def _workspace():
    """Return ~/ros2_ws, from <ws>/install/my_first_robot/share/my_first_robot."""
    share = get_package_share_directory('my_first_robot')
    return os.path.abspath(os.path.join(share, '..', '..', '..', '..'))


def generate_launch_description():
    share = get_package_share_directory('my_first_robot')
    ws = _workspace()
    full_launch = os.path.join(share, 'launch', 'disaster_bot_full.launch.py')
    yolo_python = os.path.join(ws, 'yolo_env', 'bin', 'python')
    posture_script = os.path.join('src', 'my_first_robot', 'my_first_robot',
                                  'victim_posture_estimator.py')
    localizer_script = os.path.join('src', 'my_first_robot', 'my_first_robot',
                                    'victim_localizer.py')
    guard_script = os.path.join('src', 'my_first_robot', 'my_first_robot',
                                'victim_detection_guard.py')

    full_defaults = {'gui': 'true', 'x_pose': '0.0', 'y_pose': '-1.2', 'yaw': '0.0',
                     'explore_max_steps': '30', 'record_dir': ''}
    return LaunchDescription([
        DeclareLaunchArgument('posture_enabled', default_value='true',
                              description='Start victim_posture_estimator'),
        DeclareLaunchArgument('posture_model', default_value=DEFAULT_POSTURE_MODEL,
                              description='YOLO11m-pose weights file'),
        DeclareLaunchArgument('posture_max_rate_hz', default_value='1.0',
                              description='Maximum posture inference rate (Hz)'),
        DeclareLaunchArgument('localizer_enabled', default_value='true',
                              description='Start the advisory victim_localizer'),
        DeclareLaunchArgument('localizer_rvc', default_value='false',
                              description='Enable the localizer RVC withhold checks'),
        DeclareLaunchArgument('nav_guard', default_value='false',
                              description='Feed the estimator only RVC-checked detections'),
        DeclareLaunchArgument('nav_guard_close_pair', default_value='false',
                              description='Guard also withholds ambiguous close-pair boxes'),
        *[DeclareLaunchArgument(a, default_value=full_defaults[a]) for a in FULL_ARGS],

        # nav_guard: every launch_ros Node started after this point (the included
        # full launch's nodes; the estimator is the only /victim_detections
        # subscriber among them) reads /victim_detections_nav instead.
        SetRemap(src='/victim_detections', dst='/victim_detections_nav',
                 condition=IfCondition(LaunchConfiguration('nav_guard'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={a: LaunchConfiguration(a) for a in FULL_ARGS}.items()),

        ExecuteProcess(
            name='victim_posture_estimator',
            cmd=[yolo_python, posture_script, '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', ['model_path:=', LaunchConfiguration('posture_model')],
                 '-p', ['max_rate_hz:=', LaunchConfiguration('posture_max_rate_hz')]],
            cwd=ws, output='screen', sigterm_timeout='5', sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('posture_enabled'))),

        ExecuteProcess(
            name='victim_localizer',
            cmd=['python3', localizer_script, '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', ['rvc_enabled:=', LaunchConfiguration('localizer_rvc')]],
            cwd=ws, output='screen', sigterm_timeout='5', sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('localizer_enabled'))),

        ExecuteProcess(
            name='victim_detection_guard',
            cmd=['python3', guard_script, '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', ['exclusive_cluster_check:=', LaunchConfiguration('nav_guard_close_pair')]],
            cwd=ws, output='screen', sigterm_timeout='5', sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('nav_guard'))),
    ])
