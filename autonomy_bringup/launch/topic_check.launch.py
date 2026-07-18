"""
Topic-check launch file: brings up every real node (foxglove + every task's
sensors) with SLAM off by default, purely so you can run

    ros2 launch autonomy_bringup topic_check.launch.py
    # in a second terminal:
    ros2 topic list

and get the ACTUAL topic names your nodes publish, instead of trusting
whatever's written in a doc or a launch-file comment. Foxglove's whitelist
here is '.*' (everything) so nothing is hidden while you're checking.

This is just test_all.launch.py with enable_slam defaulted to false - SLAM
adds startup/shutdown overhead and CPU load you don't need for a topic-name
check, so it's off unless you explicitly turn it on.

Every enable_X flag from test_all.launch.py is still available here if you
want to check a subset instead of everything at once, e.g.:
    ros2 launch autonomy_bringup topic_check.launch.py enable_panorama_stitcher:=false

Once real names are confirmed, go update the TOPIC_WHITELIST lists
in snack_run.launch.py / heist_mission.launch.py / rover_cooked.launch.py /
exploration.launch.py to match - and fix any remapping in the node's own
launch file if the name doesn't match what the task doc expected.
"""


import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ADJUST: package this launch tree lives in
BRINGUP_PACKAGE = 'autonomy_bringup'


def generate_launch_description():
    # Same flags as test_all.launch.py, just with enable_slam defaulted off.
    args = [
        DeclareLaunchArgument('enable_foxglove', default_value='true'),
        DeclareLaunchArgument('enable_imu_gps', default_value='true'),
        DeclareLaunchArgument('enable_heading', default_value='true'),
        DeclareLaunchArgument('enable_morse_detector', default_value='true'),
        DeclareLaunchArgument('enable_route', default_value='true'),
        DeclareLaunchArgument('enable_panorama_stitcher', default_value='true'),
    ]

    test_all_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', 'test_all.launch.py',
            ])
        ),
        launch_arguments={
            'enable_foxglove': LaunchConfiguration('enable_foxglove'),
            'enable_imu_gps': LaunchConfiguration('enable_imu_gps'),
            'enable_heading': LaunchConfiguration('enable_heading'),
            'enable_morse_detector': LaunchConfiguration('enable_morse_detector'),
            'enable_route': LaunchConfiguration('enable_route'),
            'enable_panorama_stitcher': LaunchConfiguration('enable_panorama_stitcher'),
        }.items(),
    )

    return LaunchDescription(args + [test_all_include])
