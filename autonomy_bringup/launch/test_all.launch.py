"""
Test-all launch file: brings up every node across every task, with one
enable_X argument per node/subsystem so any of them can be individually
disabled at the competition for testing/debugging.

All flags default true. Example partial disable:
  ros2 launch autonomy_bringup test_all.launch.py enable_morse_detector:=false enable_route:=false

Flags:
  enable_foxglove
  enable_lidar, enable_slam
  enable_imu_gps, enable_heading
  enable_morse_detector
  enable_route
  enable_panorama_stitcher
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _sensors.foxglove_helper import make_foxglove_node  # noqa: E402

# ADJUST: package this launch tree lives in
BRINGUP_PACKAGE = 'autonomy_bringup'

# Union of every topic across every task - test_all is for debugging, so
# nothing is filtered out.
TOPIC_WHITELIST = ['.*']


def _include(name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', '_sensors', name,
            ])
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument('enable_foxglove', default_value='true'),
        DeclareLaunchArgument('enable_imu_gps', default_value='true'),
        DeclareLaunchArgument('enable_heading', default_value='true'),
        DeclareLaunchArgument('enable_morse_detector', default_value='true'),
        DeclareLaunchArgument('enable_route', default_value='true'),
        DeclareLaunchArgument('enable_panorama_stitcher', default_value='true'),
    ]

    # foxglove is a bare Node (see foxglove_helper.py docstring for why),
    # so it needs its own GroupAction/IfCondition gate here rather than
    # relying on internal conditioning like the other subsystems below.
    foxglove_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        actions=[make_foxglove_node(TOPIC_WHITELIST)],
    )

    # Each of these sub-launch files handles its own IfCondition internally
    # (see lidar.launch.py / imu_gps.launch.py / etc.), so we just forward
    # our top-level flags straight through as launch_arguments.

    imu_gps_include = _include('imu_gps.launch.py', {
        'enable_imu_gps': LaunchConfiguration('enable_imu_gps'),
        'enable_heading': LaunchConfiguration('enable_heading'),
    })

    morse_detector_include = _include('morse_detector.launch.py', {
        'enable_morse_detector': LaunchConfiguration('enable_morse_detector'),
    })

    route_include = _include('route.launch.py', {
        'enable_route': LaunchConfiguration('enable_route'),
    })

    panorama_include = _include('motor_panorama.launch.py', {
        'enable_panorama_stitcher': LaunchConfiguration('enable_panorama_stitcher'),
    })

    return LaunchDescription(args + [
        foxglove_group,        imu_gps_include,
        morse_detector_include,
        route_include,
        panorama_include,
    ])