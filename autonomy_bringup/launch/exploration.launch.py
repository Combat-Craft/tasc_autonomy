"""
Bringup launch file for the Exploration task.

Nodes:
  - foxglove (whitelist scoped to this task's topics)
  - lidar + slam_toolbox (/scan, /map)
  - imu_gps + heading (/imu/acc_gyro, /imu/mag, /gps/fix, /heading, /cardinal_compass)
  - route (/gps/path)
  - panorama_stitcher (/panorama_trigger, /motor_cmd)
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _sensors.foxglove_helper import make_foxglove_node  # noqa: E402

# ADJUST: package this launch tree lives in
BRINGUP_PACKAGE = 'autonomy_bringup'

TOPIC_WHITELIST = [
    '/scan',
    '/map',
    '/imu/acc_gyro',
    '/imu/mag',
    '/gps/fix',
    '/heading',
    '/cardinal_compass',
    '/gps/path',
    '/panorama_trigger',
    '/motor_cmd',
]


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
    foxglove_node = make_foxglove_node(TOPIC_WHITELIST)

    lidar_include = _include('lidar.launch.py', {
        'enable_lidar': 'true',
        'enable_slam': 'true',
    })

    imu_gps_include = _include('imu_gps.launch.py', {
        'enable_imu_gps': 'true',
        'enable_heading': 'true',
    })

    route_include = _include('route.launch.py', {
        'enable_route': 'true',
    })

    panorama_include = _include('panorama_stitcher.launch.py', {
        'enable_panorama_stitcher': 'true',
    })

    return LaunchDescription([
        foxglove_node,
        lidar_include,
        imu_gps_include,
        route_include,
        panorama_include,
    ])
