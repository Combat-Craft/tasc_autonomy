"""
Bringup launch file for the RoverCooked task.

Nodes:
  - foxglove (whitelist scoped to this task's topics)
  - lidar + slam_toolbox (/scan, /map)
  - imu_gps WITHOUT heading (/imu/acc_gyro, /imu/mag, /gps/fix)
    -> heading/cardinal_compass not needed for this task
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
    '/motor_cmd',
]


def generate_launch_description():
    foxglove_node = make_foxglove_node(TOPIC_WHITELIST)

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
    
    imu_gps_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', '_sensors', 'imu_gps.launch.py',
            ])
        ),
        launch_arguments={
            'enable_imu_gps': 'true',
            'enable_heading': 'false',
        }.items(),
    )
    panorama_include = _include('motor_panorama.launch.py', {
            'enable_panorama_stitcher': 'true',
        })

    return LaunchDescription([
        foxglove_node,
        imu_gps_include,
        panorama_include,
    ])
