"""
Bringup launch file for the Heist Mission task.

Nodes:
  - foxglove (whitelist scoped to this task's topics)
  - imu_gps + heading (/imu/acc_gyro, /imu/mag, /gps/fix, /heading, /cardinal_compass)
  - morse_detector (/morse_code, /morse_decoded, /morse_debug_image)

NOTE: lidar is disconnected for this task.
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

# ADJUST: topic whitelist
BRINGUP_PACKAGE = 'autonomy_bringup'

TOPIC_WHITELIST = [
    '/imu/acc_gyro',
    '/imu/mag',
    '/gps/fix',
    '/heading',
    '/cardinal_compass',
    '/morse_code',
    '/morse_decoded',
    '/morse_debug_image',
    '/morse_reset',
    '/motor_cmd',
]


def generate_launch_description():
    foxglove_node = make_foxglove_node(TOPIC_WHITELIST)

    imu_gps_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', '_sensors', 'imu_gps.launch.py',
            ])
        ),
        launch_arguments={
            'enable_imu_gps': 'true',
            'enable_heading': 'true',
        }.items(),
    )

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

    morse_detector_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', '_sensors', 'morse_detector.launch.py',
            ])
        ),
        launch_arguments={
            'camera_index': 'rtspt://admin:@192.168.1.116:8554/profile0',
        }.items(),
    )

    panorama_include = _include('motor_panorama.launch.py', {
        'enable_panorama_stitcher': 'true',
    })

    return LaunchDescription([
        foxglove_node,
        imu_gps_include,
        morse_detector_include,
        panorama_include,
    ])
