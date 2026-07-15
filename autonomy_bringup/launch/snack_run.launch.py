"""
Bringup launch file for the Snack Run task.

Nodes:
  - foxglove (whitelist scoped to this task's topics)
  - lidar + slam_toolbox (/scan, /map)
  - imu_gps + heading (/imu/acc_gyro, /imu/mag, /gps/fix, /heading, /cardinal_compass)
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# The installed share/<pkg>/launch directory is NOT on PYTHONPATH by
# default, so _sensors (a plain subdirectory, not an installed Python
# module) can't be imported without this. Adding this file's own directory
# lets `_sensors` resolve as a sibling package.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _sensors.foxglove_helper import make_foxglove_node  # noqa: E402

# ADJUST: topic names in topic whitelist
BRINGUP_PACKAGE = 'autonomy_bringup'

TOPIC_WHITELIST = [
    '/scan',
    '/map',
    '/imu/acc_gyro',
    '/imu/mag',
    '/gps/fix',
    '/heading',
    '/cardinal_compass',
]


def generate_launch_description():
    foxglove_node = make_foxglove_node(TOPIC_WHITELIST)

    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(BRINGUP_PACKAGE),
                'launch', '_sensors', 'lidar.launch.py',
            ])
        ),
        launch_arguments={
            'enable_lidar': 'true',
            'enable_slam': 'false',
        }.items(),
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
            'enable_heading': 'true',
        }.items(),
    )

    return LaunchDescription([
        foxglove_node,
        lidar_include,
        imu_gps_include,
    ])
