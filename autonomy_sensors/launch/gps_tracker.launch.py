#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomy_sensors',
            executable='gps_frame_fixer',
            name='gps_frame_fixer',
            output='screen',
        ),
        Node(
            package='autonomy_sensors',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765}],
            output='screen',
        ),
    ])
