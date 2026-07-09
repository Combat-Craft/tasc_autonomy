
"""
Failsafe bringup: the "just in case" launch file.

Should never need to be used. Deliberately minimal - foxglove bridge plus
a raw camera driver publishing straight to a compressed image topic.
Depends ONLY on well-established, upstream ROS2 packages (v4l2_camera,
image_transport) - no custom autonomy_* packages, no lidar, no imu/gps,
no processing nodes of any kind. The goal is that nothing in this file
can crash and take the video feed down with it.

Args:
  video_device (string, default /dev/video0) - camera device path
"""

# ADJUST: Might have to publish to raw image topic instead of compressed

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _sensors.foxglove_helper import make_foxglove_node  # noqa: E402

# Only the raw camera feed - nothing else is whitelisted so nothing else
# can crowd out the video stream in the bridge.
TOPIC_WHITELIST = [
    '/failsafe_cam/image_raw/compressed',
]


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Camera device path for the failsafe video feed'
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='failsafe_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
        }],
        remappings=[
            ('/image_raw', '/failsafe_cam/image_raw'),
        ],
    )

    foxglove_node = make_foxglove_node(TOPIC_WHITELIST)

    return LaunchDescription([
        video_device_arg,
        camera_node,
        foxglove_node,
    ])
