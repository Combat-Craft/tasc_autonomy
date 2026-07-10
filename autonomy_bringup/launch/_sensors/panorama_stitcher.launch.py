"""
Panorama stitcher launch file.

Args:
  enable_panorama_stitcher (bool, default true) - launch the panorama node

Topics produced:
  /panorama_trigger, /motor_cmd

Used by: exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Confirmed via `ros2 pkg executables autonomy_vision`
PANORAMA_PACKAGE = 'autonomy_vision'
PANORAMA_EXECUTABLE = 'panorama_stitcher'
PANORAMA_NODE_NAME = 'panorama_stitcher'


def generate_launch_description():
    enable_panorama_arg = DeclareLaunchArgument(
        'enable_panorama_stitcher', default_value='true',
        description='Launch the panorama stitcher node'
    )

    panorama_node = Node(
        package=PANORAMA_PACKAGE,
        executable=PANORAMA_EXECUTABLE,
        name=PANORAMA_NODE_NAME,
        output='screen',
    )

    panorama_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_panorama_stitcher')),
        actions=[panorama_node],
    )

    return LaunchDescription([
        enable_panorama_arg,
        panorama_group,
    ])
