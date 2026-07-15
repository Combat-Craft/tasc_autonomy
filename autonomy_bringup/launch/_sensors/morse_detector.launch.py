"""
Morse code detector launch file.

Args:
  enable_morse_detector (bool, default true) - launch the morse detector node

Topics produced:
  /morse_code, /morse_decoded, /morse_debug_image

Used by: heist_mission, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# NOTE:  The actual node entry point is 'morse_detector'.(I think)
MORSE_PACKAGE = 'autonomy_vision'
MORSE_EXECUTABLE = 'morse_detector'
MORSE_NODE_NAME = 'morse_detector'


def generate_launch_description():
    enable_morse_detector_arg = DeclareLaunchArgument(
        'enable_morse_detector', default_value='true',
        description='Launch the morse code detector node'
    )

    morse_node = Node(
        package=MORSE_PACKAGE,
        executable=MORSE_EXECUTABLE,
        name=MORSE_NODE_NAME,
        output='screen',
    )

    morse_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_morse_detector')),
        actions=[morse_node],
    )

    return LaunchDescription([
        enable_morse_detector_arg,
        morse_group,
    ])
