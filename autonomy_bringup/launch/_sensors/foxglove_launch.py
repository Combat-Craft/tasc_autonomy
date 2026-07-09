"""
Foxglove bridge launch file.

IMPORTANT: this file is for STANDALONE TESTING ONLY
(`ros2 launch _sensors foxglove.launch.py`) with a default whitelist.

Task launch files (snack_run.launch.py, etc.) should NOT include this file
via IncludeLaunchDescription. Instead they should:

    from _sensors.foxglove_helper import make_foxglove_node
    make_foxglove_node(['/scan', '/imu/.*'])

and add the returned Node directly to their own LaunchDescription.

Why: foxglove_bridge's `topic_whitelist` param is a string ARRAY. ROS2
launch's DeclareLaunchArgument/LaunchConfiguration substitution mechanism
only carries strings across IncludeLaunchDescription boundaries, so a real
Python list can't cross that boundary without JSON-string round-tripping
(a common source of silent bugs). Direct import sidesteps the problem
entirely. See foxglove_helper.py for details.
"""

import os
import sys

from launch import LaunchDescription

# This file lives inside _sensors/ itself, so the parent directory (launch/)
# needs to be on sys.path for `_sensors` to resolve as a package.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _sensors.foxglove_helper import make_foxglove_node  # noqa: E402

# Default whitelist used only when this file is run standalone for testing.
DEFAULT_WHITELIST = ['.*']  # everything, for standalone debugging


def generate_launch_description():
    return LaunchDescription([
        make_foxglove_node(DEFAULT_WHITELIST),
    ])
