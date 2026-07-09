"""
Shared helper for creating a foxglove_bridge Node with a topic whitelist.

NOT a launch file itself — imported directly by each task launch file.
This is intentional: foxglove_bridge's `topic_whitelist` parameter is a
string array, and ROS2 launch's DeclareLaunchArgument/LaunchConfiguration
mechanism only carries strings across IncludeLaunchDescription boundaries.
Passing a real Python list through that mechanism requires JSON-string
round-tripping and is a common source of silent bugs. Importing a plain
function and passing a literal list sidesteps the problem entirely.
"""

from launch_ros.actions import Node

# ADJUST if your foxglove_bridge package/executable names differ
FOXGLOVE_PACKAGE = 'foxglove_bridge'
FOXGLOVE_EXECUTABLE = 'foxglove_bridge'


def make_foxglove_node(topic_whitelist: list, port: int = 8765) -> Node:
    """
    Build a foxglove_bridge Node.

    :param topic_whitelist: list of regex strings, e.g. ['/scan', '/imu/.*']
                             NOTE: foxglove_bridge matches topic_whitelist
                             entries as regex, not literal strings. Anchor
                             or escape as needed (e.g. '/scan$' if you want
                             an exact match and not also '/scan_filtered').
    :param port: websocket port for the bridge
    """
    return Node(
        package=FOXGLOVE_PACKAGE,
        executable=FOXGLOVE_EXECUTABLE,
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'topic_whitelist': topic_whitelist,
        }],
    )