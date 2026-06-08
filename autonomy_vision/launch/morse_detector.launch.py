from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    morse_node = Node(
        package='autonomy_vision',
        executable='morse_detector',
        name='morse_code_detector',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'threshold_percentile': 85.0,
            'min_bright_fraction': 0.01,
            'publish_hz': 10.0
        }]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765
        }]
    )

    return LaunchDescription([
        morse_node,
        foxglove_bridge
    ])
