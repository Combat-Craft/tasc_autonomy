from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value= '-1',
        description=(
            'Camera source: int index (e.g. 0), /dev/videoN path, '
            'or 1 for a rtsp(t) stream'
        )
    )

    rtsp_uri_arg = DeclareLaunchArgument(
        'rtsp_uri',
        default_value='rtsp://admin:@192.168.1.116:8554/profile0',
        description=(
            'rtsp(t)://... URL for an IP camera. '
            'rtspt:// forces TCP transport (recommended over plain )'
            'rtsp:// which is usually UDP and more prone to dropped '
            'frames / timing jitter).'
        )
    )

    threshold_percentile_arg = DeclareLaunchArgument(
        'threshold_percentile',
        default_value='85.0'
    )

    min_bright_fraction_arg = DeclareLaunchArgument(
        'min_bright_fraction',
        default_value='0.01'
    )

    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='25.0'
    )

    log_cpu_stats_arg = DeclareLaunchArgument(
        'log_cpu_stats',
        default_value='true',
        description='Log periodic CPU/memory stats to watch for spikes'
    )

    morse_node = Node(
        package='autonomy_vision',
        executable='morse_detector',
        name='morse_code_detector',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'rtsp_uri': LaunchConfiguration('rtsp_uri'),
            'threshold_percentile': LaunchConfiguration('threshold_percentile'),
            'min_bright_fraction': LaunchConfiguration('min_bright_fraction'),
            'publish_hz': LaunchConfiguration('publish_hz'),
            'log_cpu_stats': LaunchConfiguration('log_cpu_stats'),
        }]
    )

    return LaunchDescription([
        camera_index_arg,
        rtsp_uri_arg,
        threshold_percentile_arg,
        min_bright_fraction_arg,
        publish_hz_arg,
        log_cpu_stats_arg,
        morse_node,
    ])
