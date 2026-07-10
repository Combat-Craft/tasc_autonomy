"""
Lidar + slam_toolbox launch file.

Args:
  enable_lidar (bool, default true)  - launch the lidar driver node
  enable_slam  (bool, default true)  - launch slam_toolbox (requires lidar)

Used by: snack_run, rover_cooked, exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ADJUST: verify these match your actual repo
LIDAR_PACKAGE = 'rplidar_ros'          # ADJUST
LIDAR_EXECUTABLE = 'rplidar_node'      # ADJUST
LIDAR_NODE_NAME = 'rplidar_node'
LIDAR_FRAME_ID = 'laser_frame'         # ADJUST
LIDAR_SERIAL_PORT = '/dev/ttyUSB0'     # ADJUST
LIDAR_SCAN_TOPIC = '/scan'

SLAM_TOOLBOX_PACKAGE = 'slam_toolbox'
SLAM_TOOLBOX_LAUNCH_FILE = 'online_async_launch.py'  # ADJUST if using a different slam mode


def generate_launch_description():
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Launch the lidar driver node'
    )
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam', default_value='false',  # set default to false since as of 10-July-2026 we have no nav2 stack
        description='Launch slam_toolbox (produces /map)'
    )

    lidar_node = Node(
        package=LIDAR_PACKAGE,
        executable=LIDAR_EXECUTABLE,
        name=LIDAR_NODE_NAME,
        output='screen',
        parameters=[{
            'frame_id': LIDAR_FRAME_ID,
            'serial_port': LIDAR_SERIAL_PORT,
        }],
        remappings=[('/scan', LIDAR_SCAN_TOPIC)],
    )

    lidar_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        actions=[lidar_node],
    )

    slam_toolbox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(SLAM_TOOLBOX_PACKAGE),
                'launch',
                SLAM_TOOLBOX_LAUNCH_FILE,
            ])
        ),
        condition=IfCondition(LaunchConfiguration('enable_slam')),
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_slam_arg,
        lidar_group,
        slam_toolbox_include,
    ])