"""
IMU + GPS launch file.

Args:
  enable_imu_gps  (bool, default true)  - launch imu + gps driver nodes
  enable_heading  (bool, default true)  - launch heading/cardinal_compass node
                                           (RoverCooked sets this false)

Topics produced:
  /imu/acc_gyro, /imu/mag, /gps/fix          (always, when enable_imu_gps)
  /heading, /cardinal_compass                (only when enable_heading)

Used by: snack_run, heist_mission, rover_cooked, exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ADJUST: verify these match the actual package and node names
IMU_GPS_PACKAGE = 'autonomy_sensors'        # ADJUST
IMU_EXECUTABLE = 'imu_node'                    # ADJUST
GPS_EXECUTABLE = 'gps_node'                    # ADJUST
HEADING_EXECUTABLE = 'heading'            # ADJUST

IMU_NODE_NAME = 'imu_node'
GPS_NODE_NAME = 'gps_node'
HEADING_NODE_NAME = 'heading'

# ?? difference between IMU_EXECUTABLE AND IMU_NODE_NAME 


def generate_launch_description():
    enable_imu_gps_arg = DeclareLaunchArgument(
        'enable_imu_gps', default_value='true',
        description='Launch imu + gps driver nodes'
    )
    enable_heading_arg = DeclareLaunchArgument(
        'enable_heading', default_value='true',
        description='Launch heading/cardinal_compass node'
    )

    imu_node = Node(
        package=IMU_GPS_PACKAGE,
        executable=IMU_EXECUTABLE,
        name=IMU_NODE_NAME,
        output='screen',
    )

    gps_node = Node(
        package=IMU_GPS_PACKAGE,
        executable=GPS_EXECUTABLE,
        name=GPS_NODE_NAME,
        output='screen',
    )

    heading_node = Node(
        package=IMU_GPS_PACKAGE,
        executable=HEADING_EXECUTABLE,
        name=HEADING_NODE_NAME,
        output='screen',
    )

    imu_gps_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_imu_gps')),
        actions=[imu_node, gps_node],
    )

    heading_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_heading')),
        actions=[heading_node],
    )

    return LaunchDescription([
        enable_imu_gps_arg,
        enable_heading_arg,
        imu_gps_group,
        heading_group,
    ])