"""
IMU + GPS launch file.

Args:
  enable_imu_gps  (bool, default true)  - launch imu/gps sensing at all
  enable_heading  (bool, default true)  - whether heading/cardinal_compass
                                           is needed (RoverCooked sets false)

IMPORTANT - this is NOT three independent nodes stacked together.
There is no standalone 'heading_node' executable. Heading/cardinal_compass
is only produced by the COMBINED 'gps_imu_broadcaster' node, which also
covers /imu/acc_gyro, /imu/mag, /gps/fix itself. The split 'imu_node' +
'gps_node' executables do NOT produce heading at all.

So enable_heading switches which node(s) run, it doesn't add a node on top:
  enable_heading=true  -> run gps_imu_broadcaster only (combined, has heading)
  enable_heading=false -> run imu_node + gps_node (split, no heading)

Topics produced:
  /imu/acc_gyro, /imu/mag, /gps/fix          (always, when enable_imu_gps)
  /heading, /cardinal_compass                (only when enable_heading, via
                                               gps_imu_broadcaster)

Used by: snack_run, heist_mission, rover_cooked, exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Confirmed via `ros2 pkg executables autonomy_sensors`
IMU_GPS_PACKAGE = 'autonomy_sensors'
IMU_EXECUTABLE = 'imu_node'
GPS_EXECUTABLE = 'gps_node'
COMBINED_EXECUTABLE = 'gps_imu_broadcaster'   # produces heading + everything else

IMU_NODE_NAME = 'imu_node'
GPS_NODE_NAME = 'gps_node'
COMBINED_NODE_NAME = 'gps_imu_broadcaster'


def generate_launch_description():
    enable_imu_gps_arg = DeclareLaunchArgument(
        'enable_imu_gps', default_value='true',
        description='Launch imu + gps sensing'
    )
    enable_heading_arg = DeclareLaunchArgument(
        'enable_heading', default_value='true',
        description='Use combined gps_imu_broadcaster (has heading) instead of split imu_node/gps_node'
    )

    combined_node = Node(
        package=IMU_GPS_PACKAGE,
        executable=COMBINED_EXECUTABLE,
        name=COMBINED_NODE_NAME,
        output='screen',
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

    # enable_imu_gps=true AND enable_heading=true -> combined node (has heading)
    combined_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_heading')),
        actions=[combined_node],
    )

    # enable_imu_gps=true AND enable_heading=false -> split nodes (no heading)
    split_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('enable_heading')),
        actions=[imu_node, gps_node],
    )

    # Outer gate: nothing above runs at all unless enable_imu_gps is true.
    # Nesting GroupAction conditions like this ANDs them together.
    imu_gps_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_imu_gps')),
        actions=[combined_group, split_group],
    )

    return LaunchDescription([
        enable_imu_gps_arg,
        enable_heading_arg,
        imu_gps_group,
    ])
