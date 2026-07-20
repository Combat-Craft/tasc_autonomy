"""
Route planning launch file.

Args:
  enable_route (bool, default true) - launch the route/path-planning node

Topics produced:
  /gps/path

Depends on: /gps/fix from imu_gps (not enforced here — task launch file
is responsible for including imu_gps before/alongside this).

Used by: exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ADJUST: verify these match the actual repo
ROUTE_PACKAGE = 'autonomy_sensors'                # 
ROUTE_EXECUTABLE = 'route_logger'        # 
ROUTE_NODE_NAME = 'route_node'   # not sure if it's route_node or route_logger


def generate_launch_description():
    enable_route_arg = DeclareLaunchArgument(
        'enable_route', default_value='true',
        description='Launch the route/path-planning node'
    )

    route_node = Node(
        package=ROUTE_PACKAGE,
        executable=ROUTE_EXECUTABLE,
        name=ROUTE_NODE_NAME,
    )

    route_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_route')),
        actions=[route_node],
    )

    return LaunchDescription([
        enable_route_arg,
        route_group,
    ])
