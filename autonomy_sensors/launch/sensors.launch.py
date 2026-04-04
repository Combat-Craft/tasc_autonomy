from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
    
        # ROS Bridge websocket server, for Foxglove
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket.py',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'port': 9090},
                {'default_call_service_timeout': 5.0},
                {'call_services_in_new_thread': True},
                {'send_action_goals_in_new_thread': True}
            ]
        ),

    ])
