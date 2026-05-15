from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription   #this and the next 2 are for launch files of another package
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource # for rplidar
from ament_index_python.packages import get_package_share_directory # for rplidar
from launch_ros.actions import Node

import os

# look at this for finetuning: 
# https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html

# I used this to include foxglove
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

def generate_launch_description():
    return LaunchDescription([

        # Foxglove Bridge websocket server for ROS2,
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare('foxglove_bridge'),
                  'launch',
                  "foxglove_bridge_launch.xml"
            ]),
            launch_arguments={
                'port': '8765',
                #'default_call_service_timeout': 5.0,
                #'call_services_in_new_thread': True,
                #'send_action_goals_in_new_thread': True
            }.items()
            
        )

    ])
    

