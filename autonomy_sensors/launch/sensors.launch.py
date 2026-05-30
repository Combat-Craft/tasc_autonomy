"""
currently not attached to the rover, so placed here for later ease
# RPLidar A1M8
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare('rplidar_ros'),
                  'launch',
                  "rplidar_a1_launch.py"
            ]),
            launch_arguments={
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': '115200',
                'frame_id': 'laser',
                'inverted': 'false',
                'angle_compensate': 'true',
                'scan_mode': 'Sensitivity'  # Optimal for A1M8
            }.items()
            
        ),
"""
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
        # IMU and GPS, shared node version
        Node(
           package='autonomy_sensors', 
           executable='gps_imu_broadcaster', 
           name='gps_imu_node',
           output='screen',
           parameters=[
               {'port': '/dev/ttyUSB0'}, # when with LIDAR, double check
               {'baud_rate': 115200},           
           ]
        ),
    ])
    

