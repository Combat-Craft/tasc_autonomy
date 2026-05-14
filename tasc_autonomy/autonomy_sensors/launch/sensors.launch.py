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
            
        ),

        # copied from https://github.com/mvipin/perceptor/blob/main/launch/rplidar.launch.py
        # who is using the exact same lidar as we are
        ## Use the official RPLidar A1 launch file from rplidar_ros package
        ## This is the recommended approach for A1M8 models
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([os.path.join(
        #        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
        #    )]),
        #    launch_arguments={
        #        'channel_type': 'serial',
        #        'serial_port': '/dev/ttyUSB0',
        #        'serial_baudrate': '115200',
        #        'frame_id': 'laser',
        #        'inverted': 'false',
        #        'angle_compensate': 'true',
        #        'scan_mode': 'Sensitivity'  # Optimal for A1M8
        #    }.items()
        #),
        
        # GPS node, for NavSatFix msg and foxglove TextAnnotation msg (latitude and longitude)
        #Node(
        #    package='autonomy_sensors', # Replace with your GPS driver package name
        #    executable='gps_node', # Replace with your GPS driver executable
        #    name='gps_node',
        #    output='screen',
        #    parameters=[
        #        # Add any GPS specific parameters here (e.g., port, baud_rate)
        #        {'port': '/dev/ttyUSB1'},
        #        {'baud_rate': 115200},           
        #    ]
        #),
        # IMU node, for IMU ms, g and foxglove TextAnnotation msg (latitude and longitude)
        Node(
            package='autonomy_sensors', 
            executable='imu_node', 
            name='imu_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB1'},
                {'baud_rate': 115200},           
            ]
        ),
        
        # Examples if needed
        
        # Node(
        #     package='navigation',
        #     executable='multi_camera_publisher',
        #     name='camera1_node',
        #     arguments=['1'] #The media stream for the camera (eg. '/dev/video2')
        # ),
        
        #Node(
        #    package='navigation', 
        #    executable='gps_display',
        #    name='gps_display',
        #    output='screen',
        #    # You can remap topics here if your script uses different topic names
        #    # remappings=[
        #    #     ('/image_raw', '/camera/image_raw'),
        #    #     ('/gps/fix', '/your_gps_topic'),
        #    # ]
        #),
    ])
    

