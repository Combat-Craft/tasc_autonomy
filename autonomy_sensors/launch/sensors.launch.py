from launch import LaunchDescription
from launch_ros.actions import Node

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
        # GPS node, for NavSatFix msg and foxglove TextAnnotation msg (latitude and longitude)
        Node(
            package='autonomy_sensors', # Replace with your GPS driver package name
            executable='gps_node', # Replace with your GPS driver executable
            name='gps_node',
            output='screen',
            parameters=[
                # Add any GPS specific parameters here (e.g., port, baud_rate)
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 115200},           
            ]
        ),
        # IMU node, for IMU ms, g and foxglove TextAnnotation msg (latitude and longitude)
        Node(
            package='autonomy_sensors', 
            executable='imu_node', 
            name='imu_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
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
    

