from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription   # for launch files of another package
from launch.substitutions import PathJoinSubstitution # for launch files of another package
from launch_ros.substitutions import FindPackageShare # for launch files of another package
from launch_ros.actions import Node

import os

# look at this for finetuning: 
# https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html

# I used this to include foxglove
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

def generate_launch_description():
    return LaunchDescription([
        # for calling other launch files within the autonomy_vision package
        vision_launch_dir = PathJoinSubstitution([FindPackageShare('autonomy_vision'), 'launch'])
        
        ## call other launch files in this package
        
        # for panorama / fish-eye
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'motor.launch.py'])
        ),

        ## for calling 3rd party launch files
        
        # orbbec gemini2
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare('orbbec_camera'),   
                  'launch',
                  "gemini2_edited2.launch.py"
            ]),
            # launch_arguments={}.items()
        ),
        
        ## call the nodes normally
        
        # multi camera streamer, copied from 
        # https://github.com/Combat-Craft/AN_WS/blob/main/nova_vision/launch/dual_cameras.launch.py
        Node(
            package='autonomy_vision',
            executable='multi_camera_streamer',
            name='camera0_node',
            arguments=['0'] #The media stream for the camera (eg. 0, -1 or '/dev/video2')
        ),
        Node(
            package='autonomy_vision',
            executable='multi_camera_streamer',
            name='camera1_node',
            arguments=['1'] #The media stream for the camera (eg. 0, -1 or '/dev/video2')
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
    

