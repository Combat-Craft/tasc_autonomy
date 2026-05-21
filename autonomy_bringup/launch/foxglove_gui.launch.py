from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription   #this and the next 2 are for launch files of another package
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

# look at this for finetuning: 
# https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html

# I used this to include foxglove
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

def generate_launch_description():
    return LaunchDescription([

        # Foxglove Bridge websocket server for ROS2
        # https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge#configuration
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare("foxglove_bridge"),
                  "launch",
                  "foxglove_bridge_launch.xml"
            ]),
            launch_arguments={
                "port": '8765',
                ## the whitelists uses regex 
                ## https://en.cppreference.com/cpp/regex/ecmascript  
                # "topic_whitelist": [".*", #the default ],
                # "service_whitelist": [".*"]
                # "param_whitelist": [".*"]
                # "num_threads": 0 # defaults to 0 = 1 thread per core
            }.items()      
        ),
        
        # Launch all sensors
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare("autonomy_sensors"),
                  "launch",
                  "sensors.launch.py"
            ])    
        ),
        
        # Launch all cameras/visions
        #IncludeLaunchDescription(
        #    PathJoinSubstitution([
        #      FindPackageShare("autonomy_vision"),
         #         "launch",
        #          "vision.launch.py"
        #    ])    
        #),
        
        # orbbec gemini2 - FOR NOW, DIRECT AS ^LAUNCH IS BAD
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare('orbbec_camera'),   
                  'launch',
                  #"gemini.launch.py"
                  "gemini2_edited2.launch.py"
            ]),
            # launch_arguments={}.items()
        ),

    ])
    

