'''
# orbbec gemini2 - FOR NOW, DIRECT AS ^LAUNCH IS BAD
# but why bad?
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare('orbbec_camera'),   
                  'launch',
                  "gemini2.launch.py"
            ]),
            launch_arguments={
                # == hardware stuff ==
                'camera_name':'camera',
                'depth_registration':'true', # reset to true for point cloud
                #'serial_number':'',
                #'usb_port':'',
                'device_num':'1',
                'vendor_id':'0x2bc5',
                #'product_id':''

                # == Point cloud params ==
                'enable_point_cloud':'true',
                'enable_colored_point_cloud':'true',
                #'cloud_frame_id':'',
                'point_cloud_qos':'default',
                'connection_delay':'100',

                # == color stuff (resolution, fps, profile, mirroring) ==
                'color_width':'1280',
                'color_height':'720',
                'color_fps':'30',
                'color_format':'MJPG',
                'enable_color':'true',
                'flip_color':'false',
                'color_qos':'default',
                'color_camera_info_qos':'default',
                'enable_color_auto_exposure':'true',
                'color_exposure':'-1',
                'color_gain':'-1',
                'enable_color_auto_white_balance':'true',
                'color_white_balance':'-1',

                # == depth stuff (resolution, fps, profile, mirroring) == 
                'depth_width':'640',
                'depth_height':'400',
                'depth_fps':'15',
                'depth_format':'Y14',
                'enable_depth':'true',
                'flip_depth':'false',
                'depth_qos':'default',
                'depth_camera_info_qos':'default',
                'depth_ae_roi_left':'-1',
                'depth_ae_roi_top':'-1',
                'depth_ae_roi_right':'-1',
                'depth_ae_roi_bottom':'-1',

                # == IR stuff ==
                'ir_width':'640',
                'ir_height':'400',
                'ir_fps':'15',
                'ir_format':'Y8',
                'enable_ir':'true',
                'flip_ir':'false',
                'ir_qos':'default',
                'ir_camera_info_qos':'default',
                'enable_ir_auto_exposure':'true',
                'ir_exposure':'-1',
                'ir_gain':'-1',
        
                # IMU stuff
                'enable_sync_output_accel_gyro':'true',
                'enable_accel':'false',
                'accel_rate':'100hz',
                'accel_range':'4g',
                'enable_gyro':'false',
                'gyro_rate':'100hz',
                'gyro_range':'1000dps',
                'liner_accel_cov':'0.01',
                'angular_vel_cov':'0.01',
        
                # tf
                'publish_tf':'true',
                'tf_publish_rate':'0.0',
                #'ir_info_url':'',
                #'color_info_url':'',
                'log_level':'none',
                'enable_publish_extrinsic':'false',
                'enable_d2c_viewer':'false',
                'enable_ldp':'true',
                
                # Configure the path for depth filter file, for example: /config/depthfilter/Gemini2_v1.7.json
                #'depth_filter_config':'',
                # Depth work mode support is as follows:
                # Unbinned Dense Default
                # Unbinned Sparse Default
                # Binned Sparse Default
                # Obstacle Avoidance
                #'depth_work_mode':'',
                'sync_mode':'standalone',
                'depth_delay_us':'0',
                'color_delay_us':'0',
                'trigger2image_delay_us':'0',
                'trigger_out_delay_us':'0',
                'trigger_out_enabled':'false',
                'enable_frame_sync':'true',
                'ordered_pc':'false',
                'use_hardware_time':'true',
                'enable_depth_scale':'true',
                'align_mode':'HW',
                'retry_on_usb3_detection_failure':'false',
                'laser_energy_level':'-1',
                'enable_heartbeat':'false'
           
            }.items()
        ),
'''


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription   #this and the next 2 are for launch files of another package
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

## THE PURPOSE OF THIS CODE TO RUN ON FOXGLOVE GUI
## WHITELIST MAY BE NEEDED TO PREVENT UNNEEDED TOPICS FROM STREAMING OVER THE NETWORK

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
        
        # Launch all sensors - currently imu, gps, gps route logger (real) 
        
        #IncludeLaunchDescription(
        #    PathJoinSubstitution([
        #      FindPackageShare("autonomy_sensors"),
        #          "launch",
        #          "sensors.launch.py"
        #    ])    
        #),
        
        Launch all cameras/visions - currently only multicamera
        IncludeLaunchDescription(
            PathJoinSubstitution([
              FindPackageShare("autonomy_vision"),
                 "launch",
                 "vision.launch.py"
           ])    
        ),

    ])
    

