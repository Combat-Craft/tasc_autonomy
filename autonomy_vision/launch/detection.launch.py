#!/usr/bin/env python3

"""
ROS2 Launch file

# Launch Command: ros2 launch autonomy_vision detection.launch.py

1. USB Camera Node
2. YOLO Detection Node

System flow: USB Camera (/dev/video0) --> usb_cam node --> image topic (/image_raw) --> webcam_detection2D mode (YOLO) --> detection output

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    # LAUNCH ARGUMENTS
    # Camera device path (default: /dev/video0)
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Camera device path'
    )

    # Camera resolution width
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='640',
        description='Camera image width'
    )

    # Camera resolution height
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='480',
        description='Camera image height'
    )

    # Camera frame rate
    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='30.0',
        description='Camera framerate'
    )
 
    #  1. USB Camera Node:
    # Captures raw images from USB camera and publishes them to ROS2 image topics for downstream processing.
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device':  LaunchConfiguration('camera_device'),
            'pixel_format':  'mjpeg2rgb',
            'image_width':   LaunchConfiguration('image_width'),
            'image_height':  LaunchConfiguration('image_height'),
            'framerate':     LaunchConfiguration('framerate'),
        }]
    )
 
    #  2. YOLO Detector Node
    # Runs object detection on incoming camera frames.
    # Delayed start ensures usb_cam is fully initialized before detection begins to prevent empty frame errors.
    detector_node = TimerAction(
        period=2.0, # 2 sec delay before starting detector
        actions=[
            Node(
                package='autonomy_vision',
                executable='webcam_detection2D',
                name='webcam_detector',
                output='screen',
            )
        ]
    )
 
    # Launch description
    return LaunchDescription([
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_cam_node,
        detector_node,
    ])