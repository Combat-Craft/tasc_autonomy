#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    # ------------------------------------------------------------------ #
    #  Launch Arguments                                                    #
    # ------------------------------------------------------------------ #
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Camera device path'
    )
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='640',
        description='Camera image width'
    )
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='480',
        description='Camera image height'
    )
    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='30.0',
        description='Camera framerate'
    )
 
    # ------------------------------------------------------------------ #
    #  1. USB Camera Node                                                  #
    # ------------------------------------------------------------------ #
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
 
    # ------------------------------------------------------------------ #
    #  2. YOLO Detector Node                                               #
    #     Delayed 2 s to give usb_cam time to initialise                  #
    # ------------------------------------------------------------------ #
    detector_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='autonomy_vision',
                executable='webcam_detection2D',
                name='webcam_detector',
                output='screen',
            )
        ]
    )
 
    return LaunchDescription([
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_cam_node,
        detector_node,
    ])