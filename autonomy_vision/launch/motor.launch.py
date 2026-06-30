"""
ROS2 Launch file

Launches the motor, camera, and panorama-related nodes needed for manual servo control and panorama capture.

Launch command:
    Normal mode with camera:
        ros2 launch autonomy_vision motor.launch.py

    Test mode without requiring camera frames:
        ros2 launch autonomy_vision motor.launch.py test_mode:=true

    Select a specific camera device:
        ros2 launch autonomy_vision motor.launch.py camera_device:=/dev/video0

Launch arguments:
    - test_node
    - camera_device
    - serial_port
    - baud_rate

Started nodes:
    - motor_controller
    - motor_input
    - panorama_stitcher
    - camera
    - image_republisher

Important topics:
    /motor_cmd:
        Servo angle command in degrees, from -135 to +135.

    /panorama_trigger:
        Trigger message used to start the panorama sweep.

    /image_raw:
        Raw image topic from the camera node.

    /arm_cam/image/compressed:
        Compressed image topic used by panorama_stitcher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------

    # Controls whether panorama_stitcher runs in test mode.
    # In test mode, it does not wait for real camera frames.
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Run panorama stitcher without requiring camera frames'
    )

    # Allows the camera device to be changed from the command line.
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    # Allows the serial port to be changed from the command line.
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino motor controller'
    )

    # Allows the baud rate to be changed from the command line.
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for Arduino serial communication'
    )

    # Stores the runtime value of each launch argument that are passed into the nodes.
    test_mode = LaunchConfiguration('test_mode')
    camera_device = LaunchConfiguration('camera_device')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')


    # -------------------------------------------------
    # Nodes
    # -------------------------------------------------

    # Motor controller node:
    # Subscribes to /motor_cmd and forwards mapped servo values to Arduino.
    motor_controller_node = Node(
        package='autonomy_vision',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate
        }]
    )

    # Motor input node:
    # Opens a separate terminal for user input.
    # The user can enter servo angles, 'p' for panorama, or 'e' to exit.
    motor_input_node = Node(
        package='autonomy_vision',
        executable='motor_input',
        name='motor_input',
        output='screen',
        prefix='gnome-terminal --'
    )

    # Camera node:
    # Starts the v4l2 camera node using the selected camera device.
    # This node publishes raw images, usually on /image_raw.
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{'video_device': camera_device}]
    )

    # Image republisher node:
    # Converts raw images from /image_raw into compressed images.
    # The panorama stitcher subscribes to /arm_cam/image/compressed.
    image_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/image_raw'),
            ('out/compressed', '/arm_cam/image/compressed')
        ]
    )


    # Panorama stitcher node:
    # Runs the panorama sweep logic.
    # It subscribes to /panorama_trigger, publishes servo commands to /motor_cmd,
    # and receives compressed camera frames from /arm_cam/image/compressed.
    # The test_mode parameter is passed in from the launch argument.
    panorama_stitcher_node = Node(
        package='autonomy_vision',
        executable='panorama_stitcher',
        name='panorama_stitcher',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[{'test_mode': test_mode}]
    )

    # -------------------------------------------------
    # Return launch description
    # -------------------------------------------------

    # The launch description includes the launch arguments first,
    # followed by all nodes that should be started. 
    return LaunchDescription([
        test_mode_arg,
        camera_device_arg,
        serial_port_arg,
        baud_rate_arg,
        motor_controller_node,
        motor_input_node,
        panorama_stitcher_node,
        camera_node,
        image_republisher_node,
    ])