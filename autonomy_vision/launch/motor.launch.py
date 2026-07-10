"""
ROS2 Launch file

Launches the motor controller and panorama capture nodes.

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
    - panorama_capture

Important topics:
    /motor_cmd:
        Servo angle command in degrees, from -135 to +135.

    /panorama_trigger:
        Trigger message used to start the panorama sweep.
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
        # output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate
        }]
    )

    '''
    # Motor input node:
    # Opens a separate terminal for user input.
    # The user can enter servo angles, 'p' for panorama, or 'e' to exit.
    motor_input_node = Node(
        package='autonomy_vision',
        executable='motor_input',
        # name='motor_input',
        # output='screen',
        prefix='gnome-terminal --'
    )
    '''



    # Panorama capture node:
    # Runs the panorama sweep logic.
    # It subscribes to /panorama_trigger, publishes servo commands to /motor_cmd,
    # and receives compressed camera frames from the ip camera.
    # The test_mode parameter is passed in from the launch argument.
    panorama_capture_node = Node(
        package='autonomy_vision',
        executable='panorama_capture',
        name='panorama_capture',
        # output='screen',
        # prefix='gnome-terminal --',
        parameters=[{'test_mode': test_mode}]
    )

    # -------------------------------------------------
    # Return launch description
    # -------------------------------------------------

    # The launch description includes the launch arguments first,
    # followed by all nodes that should be started. 
    return LaunchDescription([
        test_mode_arg,
        serial_port_arg,
        baud_rate_arg,
        motor_controller_node,
        panorama_capture_node,
    ])
