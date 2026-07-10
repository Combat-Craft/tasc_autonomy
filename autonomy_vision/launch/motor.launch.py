"""
ROS2 Launch File

Launches the non-interactive nodes:
    - motor_controller
    - panorama_capture

This launch file does not launch motor_input.py because motor_input.py uses keyboard input. 

Workflow:
    Terminal 1 — launch the non-interactive nodes:
        ros2 launch autonomy_vision motor.launch.py

    Terminal 2 — run the interactive input node:
        ros2 run autonomy_vision motor_input

    In motor_input.py, enter:
        - a servo angle (-120 to +120)
        - p,folder_name to trigger a named panorama sweep
        - e to exit

Launch commands:
    Normal mode with real IP camera snapshots:
        ros2 launch autonomy_vision motor.launch.py

    Test mode without requiring real camera images:
        ros2 launch autonomy_vision motor.launch.py test_mode:=true

Launch arguments:
    - test_mode
        Passed to panorama_capture.py.
        If true, panorama_capture simulates captures and does not require real camera snapshots.
        Default: False

    - serial_port
        Passed to motor_controller.py.
        Arduino serial device path.
        Default:/dev/ttyUSB0.

    - baud_rate
        Passed to motor_controller.py.
        Arduino serial baud rate.
        Default: 115200.

    - cam_url
        Passed to panorama_capture.py.
        IP camera snapshot URL used to capture JPEG images.
        Default: http://192.168.1.117:6688/snapshot/PROFILE_000

    - save_dir
        Passed to panorama_capture.py.
        Base folder where panorama sweep folders are saved.
        Default: ~/panorama_images.

Started nodes:
    - motor_controller
        Subscribes to /motor_cmd.
        Sends mapped servo commands to the Arduino over serial.

    - panorama_capture
        Subscribes to /panorama_trigger, /gps/fix, and /heading.
        Publishes servo angle commands to /motor_cmd.
        Captures IP camera snapshots and saves frame_*.jpg plus metadata.csv.

Interactive node not launched here (Used by user to input commands):
    - motor_input
        Run separately with:
            ros2 run autonomy_vision motor_input

        Publishes manual servo commands and panorama trigger messages.

Important topics:
    /motor_cmd

        Servo angle command in degrees.
        Expected range is -120 to +120.

        Published by:
            - motor_input.py for manual servo movement
            - panorama_capture.py during automated panorama sweeps

        Subscribed by:
            - motor_controller.py

    /panorama_trigger

        Trigger message used to start a panorama sweep.

        Published by:
            - motor_input.py

        Subscribed by:
            - panorama_capture.py

    /gps/fix
        Provides GPS latitude and longitude for panorama metadata.

        Subscribed by:
            - panorama_capture.py

    /heading

        Provides rover heading in degrees clockwise from north.
        Used by panorama_capture.py to calculate camera-facing heading:

            camera_heading = rover_heading + servo_angle

        Subscribed by:
            - panorama_capture.py

    /rosout
        Contains log messages from motor_controller and panorama_capture.

Output:
    panorama_capture.py saves each triggered sweep under save_dir.

    Example:
        ~/panorama_images/test_sweep_01/
            frame_000.jpg
            frame_001.jpg
            ...
            metadata.csv

Offline stitching:
    After a sweep is captured, run panorama_opencv_stitcher.py separately to create
    panorama.jpg from the saved frame_*.jpg images and metadata.csv.
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
