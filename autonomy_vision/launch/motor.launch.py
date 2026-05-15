"""
ROS2 Launch file

Launch command: ros2 launch autonomy_vision motor.launch.py

Starts two nodes:
- motor_controller.py | Sends servo angle commands to Arduino over serial
- motor_input.py | Allows user to enter motor angles within range of -135 to +135 deg | Openned in new window
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomy_vision',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='autonomy_vision',
            executable='motor_input',
            name='motor_input',
            output='screen',
            prefix='gnome-terminal --'  # Open in new window
        ),
    ])