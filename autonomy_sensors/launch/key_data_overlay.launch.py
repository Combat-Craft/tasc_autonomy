from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam_stream',
            executable='multi_camera_publisher',
            name='multi_camera_publisher',
            output='screen',
        ),

        #Node(
        #    package='cam_stream',
        #    executable='gps_node',
        #    name='gps_node',
        #    output='screen',
        #    parameters=[
        #        {'port': '/dev/ttyACM0'},
        #        {'baud_rate': 115200},
        #    ]
        #),
        # Add rosapi node
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
        ),

        # ROS Bridge websocket server
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
        # Sensors node
        Node(
            package='cam_stream',
            executable='sensors_node',
            name='sensors_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 115200}
            ]
        ),   

        Node(
            package='cam_stream',
            executable='distance_tracker',
            name='distance_tracker',
            output='screen',
        ),
        #
        Node(
            package='cam_stream',
            executable='heading',
            name='heading',
            output='screen',
        ),
    ])
