from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

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

        ## IMU (raw_data and magnetic field), GPS, Heading, Cardinal Compass
        Node(
            package='autonomy_sensors',
            executable='gps_imu_broadcaster',
            name='serial_imu_gps',
            output='screen',
        ),   
    ])
