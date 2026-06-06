from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    mode_arg = DeclareLaunchArgument('mode', default_value='fake',
        description='GPS mode: fake or real')
    foxglove_arg = DeclareLaunchArgument('foxglove', default_value='true',
        description='Launch Foxglove bridge')
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
        description='Serial port for real GPS')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='9600',
        description='Baud rate for real GPS')

    mode = LaunchConfiguration('mode')
    foxglove = LaunchConfiguration('foxglove')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')

    is_fake = IfCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    is_real = UnlessCondition(PythonExpression(["'", mode, "' == 'fake'"]))

    fake_gps_node = Node(
        package='gps_tracker',
        executable='fake_gps',
        name='fake_gps',
        output='screen',
        condition=is_fake
    )

    route_logger_node = Node(
        package='gps_tracker',
        executable='route_logger',
        name='route_logger',
        output='screen'
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(foxglove),
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        mode_arg, foxglove_arg, port_arg, baudrate_arg,
        fake_gps_node,
        route_logger_node,
        foxglove_bridge_node,
    ])
