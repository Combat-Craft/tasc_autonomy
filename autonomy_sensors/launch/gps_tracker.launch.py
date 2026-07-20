from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='fake',
        description='GPS source: fake | ip | real')
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge (true/false)')
    port_arg = DeclareLaunchArgument(
        'port', default_value='',
        description='Serial port for ESP32 (empty = auto-detect)')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='Baud rate for ESP32')
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='true',
        description='Loop waypoints (true/false)')
    walk_step_arg = DeclareLaunchArgument(
        'walk_step_m', default_value='5.0',
        description='Step size in metres for ip mode walk')

    mode        = LaunchConfiguration('mode', )
    #foxglove    = LaunchConfiguration('foxglove')
    port        = LaunchConfiguration('port')
    baudrate    = LaunchConfiguration('baudrate')
    loop        = LaunchConfiguration('loop')
    walk_step_m = LaunchConfiguration('walk_step_m')

    is_fake = IfCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    is_ip   = IfCondition(PythonExpression(["'", mode, "' == 'ip'"]))
    is_real = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))

    # ── Nodes ───────────────────────────────────────────────────────────────

    fake_gps_node = Node(
        package='autonomy_sensors',
        executable='fake_gps',
        name='gps_publisher',
        output='screen',
        condition=is_fake,
        parameters=[{'loop': loop}]
    )

    ip_gps_node = Node(
        package='autonomy_sensors',
        executable='ip_gps',
        name='gps_publisher',
        output='screen',
        condition=is_ip,
        parameters=[{
            'walk_step_m': walk_step_m,
            'loop': loop,
        }]
    )

     
    real_gps_node = Node(
        package='autonomy_sensors',
        executable='gps_imu_broadcaster',
        name='gps_imu_broadcaster',
        output='screen',
        condition=is_real,
        parameters=[{
            'port': port,
            'baud': baudrate,
            'frame_id_imu': 'imu_link',
            'frame_id_gps': 'gps_link',
        }]
    )

    route_logger_node = Node(
        package='autonomy_sensors',
        executable='route_logger',
        name='route_logger',
    )

    #foxglove_bridge_node = Node(
    #    package='foxglove_bridge',
    #    executable='foxglove_bridge',
    #    name='foxglove_bridge',
    #    condition=IfCondition(foxglove),
    #    parameters=[{'port': 8765}]
    #)

    return LaunchDescription([
        mode_arg, foxglove_arg, port_arg, baudrate_arg,
        loop_arg, walk_step_arg,
        fake_gps_node,
        ip_gps_node,
        real_gps_node,
        route_logger_node,
        #foxglove_bridge_node,
    ])
