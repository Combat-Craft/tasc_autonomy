#!/usr/bin/env python3
"""
Launch file for C++ GPS stack

Two modes:
1. FAKE - Use hardcoded GPS data (gps_publisher) + route_logger
2. REAL - Use real GPS (gps_broadcaster) + route_logger + Foxglove

Usage:
    ros2 launch gps_tracker gps_tracker.launch.py mode:=fake
    ros2 launch gps_tracker gps_tracker.launch.py mode:=real
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    
    # ================================================================
    # Arguments
    # ================================================================
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='fake',
        description='GPS mode: fake (hardcoded) or real (serial hardware)'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for real GPS'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Baud rate for real GPS'
    )
    
    foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Launch Foxglove bridge'
    )
    
    # ================================================================
    # Configs
    # ================================================================
    
    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    foxglove = LaunchConfiguration('foxglove')
    
    # ================================================================
    # FAKE GPS MODE (Testing without hardware)
    # ================================================================
    
    fake_gps_node = Node(
        package='gps_tracker',
        executable='fake_gps',
        name='fake_gps',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    )
    
    # ================================================================
    # REAL GPS MODE (Hardware serial)
    # ================================================================
    
    # Note: gps_broadcaster is a separate node for real hardware.
    # To use real GPS, add a gps_broadcaster executable to this package
    # or install the nmea_navsat_driver package:
    #   sudo apt install ros-$ROS_DISTRO-nmea-navsat-driver
    real_gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_broadcaster',
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'fake'"])),
        parameters=[
            {'port': port},
            {'baud': baudrate},
        ]
    )
    
    # ================================================================
    # ROUTE LOGGER (Converts lat/lon → local XY + creates Path)
    # ================================================================
    
    route_logger_node = Node(
        package='gps_tracker',
        executable='route_logger',
        name='route_logger',
        output='screen'
    )
    
    # ================================================================
    # FOXGLOVE BRIDGE (Visualization)
    # ================================================================
    
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(foxglove),
        parameters=[
            {'port': 8765},
        ]
    )
    
    # ================================================================
    # Status Messages
    # ================================================================
    
    fake_mode_msg = LogInfo(
        msg=[
            '\n',
            '╔═══════════════════════════════════════════════════════╗\n',
            '║  🎮 FAKE GPS MODE (Testing without hardware)          ║\n',
            '╠═══════════════════════════════════════════════════════╣\n',
            '║  Publishing hardcoded waypoints to /gps/fix           ║\n',
            '║  route_logger converting to /gps/path                 ║\n',
            '║                                                       ║\n',
            '║  Topics:                                              ║\n',
            '║    📍 /gps/fix  → Hardcoded Toronto coordinates      ║\n',
            '║    🛤️  /gps/path → Local XY coordinates             ║\n',
            '║                                                       ║\n',
            '║  Foxglove: https://foxglove.dev/studio               ║\n',
            '║  Connect:  ws://localhost:8765                        ║\n',
            '║                                                       ║\n',
            '║  Add panels:                                          ║\n',
            '║    - 3D panel (topic: /gps/path)                     ║\n',
            '║    - Plot (showing X/Y coordinates)                  ║\n',
            '╚═══════════════════════════════════════════════════════╝\n',
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    )
    
    real_mode_msg = LogInfo(
        msg=[
            '\n',
            '╔═══════════════════════════════════════════════════════╗\n',
            '║  🛰️  REAL GPS MODE (Hardware serial)                 ║\n',
            '╠═══════════════════════════════════════════════════════╣\n',
            '║  Reading NMEA from serial port                        ║\n',
            '║  route_logger converting to local coordinates         ║\n',
            '║                                                       ║\n',
            '║  Topics:                                              ║\n',
            '║    📍 /gps/fix  → From serial hardware               ║\n',
            '║    🛤️  /gps/path → Local XY trajectory              ║\n',
            '║                                                       ║\n',
            '║  Foxglove: https://foxglove.dev/studio               ║\n',
            '║  Connect:  ws://localhost:8765                        ║\n',
            '╚═══════════════════════════════════════════════════════╝\n',
        ],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    )
    
    # ================================================================
    # Build LaunchDescription
    # ================================================================
    
    ld = LaunchDescription([
        mode_arg,
        port_arg,
        baudrate_arg,
        foxglove_arg,
        
        fake_mode_msg,
        real_mode_msg,
        
        fake_gps_node,
        real_gps_node,
        route_logger_node,
        foxglove_bridge_node,
    ])
    
    return ld