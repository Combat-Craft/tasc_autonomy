"""
Motor + panorama launch file.

IMPORTANT: this does NOT call the standalone 'panorama_stitcher' executable.
That was the old design (panorama_stitcher.launch.py) Real pipeline (per autonomy_vision's
motor.launch.py) is two nodes started together:
    - motor_controller   (subscribes /motor_cmd, drives Arduino servo)
    - panorama_capture    (subscribes /panorama_trigger, /gps/fix, /heading;
                            publishes /motor_cmd during automated sweeps)

motor_input.py (manual keyboard control) is intentionally NOT launched here
per autonomy_vision's own docstring, it's meant to be run by hand in a
separate terminal (`ros2 run autonomy_vision motor_input`), since it reads
interactive keyboard input and isn't something a bringup script should own.

Depends on /gps/fix and /heading being published - i.e. imu_gps.launch.py
must be running with enable_heading:=true (the combined gps_imu_broadcaster
path), or panorama_capture's subscriptions will just sit empty. Not a problem 
since in topic_check and exploration imu_gps will be launched.

Args:
  enable_panorama_stitcher (bool, default true) - master on/off for this file
  test_mode    (bool,   default false) - passed through to panorama_capture;
                                          simulates captures, no real camera needed
  serial_port  (string, default /dev/ttyUSB0)          - passed through to motor_controller
  baud_rate    (string, default 115200)                - passed through to motor_controller
  cam_url      (string, default http://192.168.1.117:6688/snapshot/PROFILE_000)
                                                          - passed through to panorama_capture
  save_dir     (string, default ~/panorama_images)      - passed through to panorama_capture

Used by: exploration, test_all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ADJUST: verify this matches your actual repo
VISION_PACKAGE = 'autonomy_vision'


def generate_launch_description():
    enable_panorama_arg = DeclareLaunchArgument(
        'enable_panorama_stitcher', default_value='true',
        description='Launch motor_controller + panorama_capture'
    )
    test_mode_arg = DeclareLaunchArgument(
        'test_mode', default_value='false',
        description='Passed to panorama_capture - simulate captures, no real camera needed'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Passed to motor_controller - Arduino serial device path'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Passed to motor_controller - Arduino serial baud rate'
    )
    cam_url_arg = DeclareLaunchArgument(
        'cam_url', default_value='http://192.168.1.117:6688/snapshot/PROFILE_000',
        description='Passed to panorama_capture - IP camera snapshot URL'
    )
    save_dir_arg = DeclareLaunchArgument(
        'save_dir', default_value='~/panorama_images',
        description='Passed to panorama_capture - base folder for saved sweeps'
    )

    motor_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(VISION_PACKAGE),
                'launch', 'motor.launch.py',
            ])
        ),
        launch_arguments={
            'test_mode': LaunchConfiguration('test_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'cam_url': LaunchConfiguration('cam_url'),
            'save_dir': LaunchConfiguration('save_dir'),
        }.items(),
    )

    panorama_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_panorama_stitcher')),
        actions=[motor_launch_include],
    )

    return LaunchDescription([
        enable_panorama_arg,
        test_mode_arg,
        serial_port_arg,
        baud_rate_arg,
        cam_url_arg,
        save_dir_arg,
        panorama_group,
    ])