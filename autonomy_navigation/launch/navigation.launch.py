import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # share autonomy navigation directory
    pkg_nav = get_package_share_directory('autonomy_navigation')

    # grab configs
    ekf_config = os.path.join(pkg_nav, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_nav, 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        Node(
            package = 'robot_localization',
            executable = 'ekf_node',
            name = 'ekf_global_node',
            output = 'screen',
            parameters = [ekf_config],
            remappings = [('odometry/filtered', 'odometry/global')],
        ),
        Node ( 
            package = 'robot_localization',
            executable = 'navsat_transform_node',
            name = 'navsat_transform_node',
            output = 'screen',
            parameters = [navsat_config],
            remappings=[('imu', '/imu/acc_gyro'), ('gps/fix', '/fix'), ('odometry/filtered', 'odometry/global')]
        )
    ])
