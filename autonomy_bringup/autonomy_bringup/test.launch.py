import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_share = FindPackageShare(package='asimov_nav2test').find('asimov_nav2test')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    #ros2_controls
    robot_controllers = os.path.join(pkg_share, "config", "ros2controls_controllers.yaml")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    #ros2_control controller manager node ?
    # https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/doc/userdoc.rst
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    load_joint_state_controller = ExecuteProcess(
	    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
		'joint_broad'],
	    output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
	    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
		'diff_cont'],
	    output='screen'
    ) 
    
    spawn_entity = Node(
	package='gazebo_ros',
	executable='spawn_entity.py',
	arguments=['-entity', 'asimov_nav2test', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        RegisterEventHandler(
	    event_handler=OnProcessExit(
		target_action=spawn_entity,
		on_exit=[load_joint_state_controller],
	    )
	),
	RegisterEventHandler(
	    event_handler=OnProcessExit(
		target_action=load_joint_state_controller,
		on_exit=[load_diff_drive_base_controller],
	    )
	),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ])
