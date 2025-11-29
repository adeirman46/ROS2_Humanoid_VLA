#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_g1_gazebo = get_package_share_directory('g1_gazebo_sim')
    pkg_g1_controller = get_package_share_directory('g1_controller')
    
    # Paths
    gazebo_robot_launch = os.path.join(pkg_g1_gazebo, 'launch', 'gazebo_robot_ros2_control.launch.py')
    controller_config = os.path.join(pkg_g1_controller, 'config', 'g1_controllers.yaml')
    
    # Include robot+gazebo launch with ros2_control
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_robot_launch)
    )
    
    # Spawn controllers using spawner nodes (delayed)
    spawn_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    spawn_position_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['position_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # GUI controller (delayed even more)
    gui_controller = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='g1_controller',
                executable='ros2_control_gui.py',
                name='ros2_control_gui',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_sim,
        spawn_joint_state_broadcaster,
        spawn_position_controller,
        gui_controller
    ])
