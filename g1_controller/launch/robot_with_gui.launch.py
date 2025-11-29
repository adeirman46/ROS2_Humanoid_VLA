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
    
    # Paths
    gazebo_robot_launch = os.path.join(pkg_g1_gazebo, 'launch', 'gazebo_robot_only.launch.py')
    
    # Include robot+gazebo launch
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_robot_launch)
    )
    
    # GUI controller (delayed to let Gazebo stabilize)
    gui_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='g1_controller',
                executable='gui_controller.py',
                name='gui_controller',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_sim,
        gui_controller
    ])
