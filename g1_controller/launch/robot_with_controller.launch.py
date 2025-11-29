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
    
    # Launch arguments
    declare_controller_type = DeclareLaunchArgument(
        'controller',
        default_value='stand',
        description='Controller type: stand or zero'
    )
    
    # Include robot+gazebo launch
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_robot_launch)
    )
    
    # Stand controller (delayed to let Gazebo stabilize) - always launch
    stand_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='g1_controller',
                executable='stand_controller.py',
                name='stand_controller',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_controller_type,
        robot_sim,
        stand_controller
    ])
