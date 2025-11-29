#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import re


def process_urdf(context):
    """Convert package:// URIs to file:// URIs in URDF"""
    pkg_g1_package = get_package_share_directory('g1_package')
    urdf_file = os.path.join(pkg_g1_package, 'urdf', 'g1_29dof.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Replace package://g1_package with file:// absolute path
    urdf_content = urdf_content.replace(
        'package://g1_package',
        f'file://{pkg_g1_package}'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    
    # Robot state publisher with modified URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': use_sim_time == 'true'
        }],
        output='screen'
    )
    
    # Spawn robot - delayed to ensure Gazebo is ready
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'g1_robot',
                    '-topic', 'robot_description',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z
                ],
                output='screen'
            )
        ]
    )
    
    return [robot_state_publisher, spawn_robot]


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Robot spawn X position'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn Y position'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='Robot spawn Z position (height above ground)'
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # Process URDF and create nodes
    urdf_processor = OpaqueFunction(function=process_urdf)
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gazebo,
        urdf_processor
    ])
