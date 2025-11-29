#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_g1_package = get_package_share_directory('g1_package')
    pkg_g1_gazebo_sim = get_package_share_directory('g1_gazebo_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_g1_controller = get_package_share_directory('g1_controller')
    
    # Use the ros2_control URDF
    urdf_file = os.path.join(pkg_g1_package, 'urdf', 'g1_29dof_ros2_control.urdf')
    world_file = os.path.join(pkg_g1_gazebo_sim, 'worlds', 'small_house.world')
    
    # Process URDF - convert package:// to file://
    def process_urdf(context):
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
        
        # Replace package:// URIs with file:// URIs
        urdf_content = urdf_content.replace(
            'package://g1_package',
            f'file://{pkg_g1_package}'
        )
        
        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_content,
                'use_sim_time': True
            }],
            output='screen'
        )
        
        # Spawn robot in house
        spawn_robot = TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'g1_robot',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.75'
                    ],
                    output='screen'
                )
            ]
        )
        
        return [robot_state_publisher, spawn_robot]
    
    # Launch Gazebo with house world
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    # Process URDF function
    urdf_processor = OpaqueFunction(function=process_urdf)
    
    # Spawn controllers
    spawn_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
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
                arguments=['position_trajectory_controller'],
                output='screen'
            )
        ]
    )
    
    # GUI controller
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
        gazebo,
        urdf_processor,
        spawn_joint_state_broadcaster,
        spawn_position_controller,
        gui_controller
    ])
