#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_g1_package = get_package_share_directory('g1_package')
    pkg_g1_controller = get_package_share_directory('g1_controller')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Use the ros2_control URDF
    urdf_file = os.path.join(pkg_g1_package, 'urdf', 'g1_29dof_ros2_control.urdf')
    
    # Process URDF - convert package:// to file://
    def process_urdf(context):
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
        
        # Replace package:// URIs with file:// URIs
        urdf_content = urdf_content.replace(
            'package://g1_package',
            f'file://{pkg_g1_package}'
        )
        
        # Fix the controller config path - replace ROS1 $(find) with actual path
        controller_config_path = os.path.join(pkg_g1_controller, 'config', 'g1_controllers.yaml')
        urdf_content = urdf_content.replace(
            '$(find g1_controller)/config/g1_controllers.yaml',
            controller_config_path
        )
        
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
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
        
        # Spawn robot
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
                        '-z', '0.75'  # Adjusted height for standing pose
                    ],
                    output='screen'
                )
            ]
        )
        
        return [robot_state_publisher, spawn_robot]
    
    #  Launch Gazebo
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # Process URDF function
    urdf_processor = OpaqueFunction(function=process_urdf)
    
    return LaunchDescription([
        gazebo,
        urdf_processor
    ])
