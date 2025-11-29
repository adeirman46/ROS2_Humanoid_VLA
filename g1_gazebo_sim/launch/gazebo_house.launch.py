#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    pkg_g1_gazebo = get_package_share_directory('g1_gazebo_sim')
    pkg_g1_package = get_package_share_directory('g1_package')
    
    # Paths
    world_file = os.path.join(pkg_g1_gazebo, 'worlds', 'small_house.world')
    urdf_file = os.path.join(pkg_g1_package, 'urdf', 'g1_29dof.urdf')
    rviz_config = os.path.join(pkg_g1_gazebo, 'config', 'gazebo.rviz')
    
    # Model paths for GAZEBO_MODEL_PATH environment variable
    # Include both the AWS house models and the g1_package for meshes
    house_models_path = os.path.join(pkg_g1_gazebo, 'models')
    gazebo_model_path = house_models_path + ':' + pkg_g1_package
    \
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.5')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')
    gui = LaunchConfiguration('gui', default='true')
    rviz_arg = LaunchConfiguration('rviz', default='false')
    
    # Declare launch arguments
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
    
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Robot spawn yaw orientation'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz'
    )
    
    # Get URDF content via xacro
    robot_description_content = Command(['xacro ', urdf_file])
    
    # Gazebo server and client combined with proper GAZEBO_MODEL_PATH
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path}
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'g1_robot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz_arg)
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_gui,
        declare_rviz,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz_node
    ])
