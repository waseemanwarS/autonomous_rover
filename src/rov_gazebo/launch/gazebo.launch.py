#!/usr/bin/env python3
# rov_gazebo/launch/gazebo.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_rov_gazebo = FindPackageShare('rov_gazebo').find('rov_gazebo')
    pkg_rov_description = FindPackageShare('rov_description').find('rov_description')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty.world')
    
    # Gazebo server
    gz_server = Node(
        package='gz_sim',
        executable='gz_sim',
        name='gz_sim',
        output='screen',
        arguments=[
            '-r',
            PathJoinSubstitution([pkg_rov_gazebo, 'worlds', world])
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ROS-GZ Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={PathJoinSubstitution([pkg_rov_gazebo, "config", "gz_bridge.yaml"])}'
        ],
        output='screen'
    )
    
    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rov_gazebo, 'launch', 'spawn_rover.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.world'),
        gz_server,
        gz_bridge,
        spawn_robot
    ])