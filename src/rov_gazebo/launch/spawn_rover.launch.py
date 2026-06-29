#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_rov_description = FindPackageShare(
        'rov_description').find('rov_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    urdf_file = PathJoinSubstitution([
        pkg_rov_description,
        'urdf',
        'rover.urdf.xacro'
    ])

    robot_description_content = f'$(xacro {urdf_file} gazebo:=true)'

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rover',
        output='screen',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.6'
        ),

        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0'
        ),

        spawn_robot
    ])