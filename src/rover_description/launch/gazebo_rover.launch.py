from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import tempfile

def generate_rover_sdf(xacro_file):
    rover_sdf_file = os.path.join(tempfile.gettempdir(), 'rover.sdf')
    urdf_tmp = os.path.join(tempfile.gettempdir(), 'rover.urdf')
    
    urdf = subprocess.check_output(['xacro', xacro_file], text=True)
    with open(urdf_tmp, 'w', encoding='utf-8') as urdf_handle:
        urdf_handle.write(urdf)
        
    sdf_output = subprocess.check_output(
        ['gz', 'sdf', '-p', urdf_tmp],
        stderr=subprocess.STDOUT,
        text=True,
    )
    with open(rover_sdf_file, 'w', encoding='utf-8') as sdf_handle:
        sdf_handle.write(sdf_output)
        
    return rover_sdf_file
        
def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    
    try:
        rover_sdf_file = generate_rover_sdf(xacro_file)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(
            '[rover_description] Failed to generate SDF from xacro'
            f'Command output: {error.output}'
        ) from error
            
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )
    
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    spawn_delay = LaunchConfiguration('spawn_delay')
    
    gz_sim = ExecuteProcess(
    cmd=['gz', 'sim', '-r', '-v', gz_verbosity, '/home/waseemanwar/rover_ws/src/rover_description/worlds/rover_simple_world.sdf'],
    output='screen'
)
    
    spawn_rover = ExecuteProcess(
    cmd=[
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', 'rover_simple_world',  # Match the world name in SDF
        '-file', rover_sdf_file,
        '-name', 'rover',
        '-x', '0', '-y', '0', '-z', '0.5'
    ],
    output='screen',
)
    
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
    )
                                                    
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='3',
            description='Gazebo verbosity level'
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='1.0',
            description='Delay before spawning the robot in Gazebo',
        ),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        LogInfo(msg=['[rover_description] xacro: ', xacro_file]),
        LogInfo(msg=['[rover_description] generated_sdf: ', rover_sdf_file]),
        
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',    
            name='lidar_sensor_frame_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'lidar_link',
                'simple_robot/base_link/lidar_sensor',
            ],
        ),
        
        gz_sim,
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_sim,
                on_start=[LogInfo(msg='[rover_description] Gazebo started')]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_sim,
                on_exit=[LogInfo(msg='[rover_description] Gazebo exited')]
            )
        ),
        
        TimerAction(
            period=spawn_delay,
            actions=[
                LogInfo(msg='[rover_description] Spawning robot...'),
                spawn_rover
            ]
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_rover,
                on_exit=[LogInfo(msg='[rover_description] Spawn step finished. If /scan is empty, inspect gz topic -i -t /scan .')]
            )
        ),

        scan_bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=scan_bridge,
                on_start=[LogInfo(msg='[rover_description] ros_gz_bridge started for /scan')]
            )
        )
    ])