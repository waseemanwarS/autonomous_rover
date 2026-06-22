from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import subprocess
import tempfile

def generate_launch_description():

    namePackage = 'rover_description'
    pkg_share = get_package_share_directory(namePackage)
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    # Generate URDF
    try:
        subprocess.run(['xacro', xacro_file, '-o', urdf_file], check=True)
    except subprocess.CalledProcessError:
        print('[rover_description] Failed to generate URDF from xacro')
        with open(xacro_file, 'r') as f:
            xacro_content = f.read()
        robotDescription = xacro.process(xacro_content).toxml()
    else:
        with open(urdf_file, 'r') as f:
            robotDescription = f.read()

    # Create obstacle SDFs
    box_sdf = os.path.join(tempfile.gettempdir(), 'box.sdf')
    with open(box_sdf, 'w') as f:
        f.write('''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="box">
    <link name="link">
      <inertial>
        <mass>10.0</mass>
        <inertia><ixx>0.083</ixx><iyy>0.083</iyy><izz>0.083</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        <material><ambient>0.8 0.2 0.2 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>''')

    cylinder_sdf = os.path.join(tempfile.gettempdir(), 'cylinder.sdf')
    with open(cylinder_sdf, 'w') as f:
        f.write('''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="cylinder">
    <link name="link">
      <inertial>
        <mass>8.0</mass>
        <inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.1</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.3</radius><length>0.6</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.3</radius><length>0.6</length></cylinder></geometry>
        <material><ambient>0.8 0.8 0.2 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>''')

    sphere_sdf = os.path.join(tempfile.gettempdir(), 'sphere.sdf')
    with open(sphere_sdf, 'w') as f:
        f.write('''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="sphere">
    <link name="link">
      <inertial>
        <mass>5.0</mass>
        <inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.3</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.3</radius></sphere></geometry>
        <material><ambient>0.6 0.1 0.6 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>''')

    # Gazebo launch with empty.sdf
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': ['-r -v 3 empty.sdf'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn obstacles
    spawn_obstacles = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep 5 && '
            f'ros2 run ros_gz_sim create -file {box_sdf} -name red_box -x 1.5 -y 0 -z 0.5 && '
            f'ros2 run ros_gz_sim create -file {box_sdf} -name blue_box -x -1.5 -y 1.5 -z 0.5 && '
            f'ros2 run ros_gz_sim create -file {box_sdf} -name green_box -x 0 -y -1.5 -z 0.5 && '
            f'ros2 run ros_gz_sim create -file {cylinder_sdf} -name yellow_cylinder -x 2 -y 1.5 -z 0.5 && '
            f'ros2 run ros_gz_sim create -file {sphere_sdf} -name purple_sphere -x -2 -y 0 -z 0.5'
        ],
        output='screen'
    )

    # Spawn rover
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rover',  # UNIQUE NAME
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_rover',  # UNIQUE NAME
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_rover',  # UNIQUE NAME
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static transform for odom to base_footprint
    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_rover',  # UNIQUE NAME
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    # Bridge the lidar frame mismatch - UNIQUE NAME
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge_rover',  # UNIQUE NAME - FIXED!
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'rover/base_footprint/gpu_lidar'],
        parameters=[{'use_sim_time': True}]
    )

    # Bridge for scan and cmd_vel
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge_rover',  # UNIQUE NAME
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Odometry bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge_rover',  # UNIQUE NAME
        output='screen',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        parameters=[{'use_sim_time': True}]
    )

    # Clock bridge for sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge_rover',  # UNIQUE NAME
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}]
    )

    # EKF node for odometry
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_rover',  # UNIQUE NAME
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            'frequency': 30.0,
            'two_d_mode': True,
            
            'odom0': '/odom',
            'odom0_config': [True, True, False,
                            False, False, True,
                            False, False, False,
                            False, False, False],
            'odom0_differential': False,
            'odom0_relative': False,
            'publish_tf': True,
            'publish_acceleration': False,
        }]
    )

    return LaunchDescription([
        LogInfo(msg=['[rover_description] ====================================']),
        LogInfo(msg=['[rover_description] Starting Rover Simulation']),
        LogInfo(msg=['[rover_description] xacro: ', xacro_file]),
        LogInfo(msg=['[rover_description] ====================================']),

        gazeboLaunch,

        nodeRobotStatePublisher,

        joint_state_publisher,

        odom_to_base_footprint,
        lidar_frame_bridge,  # Now has unique name

        spawnModelNodeGazebo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawnModelNodeGazebo,
                on_exit=[LogInfo(msg='[rover_description] Rover spawned!')]
            )
        ),

        spawn_obstacles,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_obstacles,
                on_exit=[LogInfo(msg='[rover_description] Obstacles spawned!')]
            )
        ),

        scan_bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=scan_bridge,
                on_start=[LogInfo(msg='[rover_description] /scan bridge started')]
            )
        ),

        odom_bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=odom_bridge,
                on_start=[LogInfo(msg='[rover_description] /odom bridge started')]
            )
        ),

        clock_bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=clock_bridge,
                on_start=[LogInfo(msg='[rover_description] /clock bridge started')]
            )
        ),
        
        ekf_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ekf_node,
                on_start=[LogInfo(msg='[rover_description] EKF node started')]
            )
        ),
    ])