import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    pkg_rov_description = FindPackageShare('rov_description').find('rov_description')
    pkg_rov_gazebo = FindPackageShare('rov_gazebo').find('rov_gazebo')
    pkg_rov_navigation = FindPackageShare('rov_navigation').find('rov_navigation')

    # ==================== LAUNCH ARGUMENTS ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_gazebo = LaunchConfiguration('launch_gazebo', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    launch_rviz_slam = LaunchConfiguration('launch_rviz_slam', default='true')
    launch_ekf = LaunchConfiguration('launch_ekf', default='true')
    launch_navigation = LaunchConfiguration('launch_navigation', default='false')
    launch_slam = LaunchConfiguration('launch_slam', default='false')

    # NEW: chooses how Nav2 gets map->odom.
    #   'slam' (default) -> live slam_toolbox, no AMCL, no map_server.
    #                        Matches the existing nav2_group + slam_group below.
    #   'amcl'            -> static saved map via map_server + amcl,
    #                        slam_toolbox stays off even if launch_slam=true
    #                        is accidentally left set, since amcl_group takes
    #                        priority and slam_group is skipped in that mode.
    localization_mode = LaunchConfiguration('localization_mode', default='slam')

    # NEW: path to the saved map .yaml (only used when localization_mode=amcl)
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution([pkg_rov_navigation, 'maps', 'my_map.yaml'])
    )

    world = LaunchConfiguration('world', default='empty.world')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.5')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default='rover')

    map_file = LaunchConfiguration('map_file', default='')

    # ==================== PATHS ====================
    urdf_file = PathJoinSubstitution([
        pkg_rov_description, 'urdf', 'rover.urdf.xacro'
    ])

    gz_bridge_config = PathJoinSubstitution([
        pkg_rov_gazebo, 'config', 'gz_bridge.yaml'
    ])

    ekf_config = PathJoinSubstitution([
        pkg_rov_description, 'config', 'ekf.yaml'
    ])

    nav2_params_file = PathJoinSubstitution([
        pkg_rov_navigation, 'config', 'nav2_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_rov_description, 'rviz', 'urdf.rviz'
    ])

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_cmd = Command([
        'xacro', ' ', urdf_file,
        ' gazebo:=', launch_gazebo
    ])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # ==================== NODES ====================

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. Joint State Publisher (only when not in Gazebo)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(launch_gazebo)
    )

    # 3. RViz2 (main - for URDF visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz)
    )

    # 4. Gazebo Sim
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                PathJoinSubstitution([
                    pkg_rov_gazebo,
                    'worlds',
                    world
                ])
            ]
        }.items(),
        condition=IfCondition(launch_gazebo)
    )

    # 5. Spawn Robot
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rover',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_gazebo)
    )

    # 6. ROS-GZ Bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': gz_bridge_config}
        ],
        output='screen',
        condition=IfCondition(launch_gazebo)
    )

    # 7. EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(launch_ekf)
    )

    # 8. Navigation Group (planner/controller/costmaps only - used by BOTH
    #    modes, since map->odom is supplied differently depending on mode
    #    but the planner/controller stack itself is identical either way)
    nav2_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'map': map_file,
                'autostart': 'true'
            }.items()
        )
    ], condition=IfCondition(launch_navigation))

    # 9. SLAM Group - only runs when launch_slam=true AND localization_mode=slam.
    #    Guards against both being set inconsistently (e.g. launch_slam=true
    #    left over from a previous run while testing amcl mode).
    slam_group = GroupAction([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_rov_navigation, 'config', 'slam_params.yaml']),
                {'use_sim_time': use_sim_time}
            ]
        ),
        # NEW: slam_toolbox in this install is lifecycle-managed and does
        # NOT self-activate - it was found stuck in 'unconfigured' state
        # indefinitely without this, requiring manual
        # 'ros2 lifecycle set /slam_toolbox configure/activate' calls.
        # This lifecycle_manager handles that automatically on launch,
        # the same way amcl_group's lifecycle_manager_localization does
        # for map_server/amcl.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['slam_toolbox']
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg_rov_navigation, 'rviz', 'slam.rviz'])],
            condition=IfCondition(launch_rviz_slam)
        )
    ], condition=IfCondition(PythonExpression([
        "'", launch_slam, "' == 'true' and '", localization_mode, "' == 'slam'"
    ])))

    # 10. AMCL Group - static map_server + amcl, only runs when
    #     localization_mode=amcl. This is the nav2_bringup pair that
    #     navigation_launch.py (used above) deliberately omits, since
    #     navigation_launch.py assumes something else (here: amcl, or in
    #     slam mode: slam_toolbox) is already providing map->odom.
    amcl_group = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_rov_navigation, 'config', 'amcl_params.yaml']),
                {'use_sim_time': use_sim_time}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ], condition=IfCondition(PythonExpression([
        "'", localization_mode, "' == 'amcl'"
    ])))

    # ==================== LAUNCH DESCRIPTION ====================
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_gazebo', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_rviz_slam', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_ekf', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_navigation', default_value='false'))
    ld.add_action(DeclareLaunchArgument('launch_slam', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'localization_mode', default_value='slam',
        description="How Nav2 gets map->odom: 'slam' (live slam_toolbox) or 'amcl' (static saved map)"
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_yaml_file',
        default_value=PathJoinSubstitution([pkg_rov_navigation, 'maps', 'my_map.yaml']),
        description='Path to saved map .yaml, only used when localization_mode=amcl'
    ))
    ld.add_action(DeclareLaunchArgument('world', default_value='empty.world'))
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_z', default_value='0.5'))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('robot_name', default_value='rover'))
    ld.add_action(DeclareLaunchArgument('map_file', default_value=''))

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(gz_server)
    ld.add_action(spawn_robot_node)
    ld.add_action(gz_bridge_node)
    ld.add_action(ekf_node)
    ld.add_action(nav2_group)
    ld.add_action(slam_group)
    ld.add_action(amcl_group)

    return ld