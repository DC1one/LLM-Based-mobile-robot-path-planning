#!/usr/bin/env python3
# ROS 2 Launch file for corridor_01 world

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import GazeboLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Model type [burger, waffle, waffle_pi]'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.0')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.557343')
    
    # Get package paths
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')
    turtlebot3_description_share = FindPackageShare('turtlebot3_description')
    
    # World file path
    world_path = PathJoinSubstitution([
        turtlebot3_gazebo_share,
        'worlds',
        'corridor_01.world'
    ])
    
    # Map file path
    map_file = PathJoinSubstitution([
        turtlebot3_gazebo_share,
        'object_list',
        'map_01.yaml'
    ])
    
    # Robot description path
    robot_description_path = PathJoinSubstitution([
        turtlebot3_description_share,
        'urdf',
        'turtlebot3_burger.urdf.xacro'
    ])
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        GazeboLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'pause': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_corridor',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description')
        }],
        output='screen'
    )
    
    # Spawn TurtleBot3 in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', robot_description_path,
            '-entity_type', 'robot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )
    
    # Static transform: map -> odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file}],
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        static_tf,
        map_server
    ])
