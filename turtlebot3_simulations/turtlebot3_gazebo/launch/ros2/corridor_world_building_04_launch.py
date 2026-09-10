# ROS 2 Launch File for corridor_world_building_04
# Ubuntu 26.04 Compatible - ROS 2 Jazzy/Rolling

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Model type [burger, waffle, waffle_pi]'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='-3.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Z position')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0', description='Roll angle')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch angle')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.557343', description='Yaw angle')
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    
    # Gazebo empty world launch with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(turtlebot3_gazebo_dir, 'worlds', 'department.world'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )
    
    # Robot description - use xacro to process URDF
    robot_description_path = os.path.join(
        turtlebot3_gazebo_dir, 
        'urdf', 
        'turtlebot3_burger.urdf.xacro'
    )
    
    # Robot State Publisher with processed URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_corridor',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'urdf',
                'turtlebot3_burger.urdf.xacro'
            ])
        }],
        remappings=[('/robot_description', '/robot_description')]
    )
    
    # Spawn TurtleBot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_burger',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ]
    )
    
    # Static Transform Publisher: map -> odom
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Map Server - Updated path for ROS 2 workspace
    map_file_path = os.path.join(turtlebot3_gazebo_dir, 'object_list', 'map_03.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )
    
    # Lifecycle node to activate the map server (required for nav2_map_server)
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    return LaunchDescription([
        model_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        static_tf_node,
        map_server_node,
        lifecycle_node
    ])
