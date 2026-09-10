#!/usr/bin/env python3
# ROS 2 Launch file for LLM path planning components - corridor_world_building_04 (department)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')
    
    # Environment YAML file for department
    environment_yaml = 'department.yaml'
    package_path = str(turtlebot3_gazebo_share)
    
    # Waypoint executor params
    waypoint_params = PathJoinSubstitution([
        turtlebot3_gazebo_share,
        'config',
        'waypoint_executor_params.yaml'
    ])
    
    # Environment Data Provider
    environment_data_provider = Node(
        package='turtlebot3_gazebo',
        executable='environment_data_provider_03_ros2.py',
        name='environment_data_provider',
        output='screen',
        parameters=[{
            'environment_yaml': environment_yaml,
            'package_path': package_path
        }]
    )
    
    # LLM Path Planner
    llm_path_planner = Node(
        package='turtlebot3_gazebo',
        executable='llm_path_planner_ros2.py',
        name='llm_path_planner',
        output='screen',
        parameters=[{
            'environment_yaml': environment_yaml,
            'environment_script': 'environment_03_ros2',
            'package_path': package_path,
            'safe_margin': 0.5,
            'waypoint_spacing': 0.7,
            'max_attempts': 3
        }]
    )
    
    # Object Marker Publisher
    object_marker_publisher = Node(
        package='turtlebot3_gazebo',
        executable='object_marker_publisher_ros2.py',
        name='object_marker_publisher',
        output='screen',
        parameters=[{
            'environment_yaml': str(PathJoinSubstitution([
                turtlebot3_gazebo_share,
                'object_list',
                'department.yaml'
            ]))
        }]
    )
    
    # Waypoint Executor
    waypoint_executor = Node(
        package='turtlebot3_gazebo',
        executable='waypoint_executor_ros2.py',
        name='waypoint_executor',
        output='screen',
        parameters=[
            waypoint_params,
            {
                'environment_yaml': str(PathJoinSubstitution([
                    turtlebot3_gazebo_share,
                    'object_list',
                    'department.yaml'
                ])),
                'linear_speed': 0.2,
                'angular_speed': 0.5,
                'distance_threshold': 0.2,
                'angle_threshold': 0.087,  # ~5 degrees in radians
                'obstacle_angle_range': 0.26,  # ~15 degrees in radians
                'self_obstacle_distance': 0.3,
                'critical_distance': 0.5,
                'max_replans': 3,
                'replan_cooldown': 5.0
            }
        ]
    )
    
    # Metrics Collector
    metrics_collector = Node(
        package='turtlebot3_gazebo',
        executable='metrics_collector_ros2.py',
        name='metrics_collector',
        output='screen'
    )
    
    return LaunchDescription([
        environment_data_provider,
        llm_path_planner,
        object_marker_publisher,
        waypoint_executor,
        metrics_collector
    ])
