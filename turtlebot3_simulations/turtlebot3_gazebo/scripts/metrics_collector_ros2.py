#!/usr/bin/env python3
# ROS 2 version of Metrics Collector

import rclpy
from rclpy.node import Node
import csv
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math
import time
from threading import Lock
import os

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.get_logger().info("Metrics Collector Node Initialized.")
        
        # Define the path for the CSV file (updated for ROS 2 workspace)
        csv_dir = os.path.expanduser('~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/results/mathstral')
        csv_path = os.path.join(csv_dir, 'mathstral_Env_3_1.csv')
        
        # Create the directory if it doesn't exist
        try:
            os.makedirs(csv_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Failed to create metrics CSV directory at {csv_dir}: {e}")
            return
        
        # Initialize CSV file
        try:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'User_Command',
                'Path_Planning_Time',
                'Execution_Time',
                'Waypoint_Generation_Success_Rate',
                'Path_Length',
                'Collision_Detection_Event',
                'Replanning_Rate'
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f"CSV file initialization failed: {e}")
            return
        
        # Thread lock for data synchronization
        self.lock = Lock()
        
        # Initialize metrics variables
        self.current_command = None
        self.command_start_time = None
        self.waypoint_received_time = None
        self.execution_start_time = None
        self.execution_end_time = None
        self.path_length = 0.0
        self.collision_events = 0
        self.replanning_attempts = 0
        self.successful_waypoint_generations = 0
        self.total_waypoint_generations = 0
        self.path_planning_time = 0.0
        
        # State variables
        self.current_waypoints = []
        self.previous_pose = None
        
        # Subscribe to necessary topics
        self.create_subscription(String, '/user_command', self.user_command_callback, 10)
        self.create_subscription(PoseArray, '/llm_waypoints', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(String, '/obstacle_data', self.obstacle_data_callback, 10)
        self.create_subscription(String, '/waypoint_generation_status', self.waypoint_generation_status_callback, 10)
        self.create_subscription(String, '/execution_status', self.execution_status_callback, 10)
    
    def user_command_callback(self, msg):
        with self.lock:
            self.current_command = msg.data.strip()
            self.command_start_time = time.time()
            self.total_waypoint_generations += 1
            self.get_logger().info(f"Received User Command: '{self.current_command}' at {self.command_start_time}")
    
    def waypoints_callback(self, msg):
        with self.lock:
            if not self.current_command:
                self.get_logger().warn("Waypoints received without an active user command.")
                return
            
            self.waypoint_received_time = time.time()
            self.path_planning_time = self.waypoint_received_time - self.command_start_time
            
            self.get_logger().info(f"Waypoints received for command '{self.current_command}' at {self.waypoint_received_time}")
            self.get_logger().info(f"Path Planning Time: {self.path_planning_time:.3f} seconds")
            
            self.current_waypoints = [(pose.position.x, pose.position.y) for pose in msg.poses]
            self.execution_start_time = time.time()
            self.get_logger().info(f"Execution started at {self.execution_start_time}")
            
            self.path_length = 0.0
    
    def waypoint_generation_status_callback(self, msg):
        with self.lock:
            status_message = msg.data.strip().lower()
            self.get_logger().info(f"Received waypoint generation status: '{status_message}'")
            
            try:
                status, attempts = status_message.split(':')
                attempts = int(attempts)
            except ValueError:
                self.get_logger().warn(f"Invalid waypoint generation status format: '{status_message}'")
                return
            
            if status == "success":
                self.successful_waypoint_generations += 1
                self.get_logger().info(f"Waypoint generation succeeded on attempt {attempts}.")
            elif status == "failure":
                self.get_logger().info(f"Waypoint generation failed after {attempts} attempts.")
    
    def execution_status_callback(self, msg):
        with self.lock:
            status = msg.data.strip().lower()
            if status == "completed":
                if self.execution_start_time is None:
                    return
                self.execution_end_time = time.time()
                execution_time = self.execution_end_time - self.execution_start_time
                self.get_logger().info(f"Execution Time: {execution_time:.3f} seconds")
                
                if self.total_waypoint_generations > 0:
                    success_rate = (self.successful_waypoint_generations / self.total_waypoint_generations) * 100
                else:
                    success_rate = 0.0
                
                replanning_rate = self.replanning_attempts / (self.execution_end_time - self.command_start_time) if (self.execution_end_time - self.command_start_time) > 0 else 0.0
                
                try:
                    self.csv_writer.writerow([
                        self.current_command,
                        f"{self.path_planning_time:.3f}",
                        f"{execution_time:.3f}",
                        f"{success_rate:.2f}%",
                        f"{self.path_length:.3f}",
                        f"{self.collision_events}",
                        f"{self.replanning_attempts}"
                    ])
                    self.csv_file.flush()
                    self.get_logger().info(f"Metrics for command '{self.current_command}' logged to CSV.")
                except Exception as e:
                    self.get_logger().error(f"Failed to write metrics to CSV: {e}")
                
                self.reset_metrics()
    
    def odom_callback(self, msg):
        with self.lock:
            current_pose = msg.pose.pose
            
            if self.previous_pose:
                dx = current_pose.position.x - self.previous_pose.position.x
                dy = current_pose.position.y - self.previous_pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                self.path_length += distance
            
            self.previous_pose = current_pose
    
    def scan_callback(self, msg):
        pass
    
    def obstacle_data_callback(self, msg):
        with self.lock:
            if msg.data.strip().lower() == "emergency_stop":
                self.replanning_attempts += 1
                self.collision_events += 1
                self.get_logger().info(f"Obstacle detected. Replanning attempts: {self.replanning_attempts}, Collision events: {self.collision_events}")
    
    def reset_metrics(self):
        self.current_command = None
        self.command_start_time = None
        self.waypoint_received_time = None
        self.execution_start_time = None
        self.execution_end_time = None
        self.path_length = 0.0
        self.collision_events = 0
        self.replanning_attempts = 0
        self.successful_waypoint_generations = 0
        self.total_waypoint_generations = 0
        self.current_waypoints = []
        self.previous_pose = None
        self.path_planning_time = 0.0
    
    def destroy_node(self):
        try:
            self.csv_file.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    collector = MetricsCollector()
    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
