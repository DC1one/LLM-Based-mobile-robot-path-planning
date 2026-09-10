#!/usr/bin/env python3
# ROS 2 version of Waypoint Executor

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, PoseArray, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer
import math
import json
import time
import os

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        
        # Declare parameters
        self.declare_parameter('environment_yaml', '')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance_threshold', 0.2)
        self.declare_parameter('angle_threshold', math.radians(5))
        self.declare_parameter('obstacle_angle_range', math.radians(15))
        self.declare_parameter('self_obstacle_distance', 0.3)
        self.declare_parameter('critical_distance', 0.5)
        self.declare_parameter('max_replans', 3)
        self.declare_parameter('replan_cooldown', 5.0)
        
        # Get parameters
        environment_yaml_path = self.get_parameter('environment_yaml').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.obstacle_angle_range = self.get_parameter('obstacle_angle_range').value
        self.self_obstacle_distance = self.get_parameter('self_obstacle_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_replans = self.get_parameter('max_replans').value
        self.replan_cooldown = self.get_parameter('replan_cooldown').value
        
        # Load environment data
        self.load_environment_data(environment_yaml_path)
        
        # State definitions
        self.STATE_NAVIGATING = 'Navigating'
        self.STATE_EMERGENCY_STOPPED = 'Emergency Stopped'
        self.STATE_REPLANNING = 'Replanning'
        self.STATE_RESUMING_NAVIGATION = 'Resuming Navigation'
        
        # Initial state
        self.state = self.STATE_NAVIGATING
        self.emergency_stop = False
        self.replan_attempts = 0
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers and Publishers
        self.waypoints_sub = self.create_subscription(PoseArray, '/llm_waypoints', self.waypoints_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_data_pub = self.create_publisher(String, '/obstacle_data', 10)
        self.execution_status_pub = self.create_publisher(String, '/execution_status', 10)
        
        # State variables
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.last_replan_time = 0
        self.last_waypoint_time = self.get_clock().now()
        
        # Timer for continuous waypoint processing (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Waypoint Executor initialized.")
    
    def load_environment_data(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                self.environment_data = yaml.safe_load(file)
            self.get_logger().info(f"Loaded environment data from {yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load environment data: {e}")
            self.environment_data = None
    
    def get_obstacle_boundaries(self):
        if not self.environment_data:
            self.get_logger().warn("No environment data loaded. Unable to retrieve obstacle boundaries.")
            return []
        
        obstacles = [obj for obj in self.environment_data.get('objects', []) if 'Obstacle' in obj.get('name', '')]
        obstacle_boundaries = []
        for obstacle in obstacles:
            pos = obstacle.get('position', {})
            size = obstacle.get('size', {})
            obstacle_boundaries.append({
                'x_min': pos.get('x', 0.0) - size.get('width', 1.0) / 2,
                'x_max': pos.get('x', 0.0) + size.get('width', 1.0) / 2,
                'y_min': pos.get('y', 0.0) - size.get('length', 1.0) / 2,
                'y_max': pos.get('y', 0.0) + size.get('length', 1.0) / 2
            })
        return obstacle_boundaries
    
    def waypoints_callback(self, pose_array):
        self.get_logger().info("Received new waypoints.")
        
        valid_waypoints = []
        obstacle_boundaries = self.get_obstacle_boundaries()
        
        for pose in pose_array.poses:
            waypoint_pose = PoseStamped()
            waypoint_pose.header = pose_array.header
            waypoint_pose.pose = pose
            
            try:
                # Transform waypoint to odom frame
                transformed_pose = self.tf_buffer.transform(waypoint_pose, "odom", rclpy.duration.Duration(seconds=1.0))
                waypoint_x = transformed_pose.pose.position.x
                waypoint_y = transformed_pose.pose.position.y
                
                # Validate waypoint distance from all obstacles
                is_valid = True
                for boundary in obstacle_boundaries:
                    if (boundary['x_min'] <= waypoint_x <= boundary['x_max'] and
                        boundary['y_min'] <= waypoint_y <= boundary['y_max']):
                        self.get_logger().warn(f"Waypoint ({waypoint_x:.2f}, {waypoint_y:.2f}) is inside an obstacle boundary. Ignoring.")
                        is_valid = False
                        break
                
                if is_valid:
                    valid_waypoints.append((waypoint_x, waypoint_y))
                    self.get_logger().info(f"Transformed and validated waypoint: ({waypoint_x:.2f}, {waypoint_y:.2f})")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to transform waypoint: {e}")
                continue
        
        if not valid_waypoints:
            self.get_logger().error("No valid waypoints received after validation.")
            return
        
        self.waypoints = valid_waypoints
        self.current_waypoint_index = 0
        self.get_logger().info(f"Waypoints to navigate: {self.waypoints}")
        self.last_waypoint_time = self.get_clock().now()
        
        if self.state in [self.STATE_EMERGENCY_STOPPED, self.STATE_RESUMING_NAVIGATION]:
            self.get_logger().info("New safe waypoints received. Preparing to resume navigation.")
            self.state = self.STATE_RESUMING_NAVIGATION
            self.replan_attempts = 0
    
    def odom_callback(self, data):
        self.current_pose = data.pose.pose
    
    def scan_callback(self, data):
        if self.current_pose is None:
            return
        
        relevant_distances = []
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        num_ranges = len(data.ranges)
        
        for i in range(num_ranges):
            distance = data.ranges[i]
            if 0.0 < distance < self.critical_distance:
                angle = angle_min + i * angle_increment
                
                if abs(angle) > self.obstacle_angle_range:
                    continue
                
                if distance < self.self_obstacle_distance:
                    continue
                
                relevant_distances.append(distance)
        
        obstacle_detected = False
        if len(relevant_distances) >= 3:
            relevant_distances_sorted = sorted(relevant_distances)
            median_distance = relevant_distances_sorted[len(relevant_distances_sorted) // 2]
            if median_distance < self.critical_distance:
                obstacle_detected = True
        elif len(relevant_distances) > 0:
            min_distance = min(relevant_distances)
            if min_distance < self.critical_distance:
                obstacle_detected = True
        
        current_time = time.time()
        
        if obstacle_detected:
            if self.state == self.STATE_NAVIGATING and self.replan_attempts < self.max_replans:
                if (current_time - self.last_replan_time) > self.replan_cooldown:
                    self.get_logger().warn("Emergency Stop! Obstacle detected within critical distance.")
                    self.emergency_stop = True
                    self.state = self.STATE_EMERGENCY_STOPPED
                    self.stop_robot()
                    
                    obstacle_data_msg = String()
                    obstacle_data_msg.data = "emergency_stop"
                    self.obstacle_data_pub.publish(obstacle_data_msg)
                    
                    self.get_logger().info("Published 'emergency_stop' to /obstacle_data for replanning.")
                    self.last_replan_time = current_time
                    self.replan_attempts += 1
        else:
            if self.state == self.STATE_RESUMING_NAVIGATION:
                self.get_logger().info("Successfully avoided the obstacle. Resuming navigation.")
                self.state = self.STATE_NAVIGATING
    
    def timer_callback(self):
        if self.current_pose is None:
            self.stop_robot()
            self.get_logger().warn("Current pose is unknown. Awaiting pose data.")
            return
        
        if self.state in [self.STATE_EMERGENCY_STOPPED, self.STATE_REPLANNING]:
            self.stop_robot()
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            self.publish_execution_completion()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        
        yaw = self.get_yaw()
        desired_angle = math.atan2(dy, dx)
        angle = desired_angle - yaw
        
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        
        twist = Twist()
        
        if abs(angle) > self.angle_threshold:
            twist.angular.z = self.angular_speed * angle
            twist.linear.x = 0.0
            self.get_logger().debug(f"Rotating with angular velocity: {twist.angular.z:.3f} rad/s")
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * angle
            self.get_logger().debug(f"Moving forward with linear velocity: {twist.linear.x:.3f} m/s")
        
        self.cmd_vel_pub.publish(twist)
        
        if distance < self.distance_threshold:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached at ({target_x:.2f}, {target_y:.2f}).")
            self.current_waypoint_index += 1
            time.sleep(0.1)
    
    def get_yaw(self):
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = self.euler_from_quaternion(orientation_list)
        return yaw
    
    def euler_from_quaternion(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        import math
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(2.0 * (q.w * q.y - q.x * q.z))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return roll, pitch, yaw
    
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def publish_execution_completion(self):
        completion_msg = String()
        completion_msg.data = "completed"
        self.execution_status_pub.publish(completion_msg)


def main(args=None):
    rclpy.init(args=args)
    executor_node = WaypointExecutor()
    rclpy.spin(executor_node)
    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
