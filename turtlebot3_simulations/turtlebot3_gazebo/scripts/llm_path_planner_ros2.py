#!/usr/bin/env python3
# ROS 2 version of LLM Path Planner

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import tf2_ros
from tf2_ros import TransformListener, Buffer
import re
import yaml
import os
import importlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
import ollama
import json
from threading import Lock
import tf2_geometry_msgs

class LLMPathPlanner(Node):
    def __init__(self):
        super().__init__('llm_path_planner')
        self.get_logger().info("LLMPathPlanner initialized.")

        # TF Listener Initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.current_pose = None  # Will be updated from /odom
        self.environment_data = None  # Will be loaded from YAML file
        self.lock = Lock()

        # Initialize LLM Client
        try:
            self.llm_client = ollama.Client()
            self.get_logger().info("Connected to Ollama LLM client.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Ollama client: {e}")
            return

        # Publishers and Subscribers
        self.create_subscription(String, '/user_command', self.user_command_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/obstacle_data', self.obstacle_data_callback, 10)

        self.waypoints_pub = self.create_publisher(PoseArray, '/llm_waypoints', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_generation_status', 10)

        # Load parameters
        self.declare_parameter('safe_margin', 0.5)
        self.declare_parameter('waypoint_spacing', 0.7)
        self.declare_parameter('package_path', '')
        self.declare_parameter('environment_yaml', 'default.yaml')
        self.declare_parameter('environment_script', 'environment_01')
        self.declare_parameter('max_attempts', 3)

        self.safe_margin = self.get_parameter('safe_margin').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.package_path = self.get_parameter('package_path').value
        self.environment_yaml = self.get_parameter('environment_yaml').value
        self.environment_script = self.get_parameter('environment_script').value
        self.max_attempts = self.get_parameter('max_attempts').value

        # Store current target object
        self.current_target_object = None

        # Load environment data from YAML file
        yaml_file_path = os.path.join(self.package_path, 'object_list', self.environment_yaml)
        self.load_environment_data(yaml_file_path)

        # Dynamically load the environment module
        self.environment_module = self.load_environment_module(self.environment_script)

        self.get_logger().info("LLM Path Planner ready.")

    def load_environment_module(self, environment_script):
        try:
            utils_path = os.path.join(self.package_path, 'scripts', 'utils')
            if utils_path not in os.sys.path:
                os.sys.path.append(utils_path)
                self.get_logger().debug(f"Added '{utils_path}' to system path for dynamic imports.")

            environment_module = importlib.import_module(environment_script)
            self.get_logger().info(f"Successfully loaded environment module '{environment_script}'.")
            return environment_module
        except ImportError as e:
            self.get_logger().error(f"Failed to import environment module '{environment_script}': {e}")
            return None

    def load_environment_data(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                self.environment_data = yaml.safe_load(file)
            self.get_logger().debug(f"Loaded environment data from {yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load environment data from {yaml_file_path}: {e}")
            self.environment_data = None

    def odom_callback(self, data):
        if self.tf_listener is None:
            self.get_logger().error("TF Listener not initialized.")
            return

        try:
            # Wait for transform
            while not self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                if not rclpy.ok():
                    return

            odom_pose = PoseStamped()
            odom_pose.header = data.header
            odom_pose.pose = data.pose.pose

            # Transform the pose to map frame
            transformed_pose = self.tf_buffer.transform(odom_pose, "map", rclpy.duration.Duration(seconds=1.0))
            self.current_pose = transformed_pose

            self.get_logger().debug(f"Transformed Pose (map): ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})")
        except Exception as e:
            self.get_logger().error(f"TF Transformation error: {e}")
            self.current_pose = None

    def user_command_callback(self, data):
        with self.lock:
            user_command = data.data.strip()
            self.get_logger().info(f"Received user command: {user_command}")

            if self.current_pose is None or self.environment_data is None:
                self.get_logger().warn("Current robot pose or environment data is unknown. Waiting for data.")
                return

            # Extract target object name from the command
            target_object_name = self.extract_target_room(user_command)
            if not target_object_name:
                self.get_logger().error("Failed to extract target from the command.")
                self.status_pub.publish(String(data="failure:0"))
                return

            self.get_logger().info(f"Identified target object: {target_object_name}")

            # Find target object coordinates
            target_object = self.find_target_object(target_object_name)
            if not target_object:
                self.get_logger().error(f"Target object '{target_object_name}' not found in environment data.")
                self.status_pub.publish(String(data="failure:0"))
                return

            # Store current target object
            self.current_target_object = target_object

            # Initialize attempt counter
            attempt = 0
            waypoints = None

            while attempt < self.max_attempts and not waypoints:
                attempt += 1
                self.get_logger().info(f"Attempt {attempt} to generate waypoints.")
                waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                    llm_client=self.llm_client,
                    current_pose=self.current_pose,
                    target_object=target_object,
                    environment_data=self.environment_data,
                    safe_margin=self.safe_margin,
                    waypoint_spacing=self.waypoint_spacing
                )
                if waypoints:
                    self.get_logger().info(f"Successfully generated waypoints on attempt {attempt}.")
                    self.status_pub.publish(String(data=f"success:{attempt}"))
                    self.publish_waypoints(waypoints)
                else:
                    self.get_logger().warn(f"Waypoint generation failed on attempt {attempt}.")

            if not waypoints:
                self.get_logger().error("Failed to generate waypoints after maximum attempts.")
                self.status_pub.publish(String(data=f"failure:{attempt}"))

    def obstacle_data_callback(self, msg):
        with self.lock:
            replanning_request = msg.data.strip().lower()
            self.get_logger().info(f"LLM received replanning request: {replanning_request}")

            if replanning_request == "emergency_stop":
                self.get_logger().info("Processing replanning due to emergency stop.")

                if self.current_pose is None or self.environment_data is None:
                    self.get_logger().warn("Current robot pose or environment data is unknown. Cannot generate new waypoints.")
                    self.status_pub.publish(String(data="failure:0"))
                    return

                if self.current_target_object is None:
                    self.get_logger().warn("No current target object. Cannot generate new waypoints.")
                    self.status_pub.publish(String(data="failure:0"))
                    return

                attempt = 0
                waypoints = None

                while attempt < self.max_attempts and not waypoints:
                    attempt += 1
                    self.get_logger().info(f"Replanning Attempt {attempt} to generate new waypoints.")
                    waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                        llm_client=self.llm_client,
                        current_pose=self.current_pose,
                        target_object=self.current_target_object,
                        environment_data=self.environment_data,
                        safe_margin=self.safe_margin,
                        waypoint_spacing=self.waypoint_spacing
                    )
                    if waypoints:
                        self.get_logger().info(f"Successfully generated waypoints on replanning attempt {attempt}.")
                        self.status_pub.publish(String(data=f"success:{attempt}"))
                        self.publish_waypoints(waypoints)
                    else:
                        self.get_logger().warn(f"Replanning failed on attempt {attempt}.")

                if not waypoints:
                    self.get_logger().error("Failed to generate waypoints after maximum replanning attempts.")
                    self.status_pub.publish(String(data=f"failure:{attempt}"))
            else:
                self.get_logger().warn(f"Received unknown replanning request: {replanning_request}")

    def extract_target_room(self, command):
        patterns = [
            r"go to the ([\w_]+)",
            r"navigate to the ([\w_]+)",
            r"go to the ([\w\s_]+)",
            r"navigate to the ([\w\s_]+)",
            r"go to ([\w\s_]+)",
            r"navigate to ([\w\s_]+)",
            r"room number (\d+)",
            r"room (\d+)",
            r"main entrance",
            r"([\w\s_]+)",
        ]

        for pattern in patterns:
            match = re.search(pattern, command, re.IGNORECASE)
            if match:
                target = match.group(1).strip()
                normalized_target = target.lower().replace(' ', '_')
                if normalized_target.isdigit():
                    normalized_target = f"room_number_plate_{normalized_target}"
                self.get_logger().info(f"Extracted and normalized target object name: {normalized_target}")
                return normalized_target
        self.get_logger().warn("Failed to extract a target object from the command.")
        return None

    def find_target_object(self, target_object_name):
        self.get_logger().info(f"Searching for target object: {target_object_name}")
        for obj in self.environment_data.get('objects', []):
            obj_name = obj.get('name', '')
            pos = obj.get('position', {})
            if obj_name.lower() == target_object_name.lower():
                self.get_logger().info(f"Found target object: {obj_name} at position {pos}")
                position = {
                    'x': pos.get('x', 0.0),
                    'y': pos.get('y', 0.0),
                    'z': pos.get('z', 0.0)
                }
                return {'name': obj_name, 'position': position}
        self.get_logger().error(f"Target object '{target_object_name}' not found in environment data.")
        return None

    def publish_waypoints(self, waypoints):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for waypoint in waypoints:
            pose = Pose()
            pose.position.x = waypoint['x']
            pose.position.y = waypoint['y']
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            self.get_logger().info(f"Added waypoint: ({waypoint['x']}, {waypoint['y']})")

        self.waypoints_pub.publish(pose_array)
        self.get_logger().info(f"Published PoseArray with {len(pose_array.poses)} waypoints.")


def main(args=None):
    rclpy.init(args=args)
    planner = LLMPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
