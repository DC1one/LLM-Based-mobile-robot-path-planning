#!/usr/bin/env python3

import rospy
import tf
import re
import yaml
import os
import importlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
import ollama
import json  # Ensure json is imported for parsing
from threading import Lock  # For thread safety

class LLMPathPlanner:
    def __init__(self):
        rospy.init_node('llm_path_planner', anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("LLMPathPlanner initialized.")

        # TF Listener Initialization
        self.tf_listener = tf.TransformListener()

        # Robot state
        self.current_pose = None  # Will be updated from /odom
        self.environment_data = None  # Will be loaded from YAML file
        self.lock = Lock()

        # Initialize LLM Client
        try:
            self.llm_client = ollama.Client()
            rospy.loginfo("Connected to Ollama LLM client.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Ollama client: {e}")
            rospy.signal_shutdown("LLM client initialization failed.")

        # Publishers and Subscribers
        rospy.Subscriber('/user_command', String, self.user_command_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/obstacle_data', String, self.obstacle_data_callback)  # Subscriber for replanning

        self.waypoints_pub = rospy.Publisher('/llm_waypoints', PoseArray, queue_size=10)

        # Publisher for Waypoint Generation Status
        self.status_pub = rospy.Publisher('/waypoint_generation_status', String, queue_size=10)

        # Load parameters
        self.safe_margin = rospy.get_param('~safe_margin', 0.5)
        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 0.7)
        self.package_path = rospy.get_param('~package_path', '')
        self.environment_yaml = rospy.get_param('~environment_yaml', 'default.yaml')
        self.environment_script = rospy.get_param('~environment_script', 'environment_01')  # Default to environment_01
        self.max_attempts = rospy.get_param('~max_attempts', 3)  # Maximum attempts for waypoint generation

        # Store current target object
        self.current_target_object = None



        # Load environment data from YAML file
        yaml_file_path = os.path.join(self.package_path, 'object_list', self.environment_yaml)
        self.load_environment_data(yaml_file_path)

        # Dynamically load the environment module
        self.environment_module = self.load_environment_module(self.environment_script)

        rospy.loginfo("LLM Path Planner ready.")

    def load_environment_module(self, environment_script):
        try:
            # Construct the full path to the environment script
            utils_path = os.path.join(self.package_path, 'scripts', 'utils')
            if utils_path not in os.sys.path:
                os.sys.path.append(utils_path)
                rospy.logdebug(f"Added '{utils_path}' to system path for dynamic imports.")

            # Import the module dynamically
            environment_module = importlib.import_module(environment_script)
            rospy.loginfo(f"Successfully loaded environment module '{environment_script}'.")
            return environment_module
        except ImportError as e:
            rospy.logerr(f"Failed to import environment module '{environment_script}': {e}")
            rospy.signal_shutdown("Environment module import failed.")
            return None

    def load_environment_data(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                self.environment_data = yaml.safe_load(file)
            rospy.logdebug(f"Loaded environment data from {yaml_file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load environment data from {yaml_file_path}: {e}")
            self.environment_data = None

    def odom_callback(self, data):
        rospy.logdebug("odom_callback invoked.")
        if not hasattr(self, 'tf_listener') or self.tf_listener is None:
            rospy.logerr("TF Listener not initialized.")
            return

        # Update the robot current pose by transforming odom to map frame
        try:
            self.tf_listener.waitForTransform("map", "odom", data.header.stamp, rospy.Duration(1.0))

            odom_pose = PoseStamped()
            odom_pose.header = data.header
            odom_pose.pose = data.pose.pose

            # Transform the pose to map frame
            transformed_pose = self.tf_listener.transformPose("map", odom_pose)
            self.current_pose = transformed_pose

            rospy.logdebug(f"Transformed Pose (map): ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Transformation error: {e}")
            self.current_pose = None

    def user_command_callback(self, data):
        with self.lock:
            user_command = data.data.strip()
            rospy.loginfo(f"Received user command: {user_command}")

            if self.current_pose is None or self.environment_data is None:
                rospy.logwarn("Current robot pose or environment data is unknown. Waiting for data.")
                return

            # Extract target object name from the command
            target_object_name = self.extract_target_room(user_command)
            if not target_object_name:
                rospy.logerr("Failed to extract target from the command.")
                # **Publish Failure Status**
                self.status_pub.publish("failure:0")  # 0 attempts
                return

            rospy.loginfo(f"Identified target object: {target_object_name}")

            # Find target object coordinates
            target_object = self.find_target_object(target_object_name)
            if not target_object:
                rospy.logerr(f"Target object '{target_object_name}' not found in environment data.")
                # **Publish Failure Status**
                self.status_pub.publish("failure:0")  # 0 attempts
                return

            # Store current target object
            self.current_target_object = target_object

            # Add Logging to Verify Structure
            rospy.loginfo(f"Target Object Structure: {target_object}")
            rospy.loginfo(f"Type of target_object['position']: {type(target_object['position'])}")

            # Initialize attempt counter
            attempt = 0
            waypoints = None

            while attempt < self.max_attempts and not waypoints:
                attempt += 1
                rospy.loginfo(f"Attempt {attempt} to generate waypoints.")
                waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                    llm_client=self.llm_client,
                    current_pose=self.current_pose,
                    target_object=target_object,
                    environment_data=self.environment_data,
                    safe_margin=self.safe_margin,
                    waypoint_spacing=self.waypoint_spacing
                )
                if waypoints:
                    rospy.loginfo(f"Successfully generated waypoints on attempt {attempt}.")
                    # **Publish Success Status with Attempt Count**
                    self.status_pub.publish(f"success:{attempt}")
                    self.publish_waypoints(waypoints)
                else:
                    rospy.logwarn(f"Waypoint generation failed on attempt {attempt}.")

            if not waypoints:
                rospy.logerr("Failed to generate waypoints after maximum attempts.")
                # **Publish Failure Status with Attempt Count**
                self.status_pub.publish(f"failure:{attempt}")

    def obstacle_data_callback(self, msg):
        with self.lock:
            replanning_request = msg.data.strip().lower()
            rospy.loginfo(f"LLM received replanning request: {replanning_request}")

            # Check if the message is an emergency stop signal
            if replanning_request == "emergency_stop":
                rospy.loginfo("Processing replanning due to emergency stop.")

                if self.current_pose is None or self.environment_data is None:
                    rospy.logwarn("Current robot pose or environment data is unknown. Cannot generate new waypoints.")
                    # **Publish Failure Status**
                    self.status_pub.publish("failure:0")  # 0 attempts
                    return

                if self.current_target_object is None:
                    rospy.logwarn("No current target object. Cannot generate new waypoints.")
                    # **Publish Failure Status**
                    self.status_pub.publish("failure:0")  # 0 attempts
                    return

                # Initialize attempt counter
                attempt = 0
                waypoints = None

                while attempt < self.max_attempts and not waypoints:
                    attempt += 1
                    rospy.loginfo(f"Replanning Attempt {attempt} to generate new waypoints.")
                    waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                        llm_client=self.llm_client,
                        current_pose=self.current_pose,
                        target_object=self.current_target_object,
                        environment_data=self.environment_data,
                        safe_margin=self.safe_margin,
                        waypoint_spacing=self.waypoint_spacing
                    )
                    if waypoints:
                        rospy.loginfo(f"Successfully generated waypoints on replanning attempt {attempt}.")
                        # **Publish Success Status with Attempt Count**
                        self.status_pub.publish(f"success:{attempt}")
                        self.publish_waypoints(waypoints)
                    else:
                        rospy.logwarn(f"Replanning failed on attempt {attempt}.")

                if not waypoints:
                    rospy.logerr("Failed to generate waypoints after maximum replanning attempts.")
                    # **Publish Failure Status with Attempt Count**
                    self.status_pub.publish(f"failure:{attempt}")
            else:
                rospy.logwarn(f"Received unknown replanning request: {replanning_request}")

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
                # Normalize the target: lowercase and replace spaces with underscores
                normalized_target = target.lower().replace(' ', '_')
                if normalized_target.isdigit():
                    # Construct the full object name
                    normalized_target = f"room_number_plate_{normalized_target}"
                rospy.loginfo(f"Extracted and normalized target object name: {normalized_target}")
                return normalized_target
        rospy.logwarn("Failed to extract a target object from the command.")
        return None

    def find_target_object(self, target_object_name):
        rospy.loginfo(f"Searching for target object: {target_object_name}")
        for obj in self.environment_data.get('objects', []):
            obj_name = obj.get('name', '')
            pos = obj.get('position', {})
            if obj_name.lower() == target_object_name.lower():
                rospy.loginfo(f"Found target object: {obj_name} at position {pos}")
                # Ensure position is a dictionary
                position = {
                    'x': pos.get('x', 0.0),
                    'y': pos.get('y', 0.0),
                    'z': pos.get('z', 0.0)
                }
                return {'name': obj_name, 'position': position}
        rospy.logerr(f"Target object '{target_object_name}' not found in environment data.")
        return None

    def publish_waypoints(self, waypoints):
        # Create a PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        # Append all waypoints
        for waypoint in waypoints:
            pose = Pose()
            pose.position.x = waypoint['x']
            pose.position.y = waypoint['y']
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # Neutral orientation
            pose_array.poses.append(pose)
            rospy.loginfo(f"Added waypoint: ({waypoint['x']}, {waypoint['y']})")

        # Publish the PoseArray
        self.waypoints_pub.publish(pose_array)
        rospy.loginfo(f"Published PoseArray with {len(pose_array.poses)} waypoints.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = LLMPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
