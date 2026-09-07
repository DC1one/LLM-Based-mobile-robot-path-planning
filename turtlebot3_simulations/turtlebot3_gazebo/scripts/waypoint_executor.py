#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Pose, PoseArray, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf
import math
import json
import time
import os

class WaypointExecutor:
    def __init__(self):
        rospy.init_node('waypoint_executor', anonymous=False, log_level=rospy.INFO)

        # Load environment data from YAML file
        environment_yaml_path = rospy.get_param('~environment_yaml')
        self.load_environment_data(environment_yaml_path)

        # State definitions
        self.STATE_NAVIGATING = 'Navigating'
        self.STATE_EMERGENCY_STOPPED = 'Emergency Stopped'
        self.STATE_REPLANNING = 'Replanning'
        self.STATE_RESUMING_NAVIGATION = 'Resuming Navigation'

        # Initial state
        self.state = self.STATE_NAVIGATING

        # Emergency Stop Flag
        self.emergency_stop = False

        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)  # meters per second
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # radians per second
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.2)  # meters
        self.angle_threshold = rospy.get_param('~angle_threshold', math.radians(5))  # radians (~5 degrees)
        self.obstacle_angle_range = rospy.get_param('~obstacle_angle_range', math.radians(15))  # radians (~15 degrees)
        self.self_obstacle_distance = rospy.get_param('~self_obstacle_distance', 0.3)  # meters
        self.critical_distance = rospy.get_param('~critical_distance', 0.5)  # meters
        self.max_replans = rospy.get_param('~max_replans', 3)
        self.replan_cooldown = rospy.get_param('~replan_cooldown', 5.0)  # seconds
        self.replan_attempts = 0

        # Subscribers and Publishers
        self.waypoints_sub = rospy.Subscriber('/llm_waypoints', PoseArray, self.waypoints_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)  # LaserScan subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Publisher for obstacle data to '/obstacle_data'
        self.obstacle_data_pub = rospy.Publisher('/obstacle_data', String, queue_size=10)

        # Publisher for execution status to '/execution_status'
        self.execution_status_pub = rospy.Publisher('/execution_status', String, queue_size=10)  # New Publisher

        # State variables
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.tf_listener = tf.TransformListener()

        # Cooldown tracking
        self.last_replan_time = 0
        self.last_waypoint_time = rospy.Time.now()

        rospy.loginfo("Waypoint Executor initialized.")

    def load_environment_data(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                self.environment_data = yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"Failed to load environment data: {e}")
            self.environment_data = None

    def get_obstacle_boundaries(self):
        if not self.environment_data:
            rospy.logwarn("No environment data loaded. Unable to retrieve obstacle boundaries.")
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
        rospy.loginfo("Received new waypoints.")

        # Transform and validate waypoints
        valid_waypoints = []
        obstacle_boundaries = self.get_obstacle_boundaries()

        for pose in pose_array.poses:
            waypoint_pose = PoseStamped()
            waypoint_pose.header = pose_array.header
            waypoint_pose.pose = pose

            try:
                # Transform waypoint to odom frame
                self.tf_listener.waitForTransform("odom", pose_array.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                transformed_pose = self.tf_listener.transformPose("odom", waypoint_pose)
                waypoint_x = transformed_pose.pose.position.x
                waypoint_y = transformed_pose.pose.position.y

                # Validate waypoint distance from all obstacles (bounding boxes)
                is_valid = True
                for boundary in obstacle_boundaries:
                    if (boundary['x_min'] <= waypoint_x <= boundary['x_max'] and
                        boundary['y_min'] <= waypoint_y <= boundary['y_max']):
                        rospy.logwarn(f"Waypoint ({waypoint_x:.2f}, {waypoint_y:.2f}) is inside an obstacle boundary. Ignoring this waypoint.")
                        is_valid = False
                        break

                if is_valid:
                    valid_waypoints.append((waypoint_x, waypoint_y))
                    rospy.loginfo(f"Transformed and validated waypoint: ({waypoint_x:.2f}, {waypoint_y:.2f})")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"Failed to transform waypoint: {e}")
                continue

        if not valid_waypoints:
            rospy.logerr("No valid waypoints received after validation.")
            return

        # Update waypoints and reset waypoint index
        self.waypoints = valid_waypoints
        self.current_waypoint_index = 0
        rospy.loginfo(f"Waypoints to navigate: {self.waypoints}")

        # Update the last waypoint reception time
        self.last_waypoint_time = rospy.Time.now()

        # Reset the emergency_stop flag if in Resuming Navigation or Emergency Stopped
        if self.state in [self.STATE_EMERGENCY_STOPPED, self.STATE_RESUMING_NAVIGATION]:
            rospy.loginfo("New safe waypoints received. Preparing to resume navigation.")
            self.state = self.STATE_RESUMING_NAVIGATION
            self.replan_attempts = 0

    def odom_callback(self, data):  # Callback to update the robot's current pose based on odometry data.
        self.current_pose = data.pose.pose

    def scan_callback(self, data):
        if self.current_pose is None:
            return

        obstacles_in_map = []
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        num_ranges = len(data.ranges)

        relevant_distances = []

        for i in range(num_ranges):
            distance = data.ranges[i]
            if 0.0 < distance < self.critical_distance:
                angle = angle_min + i * angle_increment

                # Check if the obstacle is within the front angular range
                if abs(angle) > self.obstacle_angle_range:
                    continue

                # Ignore obstacles too close to the robot's center (self)
                if distance < self.self_obstacle_distance:
                    continue

                relevant_distances.append(distance)

        # Apply median filtering to reduce noise
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
                    rospy.logwarn("Emergency Stop! Obstacle detected within critical distance.")
                    self.emergency_stop = True
                    self.state = self.STATE_EMERGENCY_STOPPED
                    self.stop_robot()

                    # **Publish simple emergency stop notification**
                    obstacle_data_msg = String()
                    obstacle_data_msg.data = "emergency_stop"
                    self.obstacle_data_pub.publish(obstacle_data_msg)

                    rospy.loginfo("Published 'emergency_stop' to /obstacle_data for replanning.")
                    self.last_replan_time = current_time
                    self.replan_attempts += 1
        else:
            if self.state == self.STATE_RESUMING_NAVIGATION:
                rospy.loginfo("Successfully avoided the obstacle. Resuming navigation.")
                self.state = self.STATE_NAVIGATING

    def timer_callback(self, event):
        if self.current_pose is None:
            self.stop_robot()
            rospy.logwarn("Current pose is unknown. Awaiting pose data.")
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

        # Calculate distance and angle to the waypoint
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Get current yaw
        yaw = self.get_yaw()
        desired_angle = math.atan2(dy, dx)
        angle = desired_angle - yaw

        # Normalize angle to [-pi, pi]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()

        # Proportional control for angular velocity
        if abs(angle) > self.angle_threshold:
            twist.angular.z = self.angular_speed * angle
            twist.linear.x = 0.0  # Stop moving forward while turning
            rospy.logdebug(f"Rotating with angular velocity: {twist.angular.z:.3f} rad/s")
        else:
            # Move at constant linear speed
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * angle
            rospy.logdebug(f"Moving forward with linear velocity: {twist.linear.x:.3f} m/s")
            rospy.logdebug(f"Angular velocity set to: {twist.angular.z:.3f} rad/s")

        self.cmd_vel_pub.publish(twist)

        # Check if waypoint is reached
        if distance < self.distance_threshold:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1} reached at ({target_x:.2f}, {target_y:.2f}).")
            self.current_waypoint_index += 1
            rospy.sleep(0.5)  # Small delay to stabilize movement

    def get_yaw(self):
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def publish_execution_completion(self):
        completion_msg = String()
        completion_msg.data = "completed"
        self.execution_status_pub.publish(completion_msg)

    def run(self):
        # Timer for continuous waypoint processing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10 Hz
        rospy.spin()


if __name__ == '__main__':
    try:
        executor = WaypointExecutor()
        executor.run()
    except rospy.ROSInterruptException:
        pass
