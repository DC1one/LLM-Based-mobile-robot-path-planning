#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math
import time
from threading import Lock
import os

class MetricsCollector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('metrics_collector', anonymous=False)
        rospy.loginfo("Metrics Collector Node Initialized.")

        # Define the path for the CSV file
        csv_dir = os.path.expanduser('~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/results/mathstral')
        csv_path = os.path.join(csv_dir, 'mathstral_Env_3_1.csv')

        # Create the directory if it doesn't exist
        try:
            os.makedirs(csv_dir, exist_ok=True)
        except Exception as e:
            rospy.logerr(f"Failed to create metrics CSV directory at {csv_dir}: {e}")
            rospy.signal_shutdown("CSV directory creation failed.")

        # Initialize CSV file
        try:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'User_Command',
                'Path_Planning_Time',              # in seconds
                'Execution_Time',                  # in seconds
                'Waypoint_Generation_Success_Rate',# in percentage
                'Path_Length',                     # in meters
                'Collision_Detection_Event',       # count
                'Replanning_Rate'                  # count
            ])
            self.csv_file.flush()
        except Exception as e:
            rospy.signal_shutdown("CSV file initialization failed.")

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

        # Instance variable for Path Planning Time
        self.path_planning_time = 0.0

        # State variables
        self.current_waypoints = []
        self.previous_pose = None

        # Subscribe to necessary topics
        rospy.Subscriber('/user_command', String, self.user_command_callback)
        rospy.Subscriber('/llm_waypoints', PoseArray, self.waypoints_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/obstacle_data', String, self.obstacle_data_callback)
        rospy.Subscriber('/waypoint_generation_status', String, self.waypoint_generation_status_callback)
        rospy.Subscriber('/execution_status', String, self.execution_status_callback)

    def user_command_callback(self, msg):
        with self.lock:
            self.current_command = msg.data.strip()
            self.command_start_time = time.time()
            self.total_waypoint_generations += 1
            rospy.loginfo(f"Received User Command: '{self.current_command}' at {self.command_start_time}")

    def waypoints_callback(self, msg):
        with self.lock:
            if not self.current_command:
                rospy.logwarn("Waypoints received without an active user command.")
                return

            self.waypoint_received_time = time.time()
            self.path_planning_time = self.waypoint_received_time - self.command_start_time

            rospy.loginfo(f"Waypoints received for command '{self.current_command}' at {self.waypoint_received_time}")
            rospy.loginfo(f"Path Planning Time: {self.path_planning_time:.3f} seconds")

            # Store waypoints for path length calculation
            self.current_waypoints = [(pose.position.x, pose.position.y) for pose in msg.poses]
            self.execution_start_time = time.time()
            rospy.loginfo(f"Execution started at {self.execution_start_time}")

            # Reset path length for new execution
            self.path_length = 0.0

    def waypoint_generation_status_callback(self, msg):
        with self.lock:
            status_message = msg.data.strip().lower()
            rospy.loginfo(f"Received waypoint generation status: '{status_message}'")

            # Parse the status message to extract status and attempts
            try:
                status, attempts = status_message.split(':')
                attempts = int(attempts)
            except ValueError:
                rospy.logwarn(f"Invalid waypoint generation status format: '{status_message}'. Expected 'success:<attempts>' or 'failure:<attempts>'.")
                return

            if status == "success":
                self.successful_waypoint_generations += 1
                rospy.loginfo(f"Waypoint generation succeeded on attempt {attempts}.")
            elif status == "failure":
                rospy.loginfo(f"Waypoint generation failed after {attempts} attempts.")
            else:
                rospy.logwarn(f"Unknown waypoint generation status received: '{status}'")

    def execution_status_callback(self, msg):
        with self.lock:
            status = msg.data.strip().lower()
            if status == "completed":
                if self.execution_start_time is None:
                    return
                self.execution_end_time = time.time()
                execution_time = self.execution_end_time - self.execution_start_time
                rospy.loginfo(f"Execution Time: {execution_time:.3f} seconds")

                # Calculate Waypoint Generation Success Rate
                if self.total_waypoint_generations > 0:
                    success_rate = (self.successful_waypoint_generations / self.total_waypoint_generations) * 100
                else:
                    success_rate = 0.0

                # Calculate Replanning Rate (number of replans per execution)
                replanning_rate = self.replanning_attempts / (self.execution_end_time - self.command_start_time) if (self.execution_end_time - self.command_start_time) > 0 else 0.0

                # Write metrics to CSV
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
                    rospy.loginfo(f"Metrics for command '{self.current_command}' logged to CSV.")
                except Exception as e:
                    rospy.logerr(f"Failed to write metrics to CSV: {e}")

                # Reset metrics for next command
                self.reset_metrics()
            else:
                rospy.logwarn(f"Unknown execution status received: '{status}'")

    def odom_callback(self, msg):
        with self.lock:
            current_pose = msg.pose.pose

            if self.previous_pose:
                dx = current_pose.position.x - self.previous_pose.position.x
                dy = current_pose.position.y - self.previous_pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                self.path_length += distance
                rospy.logdebug(f"Moved {distance:.3f} meters. Total Path Length: {self.path_length:.3f} meters")

            self.previous_pose = current_pose

    def scan_callback(self, msg):
        # This callback can be used to detect collision events in real-time if needed
        pass  # Collision events are tracked via /obstacle_data

    def obstacle_data_callback(self, msg):
        with self.lock:
            # Each message on /obstacle_data indicates a replanning attempt
            if msg.data.strip().lower() == "emergency_stop":
                self.replanning_attempts += 1
                self.collision_events += 1
                rospy.loginfo(f"Obstacle detected. Replanning attempts: {self.replanning_attempts}, Collision events: {self.collision_events}")

    def reset_metrics(self):
        # Reset all metrics variables after logging
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

    def run(self):
        rospy.spin()

    def __del__(self):
        try:
            self.csv_file.close()
        except Exception as e:
            pass
        # Removed rospy.loginfo to prevent logging during shutdown
        # rospy.loginfo("Metrics Collector Node Shutdown. CSV file closed.")

if __name__ == '__main__':
    try:
        collector = MetricsCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
