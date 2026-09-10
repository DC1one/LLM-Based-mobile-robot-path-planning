#!/usr/bin/env python3

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os

class ObjectMarkerPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_marker_publisher', anonymous=True)

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # Get the path to the YAML file from parameters
        self.yaml_path = rospy.get_param('~environment_yaml', '')

        if not os.path.isfile(self.yaml_path):
            rospy.logerr(f"YAML file not found at path: {self.yaml_path}")
            rospy.signal_shutdown("YAML file not found.")
            return

        # Load the YAML data
        with open(self.yaml_path, 'r') as file:
            try:
                self.environment_data = yaml.safe_load(file)
                # rospy.loginfo(f"Loaded environment data from {self.yaml_path}")
            except yaml.YAMLError as exc:
                rospy.logerr(f"Error parsing YAML file: {exc}")
                rospy.signal_shutdown("YAML parsing error.")
                return

        # Publish markers at a fixed rate
        self.publish_rate = rospy.Rate(1)  # 1 Hz
        self.publish_markers()

    def publish_markers(self):
        while not rospy.is_shutdown():
            marker_array = MarkerArray()
            current_time = rospy.Time.now()

            # Publish Corridor Walls and Static Obstacles
            objects = self.environment_data.get('objects', [])
            for idx, obj in enumerate(objects):
                name = obj.get('name', f"object_{idx}")
                position = obj.get('position', {'x': 0, 'y': 0, 'z': 0})
                size = obj.get('size', None)

                # Validate position data
                if not all(k in position for k in ('x', 'y', 'z')):
                    rospy.logwarn(f"Object '{name}' has incomplete position data. Skipping.")
                    continue

                # Determine marker type based on presence of size
                if size:
                    # Create a CUBE marker for obstacles
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = current_time
                    marker.ns = "obstacles"
                    marker.id = idx
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = position['x']
                    marker.pose.position.y = position['y']
                    marker.pose.position.z = position['z'] + size.get('height', 1.0) / 2  # Adjust z to center the cube
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = size.get('width', 1.0)
                    marker.scale.y = size.get('length', 1.0)
                    marker.scale.z = size.get('height', 1.0)
                    marker.color.a = 0.8  # Semi-transparent
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.text = ""

                else:
                    # Create a SPHERE marker for regular objects
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = current_time
                    marker.ns = "objects"
                    marker.id = idx
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = position['x']
                    marker.pose.position.y = position['y']
                    marker.pose.position.z = position['z']
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.3  # Size of the sphere
                    marker.scale.y = 0.3
                    marker.scale.z = 0.3
                    marker.color.a = 1.0  # Fully opaque
                    marker.color.r = 1.0  # Red color
                    marker.color.g = 0.0
                    marker.color.b = 0.0

                # Add the marker to the array
                marker_array.markers.append(marker)

                # If the object has a name, add a text marker
                if name:
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = current_time
                    text_marker.ns = "objects_text" if not size else "obstacles_text"
                    text_marker.id = idx + len(objects)
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = position['x']
                    text_marker.pose.position.y = position['y']
                    if size:
                        text_marker.pose.position.z = position['z'] + size.get('height', 1.0) + 0.2  # Above the cube
                    else:
                        text_marker.pose.position.z = position['z'] + 0.5  # Above the sphere
                    text_marker.pose.orientation.x = 0.0
                    text_marker.pose.orientation.y = 0.0
                    text_marker.pose.orientation.z = 0.0
                    text_marker.pose.orientation.w = 1.0
                    text_marker.scale.z = 0.4  # Font size
                    text_marker.color.a = 1.0  # Fully opaque
                    text_marker.color.r = 1.0  # White color
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.text = name

                    # Add text marker to the array
                    marker_array.markers.append(text_marker)

            # Publish the MarkerArray
            self.marker_pub.publish(marker_array)
            rospy.logdebug("Published MarkerArray.")
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        ObjectMarkerPublisher()
    except rospy.ROSInterruptException:
        pass
