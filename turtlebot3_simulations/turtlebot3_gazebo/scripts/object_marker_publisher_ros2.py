#!/usr/bin/env python3
# ROS 2 version of Object Marker Publisher

import rclpy
from rclpy.node import Node
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os

class ObjectMarkerPublisher(Node):
    def __init__(self):
        super().__init__('object_marker_publisher')
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Get the path to the YAML file from parameters
        self.declare_parameter('environment_yaml', '')
        self.yaml_path = self.get_parameter('environment_yaml').value
        
        if not os.path.isfile(self.yaml_path):
            self.get_logger().error(f"YAML file not found at path: {self.yaml_path}")
            return
        
        # Load the YAML data
        try:
            with open(self.yaml_path, 'r') as file:
                self.environment_data = yaml.safe_load(file)
            self.get_logger().info(f"Loaded environment data from {self.yaml_path}")
        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error parsing YAML file: {exc}")
            return
        
        # Create timer to publish markers at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_markers)
    
    def publish_markers(self):
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # Publish Corridor Walls and Static Obstacles
        objects = self.environment_data.get('objects', [])
        for idx, obj in enumerate(objects):
            name = obj.get('name', f"object_{idx}")
            position = obj.get('position', {'x': 0, 'y': 0, 'z': 0})
            size = obj.get('size', None)
            
            # Validate position data
            if not all(k in position for k in ('x', 'y', 'z')):
                self.get_logger().warn(f"Object '{name}' has incomplete position data. Skipping.")
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
                marker.pose.position.z = position['z'] + size.get('height', 1.0) / 2
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = size.get('width', 1.0)
                marker.scale.y = size.get('length', 1.0)
                marker.scale.z = size.get('height', 1.0)
                marker.color.a = 0.8
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                
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
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
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
                    text_marker.pose.position.z = position['z'] + size.get('height', 1.0) + 0.2
                else:
                    text_marker.pose.position.z = position['z'] + 0.5
                text_marker.pose.orientation.x = 0.0
                text_marker.pose.orientation.y = 0.0
                text_marker.pose.orientation.z = 0.0
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.4
                text_marker.color.a = 1.0
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.text = name
                
                marker_array.markers.append(text_marker)
        
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
        self.get_logger().debug("Published MarkerArray.")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
