#!/usr/bin/env python3
# ROS 2 version of Environment Data Provider for corridor_world_building_04 (department)

import rclpy
from rclpy.node import Node
import yaml
import json
import os
from std_msgs.msg import String


class EnvironmentDataProvider(Node):
    def __init__(self):
        super().__init__('environment_data_provider')
        
        # Declare parameters
        self.declare_parameter('environment_yaml', 'department.yaml')
        self.declare_parameter('package_path', '')
        
        # Get parameters
        environment_yaml = self.get_parameter('environment_yaml').value
        package_path = self.get_parameter('package_path').value
        
        # Publisher for object list
        self.object_list_pub = self.create_publisher(String, '/object_list', 10)
        
        # Load environment data
        self.environment_data = self.load_environment_data(environment_yaml, package_path)
        
        if self.environment_data is None:
            self.get_logger().error("Failed to load environment data. Shutting down.")
            return
        
        # Publish environment data periodically
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_environment_data)
        
        self.get_logger().info(f"Environment Data Provider initialized with {environment_yaml}")
    
    def load_environment_data(self, environment_yaml, package_path):
        """Load environment data from YAML file"""
        try:
            # Construct full path to YAML file
            yaml_path = os.path.join(package_path, 'object_list', environment_yaml)
            
            self.get_logger().info(f"Loading environment data from: {yaml_path}")
            
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
            
            self.get_logger().info(f"Successfully loaded environment data: {environment_yaml}")
            return data
            
        except Exception as e:
            self.get_logger().error(f"Error loading environment data: {e}")
            return None
    
    def publish_environment_data(self):
        """Publish environment data as JSON string"""
        if self.environment_data:
            msg = String()
            msg.data = json.dumps(self.environment_data)
            self.object_list_pub.publish(msg)
            self.get_logger().debug("Published environment data")


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentDataProvider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
