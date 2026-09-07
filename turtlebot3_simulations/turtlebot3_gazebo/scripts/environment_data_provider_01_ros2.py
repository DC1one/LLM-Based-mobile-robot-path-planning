#!/usr/bin/env python3
# ROS 2 version of Environment Data Provider 01

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import os


class EnvironmentDataProvider(Node):
    def __init__(self):
        super().__init__('environment_data_provider')
        
        # Declare parameters
        self.declare_parameter('environment_yaml', 'default.yaml')
        self.declare_parameter('package_path', '')
        
        # Get parameters
        yaml_file = self.get_parameter('environment_yaml').value
        yaml_file_path = self.get_parameter('package_path').value
        
        # Publisher for object list
        self.object_list_pub = self.create_publisher(String, '/object_list', 10)
        
        # Load environment data
        self.environment_data = self.load_environment_data(yaml_file, yaml_file_path)
        
        if self.environment_data is None:
            self.get_logger().error("Environment data not loaded. Shutting down.")
            return
        
        # Create timer for publishing at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.get_logger().info("Environment Data Provider initialized and publishing to /object_list.")
    
    def load_environment_data(self, yaml_file, package_path):
        if not package_path:
            self.get_logger().error("Parameter 'package_path' not set. Cannot determine YAML file path.")
            return None
        
        full_path = os.path.join(package_path, 'object_list', yaml_file)
        
        try:
            with open(full_path, 'r') as file:
                data = yaml.safe_load(file)
            self.get_logger().debug(f"Loaded environment data from {full_path}")
            return data
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {full_path}")
            return None
        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error parsing YAML file: {exc}")
            return None
    
    def publish_callback(self):
        if self.environment_data is None:
            self.get_logger().error("No environment data to publish.")
            return
        
        # Extract corridor dimensions
        corridor_length = self.environment_data.get('corridor_length')
        corridor_width = self.environment_data.get('corridor_width')
        
        if corridor_length is None or corridor_width is None:
            self.get_logger().error("Corridor dimensions 'corridor_length' and/or 'corridor_width' missing in YAML.")
            return
        
        # Extract object list
        objects = self.environment_data.get('objects', [])
        
        # Prepare a list of objects with their names, positions, and sizes (if available)
        object_list = []
        for obj in objects:
            name = obj.get('name')
            position = obj.get('position', {})
            
            if not name or not position:
                self.get_logger().warn(f"Invalid object entry: {obj}. Skipping.")
                continue
            
            # Ensure position has x, y, z
            x = position.get('x')
            y = position.get('y')
            z = position.get('z')
            
            if x is None or y is None or z is None:
                self.get_logger().warn(f"Missing position coordinates for object '{name}'. Skipping.")
                continue
            
            obj_dict = {
                'name': name,
                'position': {
                    'x': x,
                    'y': y,
                    'z': z
                }
            }
            
            # Include size if available
            size = obj.get('size', {})
            if size:
                width = size.get('width')
                height = size.get('height')
                length = size.get('length')
                if width is not None and height is not None and length is not None:
                    obj_dict['size'] = {
                        'width': width,
                        'height': height,
                        'length': length
                    }
                else:
                    self.get_logger().warn(f"Incomplete size information for object '{name}'. Size will be omitted.")
            
            object_list.append(obj_dict)
        
        # Create a dictionary to publish
        publish_data = {
            'corridor_length': corridor_length,
            'corridor_width': corridor_width,
            'objects': object_list
        }
        
        # Convert to JSON string
        publish_json = json.dumps(publish_data)
        
        # Publish to /object_list
        msg = String()
        msg.data = publish_json
        self.object_list_pub.publish(msg)
        self.get_logger().debug("Published object list to /object_list.")


def main(args=None):
    rclpy.init(args=args)
    provider = EnvironmentDataProvider()
    rclpy.spin(provider)
    provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
