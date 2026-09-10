#!/usr/bin/env python3
import rospy
import yaml
import json
import os
from std_msgs.msg import String


class EnvironmentDataProvider:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('environment_data_provider', anonymous=True)

        # Publisher for object list
        self.object_list_pub = rospy.Publisher('/object_list', String, queue_size=10)

        # Load environment data
        self.environment_data = self.load_environment_data()

        if self.environment_data is None:
            rospy.logerr("Environment data not loaded. Shutting down.")
            rospy.signal_shutdown("Environment data not loaded.")
            return

        # Set publishing rate
        self.rate = rospy.Rate(1)  # 1 Hz

        # rospy.loginfo("Environment Data Provider initialized and publishing to /object_list.")

        # Start publishing loop
        self.publish_environment_data()

    def load_environment_data(self):
        if not rospy.has_param('~environment_yaml'):
            rospy.logerr("Parameter '~environment_yaml' not set. Cannot load environment data.")
            return None

        yaml_file = rospy.get_param('~environment_yaml')
        yaml_file_path = rospy.get_param('~package_path', '')  # Adjusted parameter

        if yaml_file_path:
            full_path = os.path.join(yaml_file_path, 'object_list', yaml_file)
        else:
            rospy.logerr("Parameter '~package_path' not set. Cannot determine YAML file path.")
            return None

        try:
            with open(full_path, 'r') as file:
                data = yaml.safe_load(file)
            # rospy.loginfo(f"Loaded environment data from {full_path}")
            return data
        except FileNotFoundError:
            rospy.logerr(f"YAML file not found at {full_path}")
            return None
        except yaml.YAMLError as exc:
            rospy.logerr(f"Error parsing YAML file: {exc}")
            return None

    def publish_environment_data(self):
        while not rospy.is_shutdown():
            if self.environment_data is None:
                rospy.logerr("No environment data to publish.")
                rospy.signal_shutdown("No environment data to publish.")
                break

            # Extract main corridor dimensions
            main_corridor_length = self.environment_data.get('main_corridor_length')
            main_corridor_width = self.environment_data.get('main_corridor_width')

            # Extract side corridor dimensions
            side_corridor_length = self.environment_data.get('side_corridor_length')
            side_corridor_width = self.environment_data.get('side_corridor_width')

            # Check if all corridor dimensions are present
            if None in [main_corridor_length, main_corridor_width, side_corridor_length, side_corridor_width]:
                rospy.logerr("Some corridor dimensions are missing in YAML.")
                rospy.signal_shutdown("Invalid environment data.")
                break

            # Extract object list
            objects = self.environment_data.get('objects', [])

            # Prepare a list of objects with their names, positions, and sizes (if available)
            object_list = []
            for obj in objects:
                name = obj.get('name')
                position = obj.get('position', {})

                if not name or not position:
                    rospy.logwarn(f"Invalid object entry: {obj}. Skipping.")
                    continue

                # Ensure position has x, y, z
                x = position.get('x')
                y = position.get('y')
                z = position.get('z')

                if x is None or y is None or z is None:
                    rospy.logwarn(f"Missing position coordinates for object '{name}'. Skipping.")
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
                        rospy.logwarn(f"Incomplete size information for object '{name}'. Size will be omitted.")

                object_list.append(obj_dict)

            # Create a dictionary to publish
            publish_data = {
                'main_corridor_length': main_corridor_length,
                'main_corridor_width': main_corridor_width,
                'side_corridor_length': side_corridor_length,
                'side_corridor_width': side_corridor_width,
                'objects': object_list
            }

            # Convert to JSON string
            publish_json = json.dumps(publish_data)

            # Publish to /object_list
            self.object_list_pub.publish(publish_json)
            rospy.logdebug("Published object list to /object_list.")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        provider = EnvironmentDataProvider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass