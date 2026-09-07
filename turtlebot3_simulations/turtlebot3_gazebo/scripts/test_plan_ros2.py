#!/usr/bin/env python3
# ROS 2 version of test_plan - Text command interface

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios


class TestPlan(Node):
    def __init__(self):
        super().__init__('test_plan')
        self.publisher_ = self.create_publisher(String, '/user_command', 10)
        self.get_logger().info("Test Plan node started. Enter your navigation commands.")
        
    def run(self):
        # Save terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            while rclpy.ok():
                try:
                    # Prompt the user for input
                    print("Enter your command (e.g., 'Go to the window'): ", end='', flush=True)
                    user_input = input()
                    
                    if user_input:
                        # Log and publish the user command
                        self.get_logger().info(f"Publishing user command: {user_input}")
                        msg = String()
                        msg.data = user_input
                        self.publisher_.publish(msg)
                        
                except EOFError:
                    # Handle end-of-file (e.g., Ctrl+D)
                    self.get_logger().warn("EOF detected. Shutting down test_plan node.")
                    break
                except KeyboardInterrupt:
                    # Handle keyboard interrupt (Ctrl+C)
                    self.get_logger().info("Keyboard interrupt received. Shutting down test_plan node.")
                    break
                except Exception as e:
                    # Handle unexpected exceptions
                    self.get_logger().error(f"Error in test_plan node: {e}")
                    break
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestPlan()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
