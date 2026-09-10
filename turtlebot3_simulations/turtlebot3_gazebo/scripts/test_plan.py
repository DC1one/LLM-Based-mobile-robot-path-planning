#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    # Initialize the ROS node
    rospy.init_node('test_plan', anonymous=True)

    # Publisher for user commands
    pub = rospy.Publisher('/user_command', String, queue_size=10)

    # Set the publishing rate (1 Hz)
    rate = rospy.Rate(1)

    rospy.loginfo("Test Plan node started. Enter your navigation commands.")

    while not rospy.is_shutdown():
        try:
            # Prompt the user for input
            user_input = input("Enter your command (e.g., 'Go to the window'): ")

            if user_input:
                # Log and publish the user command
                rospy.loginfo(f"Publishing user command: {user_input}")
                pub.publish(user_input)
        except EOFError:
            # Handle end-of-file (e.g., Ctrl+D)
            rospy.logwarn("EOF detected. Shutting down test_plan node.")
            break
        except KeyboardInterrupt:
            # Handle keyboard interrupt (Ctrl+C)
            rospy.loginfo("Keyboard interrupt received. Shutting down test_plan node.")
            break
        except Exception as e:
            # Handle unexpected exceptions
            rospy.logerr(f"Error in test_plan node: {e}")
            break

        # Sleep to maintain the loop rate
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
