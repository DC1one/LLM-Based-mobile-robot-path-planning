#!/usr/bin/env python3

import rospy
import json
import re
import matplotlib.pyplot as plt
import os


def validate_waypoints(waypoints, environment_data, safe_margin, target_position):
    corridor_length = environment_data.get('corridor_length', 15.0)  # meters (Y-axis)
    corridor_width = environment_data.get('corridor_width', 4.0)  # meters (X-axis)

    # Calculate boundaries
    corridor_x_min = -corridor_width / 2 + safe_margin  # e.g., -1.5
    corridor_x_max = corridor_width / 2 - safe_margin  # e.g., 1.5
    corridor_y_min = 0.0  # 0.0
    corridor_y_max = corridor_length - safe_margin  # e.g., 14.5

    rospy.loginfo(f"Validating waypoints within corridor boundaries:")
    rospy.loginfo(f"X-axis: {corridor_x_min} <= x <= {corridor_x_max}")
    rospy.loginfo(f"Y-axis: {corridor_y_min} <= y <= {corridor_y_max}")

    valid_waypoints = []
    for waypoint in waypoints:
        x = waypoint['x']
        y = waypoint['y']

        # Boundary Compliance
        if not (corridor_x_min <= x <= corridor_x_max and
                corridor_y_min <= y <= corridor_y_max):
            rospy.logwarn(f"Waypoint ({x}, {y}) is outside corridor boundaries and will be ignored.")
            continue

        valid_waypoints.append(waypoint)

    if not valid_waypoints:
        rospy.logerr("No valid waypoints within corridor boundaries and safe margins.")
        return None

    # Ensure the final waypoint matches the target position within a tolerance
    final_waypoint = valid_waypoints[-1]
    try:
        target_x = target_position['x']
        target_y = target_position['y']
    except (TypeError, KeyError) as e:
        rospy.logerr(f"Invalid target_position format: {e}")
        return None

    tolerance = 0.05  # meters

    distance = ((final_waypoint['x'] - target_x) ** 2 + (final_waypoint['y'] - target_y) ** 2) ** 0.5
    rospy.loginfo(f"Distance between final waypoint and target: {distance} meters")

    if distance >= tolerance:
        rospy.logerr(
            f"Final waypoint ({final_waypoint['x']}, {final_waypoint['y']}) does not match the target position ({target_x}, {target_y}) within a tolerance of {tolerance} meters."
        )
        return None

    return valid_waypoints


def agents(llm_client, system_prompt, user_prompt):
    try:
        response = llm_client.chat(
            model='llama3.1', # qwen2.5, llama3.1, mathstral
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]
        )
        answer = response['message']['content']
        rospy.loginfo(f"LLM Response:\n{answer}")
        return answer
    except Exception as e:
        rospy.logerr(f"LLM interaction failed: {e}")
        return None


def parse_waypoints(response):
    if not response:
        rospy.logerr("Empty response received for waypoints parsing.")
        return None
    if not isinstance(response, str):
        rospy.logerr("Response is not a string.")
        return None
    try:
        # Remove code fences
        response = re.sub(r'^```(json)?\s*', '', response.strip())
        response = re.sub(r'\s*```$', '', response.strip())

        # Replace single quotes with double quotes
        response = response.replace("'", '"')

        # Extract the JSON array
        match = re.search(r'\[.*]', response, re.DOTALL)
        if not match:
            rospy.logerr("LLM response does not contain a valid JSON array.")
            return None
        json_str = match.group(0)

        # Parse the JSON string
        waypoints = json.loads(json_str)

        # Validate waypoints
        if not isinstance(waypoints, list):
            rospy.logerr("Waypoints should be a list of dictionaries.")
            return None
        for waypoint in waypoints:
            if not isinstance(waypoint, dict) or 'x' not in waypoint or 'y' not in waypoint:
                rospy.logerr("Invalid waypoint format. Each waypoint must be a dictionary with 'x' and 'y' keys.")
                return None
            # Convert coordinates to float
            waypoint['x'] = float(waypoint['x'])
            waypoint['y'] = float(waypoint['y'])
        return waypoints
    except Exception as e:
        rospy.logerr(f"Failed to parse waypoints: {e}")
        return None


def determine_corridor(position):
    # Single Corridor Definition
    corridor = {
        'Corridor': {
            'x_min': -1.5,
            'x_max': 1.5,
            'y_min': -0.01,
            'y_max': 14.5
        }
    }

    x = position['x']
    y = position['y']

    # rospy.loginfo(f"Checking if position ({x}, {y}) is within the corridor.")
    bounds = corridor['Corridor']
    # rospy.loginfo(f"Corridor Bounds - X-axis: {bounds['x_min']} <= {x} <= {bounds['x_max']}")
    # rospy.loginfo(f"Corridor Bounds - Y-axis: {bounds['y_min']} <= {y} <= {bounds['y_max']}")
    if (bounds['x_min'] <= x <= bounds['x_max'] and
            bounds['y_min'] <= y <= bounds['y_max']):
        rospy.loginfo(f"Position ({x}, {y}) is in the Corridor.")
        return 'Corridor'
    rospy.logwarn(f"Position ({x}, {y}) does not lie within the Corridor.")
    return None


def plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin):
    corridor_length = environment_data.get('corridor_length', 15.0)
    corridor_width = environment_data.get('corridor_width', 4.0)

    corridor_x_min = -corridor_width / 2 + safe_margin
    corridor_x_max = corridor_width / 2 - safe_margin
    corridor_y_min = 0.0
    corridor_y_max = corridor_length - safe_margin

    rospy.loginfo("Plotting waypoints and corridor boundaries.")

    fig, ax = plt.subplots(figsize=(7, 6))  # Adjust the size as needed

    # Draw corridor
    corridor = plt.Rectangle((corridor_x_min, corridor_y_min),
                             corridor_x_max - corridor_x_min,
                             corridor_y_max - corridor_y_min,
                             edgecolor='blue', facecolor='lightblue', alpha=0.5, label='Corridor')
    ax.add_patch(corridor)

    # Plot current and target positions
    ax.plot(current_position['x'], current_position['y'], 'ro', label='Current Position')
    ax.plot(target_position['x'], target_position['y'], 'go', label='Target Position')

    # Plot waypoints
    if waypoints:
        x_waypoints = [wp['x'] for wp in waypoints]
        y_waypoints = [wp['y'] for wp in waypoints]
        # Plot the connection from current position to the first waypoint
        ax.plot([current_position['x'], x_waypoints[0]], [current_position['y'], y_waypoints[0]], 'k--', label='Path')
        ax.plot(x_waypoints, y_waypoints, 'k--', marker='x', label='Waypoints')

        ax.set_xlabel('X-axis (meters)', fontweight='bold')
        ax.set_ylabel('Y-axis (meters)', fontweight='bold')
        ax.legend()
        ax.set_aspect('equal')
        ax.grid(True)
        plt.axis('equal')
        save_dir = '/home/albert/Pictures'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        filename = os.path.join(save_dir, 'E3.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.show()

def generate_waypoints_with_navigation_agent(llm_client, current_pose, target_object, environment_data, safe_margin,
                                             waypoint_spacing):
    max_attempts = 3  # Maximum number of attempts per agent

    # Extract corridor dimensions
    corridor_length = environment_data.get('corridor_length', 15.0)  # meters (Y-axis)
    corridor_width = environment_data.get('corridor_width', 4.0)  # meters (X-axis)

    # Calculate boundaries
    corridor_x_min = -corridor_width / 2 + safe_margin  # e.g., -1.5
    corridor_x_max = corridor_width / 2 - safe_margin  # e.g., 1.5
    corridor_y_min = 0.0  # 0.0
    corridor_y_max = corridor_length - safe_margin  # e.g., 14.5

    # Determine robot's current corridor (optional for single corridor)
    current_position = {
        'x': current_pose.pose.position.x,
        'y': current_pose.pose.position.y
    }
    rospy.loginfo(f"Determining current corridor for position ({current_position['x']}, {current_position['y']}).")
    current_corridor = determine_corridor(current_position)

    if current_corridor is None:
        rospy.logerr("Cannot determine the current corridor of the robot.")
        return None

    # Determine which corridor the target is in (should be the same)
    target_x = target_object['position']['x']
    target_y = target_object['position']['y']

    rospy.loginfo(f"Determined Target Position: x={target_x}, y={target_y}")

    target_position = {
        'x': target_x,
        'y': target_y
    }

    target_corridor = determine_corridor(target_position)

    if target_corridor is None:
        rospy.logerr("Target position does not lie within the corridor.")
        return None
    else:
        if current_corridor == target_corridor:
            rospy.loginfo("Robot and target are in the same corridor. No junction point required.")
        else:
            rospy.logwarn(
                "Robot and target are in different corridors, which should not occur in a single straight corridor.")
            # Depending on the scenario, handle unexpected transitions
            # For a single corridor, this should not happen
            return None

    # Construct system and user prompts
    navigation_system_prompt = f"""
Robot's current position: ({current_position['x']:.2f}, {current_position['y']:.2f})
Target position: ({target_object['position']['x']:.2f}, {target_object['position']['y']:.2f})
Corridor boundaries:

- X-axis: from {corridor_x_min:.2f} to {corridor_x_max:.2f} meters
- Y-axis: from {corridor_y_min:.2f} to {corridor_y_max:.2f} meters

Safe margin: {safe_margin} meters from walls

You are a navigation assistant for a mobile robot in a single straight corridor.
Provide a concise sequence of waypoints as (x, y) coordinates for the robot to follow to reach the destination, avoiding obstacles and maintaining the safe margins defined above. Ensure that:
1. All waypoints are within the corridor boundaries defined above.
2. Help in creating a list of waypoints from the robot's current position to the target position in the best sequence on both axes.
3. The path is direct and efficient, facilitating smooth movement towards the target without any unnecessary deviations.
4. Ensure that the path adheres to the safe margin constraints.
5. Please follow all rules; it's a humble request.
6. Do not include any comments, annotations, or additional text. The output should be a valid JSON array of waypoints only.
7. Format the output strictly as a JSON array of coordinates without any comments or additional text.
"""

    navigation_user_prompt = f"""
    Robot's current position: ({current_position['x']:.2f}, {current_position['y']:.2f})
    Target position: ({target_object['position']['x']:.2f}, {target_object['position']['y']:.2f})

    Safe margin: {safe_margin} meters from walls

    Please provide a concise sequence of waypoints as (x, y) coordinates for the robot to follow to reach the destination, avoiding obstacles and maintaining
    the safe margins defined above. Ensure that:
    1. The waypoints form a continuous and logical path towards the destination that reduces the distance between the robot's current position and the target object **without any backtracking**.
    2. Limit the number of waypoints to a maximum of 3 for efficiency.
    3. **All waypoints must form a straight or smoothly curved path towards the target without deviating away**.
    4. The final waypoint must be exactly at the target object's position.
    5. Do not include the current position in the waypoints.
    6. Each waypoint should be at least {waypoint_spacing} meters apart from the previous one to avoid redundancy.
    7. Ensure that the X-axis and Y-axis values of all waypoints remain within the corridor's boundaries without deviation.
    8. Avoid the detected obstacles by maintaining a safe distance of at least {safe_margin} meters from them.

    Do not include any comments, annotations, or additional text. The output should be a valid JSON array of waypoints only.

    Format the output strictly as a JSON array of coordinates without any comments or additional text.

    Example:
    [
      {{"x": 1.25, "y": 2.63}},
      {{"x": 1.25, "y": 5.36}},
      {{"x": 1.25, "y": 7.86}}
    ]

    Waypoints:
    """

    for attempt in range(max_attempts):
        rospy.loginfo(f"Attempt {attempt + 1} to generate waypoints.")
        navigation_output = agents(llm_client, navigation_system_prompt, navigation_user_prompt)
        if navigation_output:
            waypoints = parse_waypoints(navigation_output)
            if waypoints:
                # Validate waypoints here
                waypoints = validate_waypoints(waypoints, environment_data, safe_margin, target_object['position'])
                if waypoints:
                    rospy.loginfo(f"Navigation Agent succeeded on attempt {attempt + 1}")
                    # Optional: Visualize waypoints
                    plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin)
                    return waypoints
        rospy.logwarn(f"Navigation Agent attempt {attempt + 1} failed. Retrying...")
    else:
        rospy.logerr("Navigation Agent failed after maximum attempts.")
        return None


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('llm_path_planner_straight_corridor', anonymous=True)
        safe_margin = 0.5  # meters
        waypoint_spacing = 0.5  # meters
        environment_data = rospy.get_param('/environment_data_provider/environment_data', {})
    except rospy.ROSInterruptException:
        pass
