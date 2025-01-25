#!/usr/bin/env python3

import rospy
import json
import re
import matplotlib.pyplot as plt
import os

def validate_waypoints(waypoints, environment_data, safe_margin, target_position):
    # Extract corridor dimensions from environment_data
    main_corridor_width = environment_data.get('main_corridor_width', 18.0)
    main_corridor_length = environment_data.get('main_corridor_length', 5.0)
    corridor_01_length = environment_data.get('corridor_01_length', 29.5)
    corridor_01_width = environment_data.get('corridor_01_width', 5.0)
    corridor_02_length = environment_data.get('corridor_02_length', 29.5)
    corridor_02_width = environment_data.get('corridor_02_width', 5.0)

    # Calculate boundaries for Main Corridor
    main_corridor_x_min = -main_corridor_width / 2 + safe_margin  # -8.5
    main_corridor_x_max = main_corridor_width / 2 - safe_margin  # 8.5
    main_corridor_y_min = -0.5  # -0.5
    main_corridor_y_max = -5.0 + safe_margin  # -4.5

    # Calculate boundaries for Corridor_01
    corridor_01_x_min = -corridor_01_width * 2 + 1.5  # -8.5
    corridor_01_x_max = -corridor_01_width + safe_margin  # -4.5
    corridor_01_y_min = 0.5  # 0.5
    corridor_01_y_max = corridor_01_length - 1.0  # 28.5

    # Calculate boundaries for Corridor_02
    corridor_02_x_min = corridor_02_width - safe_margin  # 4.5
    corridor_02_x_max = corridor_01_width * 2 - 1.5  # 8.5
    corridor_02_y_min = 0.5  # 0.5
    corridor_02_y_max = corridor_02_length - safe_margin  # 28.5

    rospy.loginfo("Validating waypoints within corridor boundaries:")
    rospy.loginfo(f"Main Corridor X-axis: {main_corridor_x_min} <= x <= {main_corridor_x_max}")
    rospy.loginfo(f"Main Corridor Y-axis: {main_corridor_y_min} <= y <= {main_corridor_y_max}")
    rospy.loginfo(f"Corridor_01 X-axis: {corridor_01_x_min} <= x <= {corridor_01_x_max}")
    rospy.loginfo(f"Corridor_01 Y-axis: {corridor_01_y_min} <= y <= {corridor_01_y_max}")
    rospy.loginfo(f"Corridor_02 X-axis: {corridor_02_x_min} <= x <= {corridor_02_x_max}")
    rospy.loginfo(f"Corridor_02 Y-axis: {corridor_02_y_min} <= y <= {corridor_02_y_max}")

    valid_waypoints = []
    for waypoint in waypoints:
        x = waypoint['x']
        y = waypoint['y']

        # Check if waypoint is within Main Corridor
        if (main_corridor_x_min <= x <= main_corridor_x_max and
                main_corridor_y_max <= y <= main_corridor_y_min):
            valid_waypoints.append(waypoint)
            rospy.loginfo(f"Waypoint ({x}, {y}) is within Main Corridor.")
        # Check if waypoint is within Corridor_01
        elif (corridor_01_x_min <= x <= corridor_01_x_max and
              corridor_01_y_min <= y <= corridor_01_y_max):
            valid_waypoints.append(waypoint)
            rospy.loginfo(f"Waypoint ({x}, {y}) is within Corridor_01.")
        # Check if waypoint is within Corridor_02
        elif (corridor_02_x_min <= x <= corridor_02_x_max and
              corridor_02_y_min <= y <= corridor_02_y_max):
            valid_waypoints.append(waypoint)
            rospy.loginfo(f"Waypoint ({x}, {y}) is within Corridor_02.")
        else:
            rospy.logwarn(f"Waypoint ({x}, {y}) is outside corridor boundaries and will be ignored.")
            continue

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
    # corridor boundaries
    corridors = {
        'Main_Corridor': {
            'x_min': -8.5,
            'x_max': 8.5,
            'y_min': -4.5,
            'y_max': -0.0
        },
        'Corridor_01': {
            'x_min': -8.5,
            'x_max': -4.5,
            'y_min': 0.5,
            'y_max': 28.5
        },
        'Corridor_02': {
            'x_min': 4.5,
            'x_max': 8.5,
            'y_min': 0.5,
            'y_max': 28.5
        }
    }

    x = position['x']
    y = position['y']

    # rospy.loginfo(f"Checking if position ({x}, {y}) is within any corridor.")
    for corridor_name, bounds in corridors.items():
        # rospy.loginfo(f"Checking {corridor_name}:")
        # rospy.loginfo(f"  X-axis: {bounds['x_min']} <= {x} <= {bounds['x_max']}")
        # rospy.loginfo(f"  Y-axis: {bounds['y_min']} <= {y} <= {bounds['y_max']}")
        if (bounds['x_min'] <= x <= bounds['x_max'] and
                bounds['y_min'] <= y <= bounds['y_max']):
            rospy.loginfo(f"Position ({x}, {y}) is in {corridor_name}.")
            return corridor_name
    rospy.logwarn(f"Position ({x}, {y}) does not lie within any defined corridor.")
    return None


def plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin, junction_point=None):
    main_corridor_width = environment_data.get('main_corridor_width', 18.0)
    main_corridor_length = environment_data.get('main_corridor_length', 5.0)
    corridor_01_length = environment_data.get('corridor_01_length', 29.5)
    corridor_01_width = environment_data.get('corridor_01_width', 5.0)
    corridor_02_length = environment_data.get('corridor_02_length', 29.5)
    corridor_02_width = environment_data.get('corridor_02_width', 5.0)

    # Calculate boundaries for Main Corridor
    main_corridor_x_min = -main_corridor_width / 2 + safe_margin
    main_corridor_x_max = main_corridor_width / 2 - safe_margin
    main_corridor_y_min = -0.5
    main_corridor_y_max = -5.0 + safe_margin

    # Calculate boundaries for Corridor_01
    corridor_01_x_min = -corridor_01_width * 2 + 1.5
    corridor_01_x_max = -corridor_01_width + safe_margin
    corridor_01_y_min = 0.5
    corridor_01_y_max = corridor_01_length - 1.0

    # Calculate boundaries for Corridor_02
    corridor_02_x_min = corridor_02_width - safe_margin
    corridor_02_x_max = corridor_01_width * 2 - 1.5
    corridor_02_y_min = 0.5
    corridor_02_y_max = corridor_02_length - safe_margin

    rospy.loginfo("Plotting waypoints and corridor boundaries.")

    # Set fixed figure size (e.g., 10 inches by 8 inches)
    fig, ax = plt.subplots(figsize=(7, 6))  # Adjust the size as needed

    # Draw Main Corridor
    main_corridor = plt.Rectangle((main_corridor_x_min, main_corridor_y_min),
                                  main_corridor_x_max - main_corridor_x_min,
                                  main_corridor_y_max - main_corridor_y_min,
                                  edgecolor='blue', facecolor='lightblue', alpha=0.5, label='Main Corridor')
    ax.add_patch(main_corridor)

    # Draw Corridor_01
    corridor_01 = plt.Rectangle((corridor_01_x_min, corridor_01_y_min),
                                corridor_01_x_max - corridor_01_x_min,
                                corridor_01_y_max - corridor_01_y_min,
                                edgecolor='green', facecolor='lightgreen', alpha=0.5, label='Corridor_01')
    ax.add_patch(corridor_01)

    # Draw Corridor_02
    corridor_02 = plt.Rectangle((corridor_02_x_min, corridor_02_y_min),
                                corridor_02_x_max - corridor_02_x_min,
                                corridor_02_y_max - corridor_02_y_min,
                                edgecolor='purple', facecolor='thistle', alpha=0.5, label='Corridor_02')
    ax.add_patch(corridor_02)

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

    # Plot Junction Point if it exists
    if junction_point:
        ax.plot(junction_point[0], junction_point[1], 'mo', label='Junction Point')
        #ax.plot(junction_point[0], junction_point[2], 'mo', label='Junction Point')

    ax.set_xlabel('X-axis (meters)', fontweight='bold')
    ax.set_ylabel('Y-axis (meters)', fontweight='bold')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True)
    plt.axis('equal')
    save_dir = '/home/albert/Pictures'
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    filename = os.path.join(save_dir, 'E1.png')
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.show()


def generate_waypoints_with_navigation_agent(llm_client, current_pose, target_object, environment_data, safe_margin,
                                             waypoint_spacing):
    max_attempts = 3  # Maximum number of attempts per agent

    # Extract corridor dimensions
    main_corridor_width = environment_data.get('main_corridor_width', 18.0)
    main_corridor_length = environment_data.get('main_corridor_length', 5.0)
    corridor_01_length = environment_data.get('corridor_01_length', 29.5)
    corridor_01_width = environment_data.get('corridor_01_width', 5.0)
    corridor_02_length = environment_data.get('corridor_02_length', 29.5)
    corridor_02_width = environment_data.get('corridor_02_width', 5.0)

    # Calculate junction points
    junction_point_1 = (-6.5, -0.5)
    junction_point_2 = (6.5, -0.5)

    # Calculate boundaries for Main Corridor
    main_corridor_x_min = -main_corridor_width / 2 + safe_margin  # -8.5
    main_corridor_x_max = main_corridor_width / 2 - safe_margin  # 8.5
    main_corridor_y_min = -0.5  # -0.5
    main_corridor_y_max = -5.0 + safe_margin  # -4.5

    # Calculate boundaries for Corridor_01
    corridor_01_x_min = -corridor_01_width * 2 + 1.5  # -8.5
    corridor_01_x_max = -corridor_01_width + safe_margin  # -4.5
    corridor_01_y_min = 0.5  # 0.5
    corridor_01_y_max = corridor_01_length - 1.0  # 28.5

    # Calculate boundaries for Corridor_02
    corridor_02_x_min = corridor_02_width - safe_margin  # 4.5
    corridor_02_x_max = corridor_01_width * 2 - 1.5  # 8.5
    corridor_02_y_min = 0.5  # 0.5
    corridor_02_y_max = corridor_02_length - safe_margin  # 28.5

    # Determine robot's current corridor
    current_position = {
        'x': current_pose.pose.position.x,
        'y': current_pose.pose.position.y
    }
    rospy.loginfo(f"Determining current corridor for position ({current_position['x']}, {current_position['y']}).")
    current_corridor = determine_corridor(current_position)

    if current_corridor is None:
        rospy.logerr("Cannot determine the current corridor of the robot.")
        return None

    # Determine which corridor the target is in
    target_x = target_object['position']['x']
    target_y = target_object['position']['y']

    rospy.loginfo(f"Determined Target Position: x={target_x}, y={target_y}")

    target_position = {
        'x': target_x,
        'y': target_y
    }

    target_corridor = determine_corridor(target_position)

    if target_corridor is None:
        rospy.logerr("Target position does not lie within any defined corridor.")
        junction_point = None
    else:
        if current_corridor == target_corridor:
            junction_point = None
            rospy.loginfo("Robot and target are in the same corridor. No junction point required.")
        else:
            # Assign junction point based on the current corridor and target corridor
            if current_corridor == 'Main_Corridor':
                if target_corridor == 'Corridor_01':
                    junction_point = junction_point_1
                elif target_corridor == 'Corridor_02':
                    junction_point = junction_point_2
                else:
                    rospy.logwarn(f"No junction point defined for target corridor: {target_corridor}.")
                    junction_point = None
            else:
                # For non-Main Corridors, assign junction point based on current corridor
                if current_corridor == 'Corridor_01':
                    junction_point = junction_point_1
                elif current_corridor == 'Corridor_02':
                    junction_point = junction_point_2
                else:
                    rospy.logwarn(f"No junction point defined for current corridor: {current_corridor}.")
                    junction_point = None

    # Handle junction point string
    if junction_point:
        junction_point_str = f"{junction_point[0]:.2f}, {junction_point[1]:.2f}"
        rospy.loginfo(f"Using Junction Point: {junction_point_str}")
    else:
        junction_point_str = "None"
        rospy.loginfo("No Junction Point required for this navigation.")

    # Construct system and user prompts
    navigation_system_prompt = f"""
Robot's current position: ({current_position['x']:.2f}, {current_position['y']:.2f})
Target position: ({target_x:.2f}, {target_y:.2f})
Corridor boundaries:

Main Corridor:
- X-axis: from {main_corridor_x_min:.2f} to {main_corridor_x_max:.2f}
- Y-axis: from {main_corridor_y_min:.2f} to {main_corridor_y_max:.2f}

Corridor_01:
- X-axis: from {corridor_01_x_min:.2f} to {corridor_01_x_max:.2f}
- Y-axis: from {corridor_01_y_min:.2f} to {corridor_01_y_max:.2f}

Corridor_02:
- X-axis: from {corridor_02_x_min:.2f} to {corridor_02_x_max:.2f}
- Y-axis: from {corridor_02_y_min:.2f} to {corridor_02_y_max:.2f}

Safe margin: {safe_margin} meters from walls

You are a navigation assistant for a mobile robot in a U-shaped corridor map with three corridors: Main Corridor, Corridor_01, and Corridor_02.
Provide a concise sequence of waypoints as (x, y) coordinates for the robot to follow to reach the destination, avoiding obstacles and maintaining the safe margins defined above. Ensure that:
1. All waypoints are within the corridor boundaries defined above.
2. Help in creating a list of waypoints from the robot's current position to the target position in the best sequence on both axis.
3. Room_number_plates from 101 to 114 are in Corridor_01, Room_number_plates 201 to 218 and windows are in Corridor_02, and other objects like Stairs are in the Main Corridor.
4. If the robot needs to move from one corridor to another, include the appropriate junction point ({junction_point_str}) in the waypoints as the first waypoint and the next waypoint must be a junction point ({junction_point_str}) of the corridor where the target is located.
5. Ensure the path is direct and efficient, facilitating smooth transitions between corridors when necessary.
6. Do not include any comments, annotations, or additional text. The output should be a valid JSON array of waypoints only.
7. Format the output strictly as a JSON array of coordinates without any comments or additional text.
8. Please follow all rules; it's a humble request.
"""

    navigation_user_prompt = f"""

Robot's current position: ({current_position['x']:.2f}, {current_position['y']:.2f})
Target position: ({target_x:.2f}, {target_y:.2f})

Safe margin: {safe_margin} meters from walls

Please provide a concise sequence of waypoints as (x, y) coordinates for the robot to follow to reach the destination, avoiding obstacles and maintaining
the safe margins defined above. Ensure that:
1. The waypoints form a continuous and logical path towards the destination that reduces the distance between the robot's current position and the target object without any backtracking.
2. Limit the number of waypoints to a maximum of 6 for efficiency.
3. **All waypoints must form a straight or smoothly curved path towards the target without deviating away**.
4. The final waypoint must be exactly at the target object's position.
5. Do not include the current position in the waypoints.
6. Each waypoint should be at least {waypoint_spacing} meters apart from the previous one to avoid redundancy.
7. Ensure that the X-axis and Y-axis values of all waypoints remain within the corridor's boundaries without deviation.
8. {'If the robot needs to move from one corridor to another, include the appropriate junction point (' + junction_point_str + ') in the waypoints as the first waypoint; otherwise, generate waypoints normally as you are generating.' if junction_point else 'Generate waypoints directly towards the target without using any junction points.'}
9. Please follow all rules; it's a humble request.

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
                    try:
                        plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin,
                                       junction_point)
                    except Exception as e:
                        rospy.logerr(f"Failed to plot waypoints: {e}")
                    return waypoints
        rospy.logwarn(f"Navigation Agent attempt {attempt + 1} failed. Retrying...")
    else:
        rospy.logerr("Navigation Agent failed after maximum attempts.")
        return None


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('llm_path_planner_u_shape', anonymous=True)
        safe_margin = 0.5  # meters
        waypoint_spacing = 1.0  # meters
        environment_data = rospy.get_param('/environment_data_provider/environment_data', {})

    except rospy.ROSInterruptException:
        pass
