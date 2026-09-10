#!/usr/bin/env python3
# ROS 2 version of environment_03 for corridor_world_building_04 (department)

import rclpy
import json
import re
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt


def validate_waypoints(waypoints, environment_data, safe_margin, target_position):
    """Validate waypoints against environment boundaries"""
    # Department environment has multiple corridors
    main_corridor_length = environment_data.get('main_corridor_length', 5.0)
    main_corridor_width = environment_data.get('main_corridor_width', 18.0)
    corridor_01_length = environment_data.get('corridor_01_length', 29.5)
    corridor_01_width = environment_data.get('corridor_01_width', 5.0)
    corridor_02_length = environment_data.get('corridor_02_length', 29.5)
    corridor_02_width = environment_data.get('corridor_02_width', 5.0)
    
    valid_waypoints = []
    
    for waypoint in waypoints:
        x = waypoint['x']
        y = waypoint['y']
        
        # Check main corridor bounds
        in_main_corridor = (
            -main_corridor_width/2 + safe_margin <= x <= main_corridor_width/2 - safe_margin and
            -main_corridor_length + safe_margin <= y <= safe_margin
        )
        
        # Check corridor_01 bounds (left corridor)
        in_corridor_01 = (
            -corridor_01_width/2 - 6.5 + safe_margin <= x <= -corridor_01_width/2 - 6.5 + corridor_01_width - safe_margin and
            safe_margin <= y <= corridor_01_length - safe_margin
        )
        
        # Check corridor_02 bounds (right corridor)
        in_corridor_02 = (
            corridor_02_width/2 + 6.5 - corridor_02_width + safe_margin <= x <= corridor_02_width/2 + 6.5 - safe_margin and
            safe_margin <= y <= corridor_02_length - safe_margin
        )
        
        if in_main_corridor or in_corridor_01 or in_corridor_02:
            valid_waypoints.append(waypoint)
    
    return valid_waypoints


def parse_llm_response(llm_response):
    """Parse LLM response to extract waypoints"""
    try:
        json_match = re.search(r'\{[\s\S]*\}', llm_response)
        if json_match:
            json_str = json_match.group()
            data = json.loads(json_str)
            
            if 'waypoints' in data:
                waypoints = data['waypoints']
                if isinstance(waypoints, str):
                    waypoints = json.loads(waypoints)
                return waypoints
    except Exception as e:
        print(f"Error parsing LLM response: {e}")
    
    return []


def plot_environment(waypoints, environment_data, output_path):
    """Plot the department environment with planned path"""
    main_corridor_length = environment_data.get('main_corridor_length', 5.0)
    main_corridor_width = environment_data.get('main_corridor_width', 18.0)
    corridor_01_length = environment_data.get('corridor_01_length', 29.5)
    corridor_01_width = environment_data.get('corridor_01_width', 5.0)
    corridor_02_length = environment_data.get('corridor_02_length', 29.5)
    corridor_02_width = environment_data.get('corridor_02_width', 5.0)
    
    fig, ax = plt.subplots(figsize=(15, 12))
    
    # Draw main corridor
    main_rect = plt.Rectangle(
        (-main_corridor_width/2, -main_corridor_length),
        main_corridor_width, main_corridor_length,
        fill=False, linewidth=2, edgecolor='blue', label='Main Corridor'
    )
    ax.add_patch(main_rect)
    
    # Draw corridor_01 (left)
    corridor_01_rect = plt.Rectangle(
        (-corridor_01_width/2 - 6.5, 0),
        corridor_01_width, corridor_01_length,
        fill=False, linewidth=2, edgecolor='green', label='Corridor 01'
    )
    ax.add_patch(corridor_01_rect)
    
    # Draw corridor_02 (right)
    corridor_02_rect = plt.Rectangle(
        (corridor_02_width/2 + 6.5 - corridor_02_width, 0),
        corridor_02_width, corridor_02_length,
        fill=False, linewidth=2, edgecolor='orange', label='Corridor 02'
    )
    ax.add_patch(corridor_02_rect)
    
    # Draw objects
    objects = environment_data.get('objects', [])
    for obj in objects:
        pos = obj.get('position', {})
        size = obj.get('size', None)
        name = obj.get('name', '')
        
        if size:
            rect = plt.Rectangle(
                (pos['x'] - size.get('width', 0)/2, pos['y'] - size.get('length', 0)/2),
                size.get('width', 0), size.get('length', 0),
                fill=True, alpha=0.5, color='red'
            )
            ax.add_patch(rect)
            ax.text(pos['x'], pos['y'], name, ha='center', va='center', color='white', fontsize=6)
        else:
            ax.plot(pos['x'], pos['y'], 'go', markersize=8)
            ax.text(pos['x'], pos['y']+0.3, name, ha='center', va='bottom', fontsize=7)
    
    # Draw waypoints path
    if waypoints:
        x_coords = [wp['x'] for wp in waypoints]
        y_coords = [wp['y'] for wp in waypoints]
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, marker='o', markersize=5, label='Path')
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=15, label='Start')
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=15, label='Goal')
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Department Environment with Planned Path')
    ax.legend(loc='upper right')
    ax.grid(True)
    ax.set_aspect('equal')
    
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Environment plot saved to {output_path}")


def get_environment_prompt(environment_data):
    """Generate prompt for LLM based on environment data"""
    prompt = "You are a path planning assistant for a mobile robot in a department building environment.\n\n"
    prompt += "The environment consists of:\n"
    prompt += f"- Main corridor: {environment_data.get('main_corridor_width', 18.0)}m wide x {environment_data.get('main_corridor_length', 5.0)}m long\n"
    prompt += f"- Corridor 01 (left): {environment_data.get('corridor_01_width', 5.0)}m wide x {environment_data.get('corridor_01_length', 29.5)}m long\n"
    prompt += f"- Corridor 02 (right): {environment_data.get('corridor_02_width', 5.0)}m wide x {environment_data.get('corridor_02_length', 29.5)}m long\n\n"
    
    objects = environment_data.get('objects', [])
    if objects:
        prompt += "Key locations in the environment:\n"
        room_plates = [obj for obj in objects if 'Room_number_plate' in obj.get('name', '')]
        for obj in room_plates[:10]:  # Limit to first 10 to keep prompt manageable
            name = obj.get('name', 'unknown')
            pos = obj.get('position', {})
            prompt += f"- {name}: located at ({pos.get('x', 0)}, {pos.get('y', 0)})\n"
        if len(room_plates) > 10:
            prompt += f"... and {len(room_plates) - 10} more rooms\n"
        
        other_locations = [obj for obj in objects if 'Room_number_plate' not in obj.get('name', '') and 'Wall' not in obj.get('name', '')]
        if other_locations:
            prompt += "\nOther important locations:\n"
            for obj in other_locations[:5]:  # Limit to first 5
                name = obj.get('name', 'unknown')
                pos = obj.get('position', {})
                prompt += f"- {name}: located at ({pos.get('x', 0)}, {pos.get('y', 0)})\n"
    
    prompt += "\nGenerate a JSON response with waypoints for the robot to navigate.\n"
    prompt += "Each waypoint should have 'x' and 'y' coordinates.\n"
    prompt += "Response format: {\"waypoints\": [{\"x\": 0.0, \"y\": 0.0}, ...]}\n"
    prompt += "Ensure waypoints avoid walls and obstacles.\n"
    
    return prompt
