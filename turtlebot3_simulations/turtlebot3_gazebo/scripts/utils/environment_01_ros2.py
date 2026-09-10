#!/usr/bin/env python3
# ROS 2 version of environment_01

import rclpy
import json
import re
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import os


def validate_waypoints(waypoints, environment_data, safe_margin, target_position):
    corridor_length = environment_data.get('corridor_length', 15.0)
    corridor_width = environment_data.get('corridor_width', 4.0)
    
    corridor_x_min = -corridor_width / 2 + safe_margin
    corridor_x_max = corridor_width / 2 - safe_margin
    corridor_y_min = 0.0
    corridor_y_max = corridor_length - safe_margin
    
    valid_waypoints = []
    for waypoint in waypoints:
        x = waypoint['x']
        y = waypoint['y']
        
        if not (corridor_x_min <= x <= corridor_x_max and 
                corridor_y_min <= y <= corridor_y_max):
            continue
        
        valid_waypoints.append(waypoint)
    
    return valid_waypoints


def parse_llm_response(llm_response):
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
    corridor_length = environment_data.get('corridor_length', 15.0)
    corridor_width = environment_data.get('corridor_width', 4.0)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    corridor_rect = plt.Rectangle((-corridor_width/2, 0), corridor_width, corridor_length,
                                   fill=False, linewidth=2, edgecolor='blue')
    ax.add_patch(corridor_rect)
    
    objects = environment_data.get('objects', [])
    for obj in objects:
        pos = obj.get('position', {})
        size = obj.get('size', None)
        name = obj.get('name', '')
        
        if size:
            rect = plt.Rectangle(
                (pos['x'] - size['width']/2, pos['y'] - size['length']/2),
                size['width'], size['length'],
                fill=True, alpha=0.5, color='red'
            )
            ax.add_patch(rect)
            ax.text(pos['x'], pos['y'], name, ha='center', va='center', color='white')
        else:
            ax.plot(pos['x'], pos['y'], 'go', markersize=10)
            ax.text(pos['x'], pos['y']+0.2, name, ha='center', va='bottom', fontsize=8)
    
    if waypoints:
        x_coords = [wp['x'] for wp in waypoints]
        y_coords = [wp['y'] for wp in waypoints]
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, marker='o', markersize=5, label='Path')
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=15, label='Start')
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=15, label='Goal')
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Corridor Environment with Planned Path')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')
    
    plt.savefig(output_path)
    plt.close()
    print(f"Environment plot saved to {output_path}")


def get_environment_prompt(environment_data):
    prompt = "You are a path planning assistant for a mobile robot in a corridor environment.\n\n"
    prompt += f"Corridor dimensions:\n"
    prompt += f"- Length: {environment_data.get('corridor_length', 15.0)} meters (Y-axis)\n"
    prompt += f"- Width: {environment_data.get('corridor_width', 4.0)} meters (X-axis)\n\n"
    
    objects = environment_data.get('objects', [])
    if objects:
        prompt += "Objects in the environment:\n"
        for obj in objects:
            name = obj.get('name', 'unknown')
            pos = obj.get('position', {})
            prompt += f"- {name}: located at ({pos.get('x', 0)}, {pos.get('y', 0)})\n"
    
    prompt += "\nGenerate a JSON response with waypoints for the robot to navigate.\n"
    prompt += "Each waypoint should have 'x' and 'y' coordinates.\n"
    prompt += "Response format: {\"waypoints\": [{\"x\": 0.0, \"y\": 0.0}, ...]}\n"
    
    return prompt
