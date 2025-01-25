# LLM-Based-mobile-robot-path-planning
This repository contains the source code and resources for the research article **"Robust and Efficient Mobile Robot Navigation via LLM-Based Dynamic Waypoint Generation"** by Muhammad Taha Tariq, Congqing Wang, and Yasir Hussain, Submitted in [**(****)**]. This project involves using Large Language Models (LLM) for efficient mobile robot path planning. It integrates AI techniques for real-time navigation and decision-making in complex environments.

[**Link to the Article**]()  

---

## Installation
To set up the simulation environment:
1. Clone the required packages (`turtlebot3(https://wiki.ros.org/turtlebot3)`, `turtlebot3_gazebo(https://wiki.ros.org/turtlebot3_gazebo)`) and their dependencies in your ROS workspace.
2. Clone this repository, moving the `turtlebot3_gazebo` and `turtlebot3_description` folders into the corresponding TurtleBot3 packages.
3. Pull Ollama models llama3.1, Mathstral, and Qwen2.5 from Ollama library(https://ollama.com/).
4. Install Google Speech Recognition API.

### Compile the workspace:
```bash
catkin_make
source devel/setup.bash
```

Ensure there are no errors if dependencies are installed correctly.
## Contents
- **turtlebot3_description**: Core files to run TurtleBot3 in Gazebo with the same settings used in our work.
- **turtlebot3_simulations**: Gazebo simulation launch files, scripts, and world configurations.

## For Starting the simulations
**Start ROSCORE**
```bash
roscore
```
**Launch the Gazebo World**
```bash
roslaunch turtlebot3_gazebo turtlebot3_custom_normal.launch
roslaunch turtlebot3_gazebo turtlebot3_custom_crowd_static.launch
roslaunch turtlebot3_gazebo turtlebot3_custom_crowd_dense.launch
```
**Place the Robot in the World**
```bash
roslaunch turtlebot3_gazebo put_robot_in_world_custom.launch
```
**Simulate Environment**
```bash
rosrun turtlebot3_rl_sim simulate_crowd_custom.py
```
**Train Models**
Start training models (SAC-L(CP), TD3, DDPG, or SAC):
```bash
roslaunch turtlebot3_rl_sim start_sac_lagrangian_training.launch
roslaunch turtlebot3_rl_sim start_ddpg_training.launch
roslaunch turtlebot3_rl_sim start_td3_training.launch
roslaunch turtlebot3_rl_sim start_sac_training.launch
```

## Setup Informations
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Python**: 3.12
- **Gazebo**: 11.x

## Citation:
If you use this work, please cite it as:
```bash

```
