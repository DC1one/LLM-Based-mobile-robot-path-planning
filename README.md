# LLM-Based-mobile-robot-path-planning
This repository contains the source code and resources for the research article **"Robust and Efficient Mobile Robot Navigation via LLM-Based Dynamic Waypoint Generation"** by Muhammad Taha Tariq, Congqing Wang, and Yasir Hussain, Submitted in [**(Expert Systems with Applications)**] and a preprint is available on [ArXiv](https://arxiv.org/abs/2501.15901). This project involves using Large Language Models (LLM) for efficient mobile robot path planning. It integrates AI techniques for real-time navigation and decision-making in complex environments.

[**Link to the Article**](https://arxiv.org/abs/2501.15901)  

---
![Description of Image](https://github.com/DC1one/LLM-Based-mobile-robot-path-planning/blob/main/LLM%20MB_PP.png)

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
roslaunch turtlebot3_gazebo corridor_01.launch
roslaunch turtlebot3_gazebo corridor_02.launch
roslaunch turtlebot3_gazebo corridor_world_building_04.launch
```
**Start Ollama server**
```bash
ollama serve
```
**Launch Scripts**
```bash
roslaunch turtlebot3_gazebo corridor_1.launch
roslaunch turtlebot3_gazebo corridor_2.launch
roslaunch turtlebot3_gazebo corridor_world_building.launch
```
**Run Rviz to visualize information for different topics**
Add Maker Array, Map, TF, and, Robot model (any other topics as you wish) 
```bash
rviz -d /catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/rviz/turtlebot3_gazebo_model.rviz
```
**Run Script for giving Commands**
Example commands (Room Number ***, Navigate to Room Number ***, Go to (window, main entrance, or stairs*)
For testing Voice-based commands run: for example
```bash
rosrun turtlebot3_gazebo voice_command.py
```
For testing Text-based command run:
```bash
rosrun turtlebot3_gazebo test_plan.py
```
**For testing different LLM models from Ollama library**
Change the model name (qwen2.5, llama3.1, mathstral) in environment_** scripts in the utils folder.

## Setup Informations
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Python**: 3.12
- **Gazebo**: 11.x

## Citation:
If you use this work, please cite it as:
```bash
@misc{tariq2025robustmobilerobotpath,
      title={Robust Mobile Robot Path Planning via LLM-Based Dynamic Waypoint Generation}, 
      author={Muhammad Taha Tariq and Congqing Wang and Yasir Hussain},
      year={2025},
      eprint={2501.15901},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2501.15901}, 
}
```
