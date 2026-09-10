# LLM-Based-mobile-robot-path-planning
This repository contains the source code and resources for the research article **"Robust mobile robot path planning via LLM-based dynamic waypoint generation"** by Muhammad Taha Tariq, Congqing Wang, and Yasir Hussain, published in [**(Expert Systems with Applications)**](https://www.sciencedirect.com/science/article/abs/pii/S0957417425012229). This project involves using Large Language Models (LLM) for efficient mobile robot path planning. It integrates AI techniques for real-time navigation and decision-making in complex environments.

[**Link to the Article**](https://www.sciencedirect.com/science/article/abs/pii/S0957417425012229)  

---
![Description of Image](https://github.com/DC1one/LLM-Based-mobile-robot-path-planning/blob/main/LLM%20MB_PP.png)

## 🚀 Quick Start for ROS 2 (Ubuntu 26 / ROS 2 Lyrical)

**For detailed instructions, see [RUNNING_INSTRUCTIONS_ROS2.md](RUNNING_INSTRUCTIONS_ROS2.md)**

### One-Time Setup
```bash
sudo ./setup_ros2.sh
```

### Running the Simulation (5 Terminals)

**Terminal 1 - Ollama Server:**
```bash
ollama serve
```

**Terminal 2 - Gazebo Simulation:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_01_launch.py
```

**Terminal 3 - LLM Components:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

**Terminal 4 - Commands:**
```bash
# Text commands
source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_gazebo test_plan

# OR Voice commands
ros2 run turtlebot3_gazebo voice_command
```

**Terminal 5 - Visualization (Optional):**
```bash
rviz2
```

### Example Commands
- "Go to the window"
- "Navigate to room number 1"
- "Go to main entrance"
- "Room number 2"

---

## Installation

### ROS 1 (Original - Ubuntu 20.04 / Noetic)

To set up the simulation environment:
1. Clone the required packages (`turtlebot3(https://wiki.ros.org/turtlebot3)`, `turtlebot3_gazebo(https://wiki.ros.org/turtlebot3_gazebo)`) and their dependencies in your ROS workspace.
2. Clone this repository, moving the `turtlebot3_gazebo` and `turtlebot3_description` folders into the corresponding TurtleBot3 packages.
3. Pull Ollama models llama3.1, Mathstral, and Qwen2.5 from Ollama library(https://ollama.com/).
4. Install Google Speech Recognition API.

#### Compile the workspace:
```bash
catkin_make
source devel/setup.bash
```

Ensure there are no errors if dependencies are installed correctly.

### ROS 2 (Ubuntu 26 / Lyrical)

See [RUNNING_INSTRUCTIONS_ROS2.md](RUNNING_INSTRUCTIONS_ROS2.md) for complete setup instructions.

Key differences:
- Uses `colcon` instead of `catkin_make`
- Python scripts migrated from `rospy` to `rclpy`
- Launch files converted to Python format
- TF operations use `tf2_ros` instead of `tf`

---

## Contents
- **turtlebot3_description**: Core files to run TurtleBot3 in Gazebo with the same settings used in our work.
- **turtlebot3_simulations**: Gazebo simulation launch files, scripts, and world configurations.

## For Starting the simulations

### ROS 1 (Noetic)

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

### ROS 2 (Lyrical)

See the Quick Start section above or [RUNNING_INSTRUCTIONS_ROS2.md](RUNNING_INSTRUCTIONS_ROS2.md) for complete ROS 2 instructions.

**Key ROS 2 Commands:**
```bash
# Launch Gazebo
ros2 launch turtlebot3_gazebo corridor_01_launch.py

# Launch LLM components
ros2 launch turtlebot3_gazebo corridor_1_launch.py

# Text commands
ros2 run turtlebot3_gazebo test_plan

# Voice commands
ros2 run turtlebot3_gazebo voice_command

# Visualization
rviz2
```

**For testing different LLM models from Ollama library**
Change the model name (qwen2.5, llama3.1, mathstral) in environment_** scripts in the utils folder.

## Setup Information

### ROS 1 (Original)
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Python**: 3.8
- **Gazebo**: 11.x

### ROS 2 (Updated for Ubuntu 26)
- **OS**: Ubuntu 26.04
- **ROS**: ROS 2 Lyrical
- **Python**: 3.12+
- **Gazebo**: Garden/Ignition or Classic Gazebo 11

---
## Citation:
If you find this work helpful, please cite it as:
```bash
@article{tariq2025robust,
  title={Robust mobile robot path planning via LLM-based dynamic waypoint generation},
  author={Tariq, Muhammad Taha and Hussain, Yasir and Wang, Congqing},
  journal={Expert Systems with Applications},
  volume={282},
  pages={127600},
  year={2025},
  publisher={Elsevier}
}
```
