# Step-by-Step Guide to Run LLM-Based Mobile Robot Path Planning on ROS 2

## Overview
This project uses Large Language Models (LLMs) to generate waypoints for TurtleBot3 navigation in Gazebo simulation. The system requires multiple terminal windows and proper ROS 2 setup.

## Prerequisites Check

Before starting, ensure you have:
- Ubuntu 26.04 (Noble Numbat)
- ROS 2 Jazzy or Rolling Ridley installed (Jazzy is the LTS version for Ubuntu 24.04+, Rolling for latest Ubuntu versions)
- Python 3.12+ (default in Ubuntu 26.04)
- Ollama installed with LLM models
- Git and colcon build tools

---

## Step 1: Install System Dependencies (One-Time Setup)

### 1.1 Update Package Lists
```bash
sudo apt update
```

### 1.2 Install Required Packages
```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release python3-pip python3-yaml python3-numpy python3-matplotlib python3-venv
```

### 1.3 Install Python Dependencies
For Ubuntu 26.04, use pip with --break-system-packages flag or create a virtual environment:
```bash
# Option A: System-wide installation (use --break-system-packages for Ubuntu 26.04+)
pip3 install --break-system-packages ollama speechrecognition noisereduce pyyaml

# Option B: Virtual environment (recommended)
python3 -m venv ~/llm_robot_env
source ~/llm_robot_env/bin/activate
pip install ollama speechrecognition noisereduce pyyaml
```

### 1.4 Install and Setup Ollama
```bash
# Install Ollama (works on Ubuntu 26.04)
curl -fsSL https://ollama.com/install.sh | sh

# Start Ollama service
systemctl --user enable ollama
systemctl --user start ollama

# If systemctl doesn't work, run Ollama directly:
# ollama serve  # Run in Terminal 1 later

# Pull required LLM models (this may take several minutes)
ollama pull llama3.1
ollama pull qwen2.5
ollama pull mathstral
```

### 1.5 Configure TurtleBot3 Model
Add to your `~/.bashrc`:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2: Setup ROS 2 Workspace

### 2.1 Create Workspace Directory
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2.2 Copy Project Files
```bash
cp -r /workspace/turtlebot3_simulations ~/ros2_ws/src/
cp -r /workspace/turtlebot3_description ~/ros2_ws/src/
```

### 2.3 CRITICAL: Replace ROS 1 Scripts with ROS 2 Versions

The project contains both ROS 1 (`rospy`) and ROS 2 (`rclpy`) versions of scripts. We need to use the ROS 2 versions.

**ROS 2 Scripts Available:**
- `environment_data_provider_01_ros2.py` → `environment_data_provider_01.py`
- `llm_path_planner_ros2.py` → `llm_path_planner.py`
- `test_plan_ros2.py` → `test_plan.py`
- `waypoint_executor_ros2.py` → `waypoint_executor.py` (newly created)
- `object_marker_publisher_ros2.py` → `object_marker_publisher.py` (newly created)
- `metrics_collector_ros2.py` → `metrics_collector.py` (newly created)
- `utils/environment_01_ros2.py` → `utils/environment_01.py` (newly created)

```bash
cd ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/scripts

# Backup original ROS 1 scripts
mkdir -p ros1_backup
cp *.py ros1_backup/ 2>/dev/null || true
cp utils/*.py ros1_backup/utils/ 2>/dev/null || true

# Replace with ROS 2 versions
cp environment_data_provider_01_ros2.py environment_data_provider_01.py
cp llm_path_planner_ros2.py llm_path_planner.py
cp test_plan_ros2.py test_plan.py
cp waypoint_executor_ros2.py waypoint_executor.py
cp object_marker_publisher_ros2.py object_marker_publisher.py
cp metrics_collector_ros2.py metrics_collector.py
cp utils/environment_01_ros2.py utils/environment_01.py
```

---

## Step 3: Update CMakeLists.txt

Edit the CMakeLists.txt to install the correct script versions:

```bash
cd ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo
```

Update the `CMakeLists.txt` file to reference the correct Python scripts (the ROS 2 versions).

---

## Step 4: Build the Workspace

```bash
cd ~/ros2_ws
# For Ubuntu 26.04, use ROS 2 Jazzy or Rolling
source /opt/ros/jazzy/setup.bash  # or 'rolling' depending on your ROS 2 version
colcon build --symlink-install
```

If you encounter errors:
- Check that all Python scripts use `rclpy` instead of `rospy`
- Verify package dependencies are installed
- Check for syntax errors in Python scripts
- For Python 3.12+ compatibility, ensure no deprecated Python features are used

Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 5: Run the Simulation (5 Terminal Windows Required)

Open **5 separate terminal windows** and follow these steps:

### Terminal 1: Start Ollama Server
```bash
# Source ROS 2 (optional but good practice)
source ~/ros2_ws/install/setup.bash

# Start Ollama server (keep this terminal open)
ollama serve
```

Wait until you see "Listening on..." message.

### Terminal 2: Launch Gazebo Simulation
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Launch the corridor_01 world with TurtleBot3
ros2 launch turtlebot3_gazebo corridor_01_launch.py
```

Wait for Gazebo to fully load. You should see:
- The TurtleBot3 robot
- The corridor environment
- No error messages in the terminal

### Terminal 3: Launch LLM Path Planning Components
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Launch all LLM components
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

This starts:
- Environment Data Provider
- LLM Path Planner
- Object Marker Publisher
- Waypoint Executor
- Metrics Collector

Check for any error messages. If scripts fail, they likely still contain ROS 1 code.

### Terminal 4: Send Commands (Text Interface)
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Run text command interface
ros2 run turtlebot3_gazebo test_plan
```

You'll see a prompt: `Enter your command (e.g., 'Go to the window'):`

**Example commands:**
- "Go to the window"
- "Navigate to room number 1"
- "Go to main entrance"
- "Room number 2"

### Terminal 5: RViz2 Visualization (Optional)
```bash
rviz2
```

In RViz2:
1. Set **Fixed Frame** to `map`
2. Add displays:
   - **PoseArray** → Topic: `/llm_waypoints` (planned waypoints)
   - **MarkerArray** → Topic: `/visualization_marker_array` (objects)
   - **TF** → Transform tree
   - **RobotModel** → Robot visualization
   - **LaserScan** → Topic: `/scan` (LiDAR data)

---

## Step 6: Verification and Testing

### 6.1 Check Running Nodes
In a new terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 node list
```

Expected nodes:
- `/environment_data_provider`
- `/llm_path_planner`
- `/waypoint_executor`
- `/object_marker_publisher`
- `/metrics_collector`
- `/test_plan`

### 6.2 Check Topics
```bash
ros2 topic list
```

Key topics:
- `/user_command` - Your commands
- `/llm_waypoints` - Generated waypoints
- `/cmd_vel` - Robot velocity commands
- `/odom` - Odometry data
- `/scan` - Laser scan data
- `/obstacle_data` - Obstacle detection

### 6.3 Monitor Waypoint Generation
```bash
ros2 topic echo /waypoint_generation_status
```

Should show successful waypoint generation attempts.

### 6.4 Check Execution Status
```bash
ros2 topic echo /execution_status
```

Should show `completed` when navigation finishes.

---

## Step 7: Troubleshooting Common Issues

### Issue 1: "Package 'turtlebot3_gazebo' not found"
**Solution:**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # or 'rolling' for Ubuntu 26.04+
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Issue 2: Python Import Errors (ModuleNotFoundError)
**Solution:**
For Ubuntu 26.04 with Python 3.12+:
```bash
# Option A: System-wide with break-system-packages
pip3 install --break-system-packages ollama speechrecognition noisereduce pyyaml numpy matplotlib

# Option B: Use virtual environment (recommended)
source ~/llm_robot_env/bin/activate
pip install ollama speechrecognition noisereduce pyyaml numpy matplotlib
```

### Issue 3: TF Transformation Failures
**Solution:** Ensure static transform publisher is running:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Issue 4: Ollama Connection Refused
**Solutions:**
1. Ensure Ollama is running: `ollama serve`
2. Check models: `ollama list`
3. Restart Ollama: `systemctl --user restart ollama`

### Issue 5: Script Fails with "rospy" Import Error
**Solution:** The script hasn't been migrated to ROS 2. Replace `import rospy` with `import rclpy` and update the node structure. See the `_ros2.py` files for examples.

### Issue 6: Robot Doesn't Move
**Check:**
1. Is `/cmd_vel` publishing? `ros2 topic echo /cmd_vel`
2. Is Gazebo running?
3. Are waypoints generated? `ros2 topic echo /llm_waypoints`
4. Check for obstacles triggering emergency stop

### Issue 7: Waypoint Generation Fails
**Check:**
1. Ollama server is running
2. LLM models downloaded: `ollama list`
3. Environment YAML exists and is valid
4. Check logs with `--show-logs` flag

---

## Step 8: Quick Reference Commands

```bash
# Daily usage workflow:

# Terminal 1
ollama serve

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_01_launch.py

# Terminal 3
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_1_launch.py

# Terminal 4
source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_gazebo test_plan

# Useful debugging:
ros2 node list                    # List nodes
ros2 topic list                   # List topics
ros2 topic echo /topic_name       # Monitor topic
ros2 param dump /node_name        # View parameters
ros2 run tf2_tools view_frames    # Visualize TF tree
```

---

## Expected Workflow

1. Start Ollama server (Terminal 1)
2. Launch Gazebo with robot (Terminal 2)
3. Launch LLM planning components (Terminal 3)
4. Give command like "Go to the window" (Terminal 4)
5. Watch LLM generate waypoints based on environment
6. Robot navigates autonomously avoiding obstacles
7. Monitor progress in RViz2 (Terminal 5)
8. Check metrics after completion

---

## Important Notes

1. **Script Migration**: Most scripts still use ROS 1 (`rospy`). They MUST be migrated to ROS 2 (`rclpy`).

2. **Sample ROS 2 Scripts Provided**:
   - `llm_path_planner_ros2.py` - Fully migrated
   - `environment_data_provider_01_ros2.py` - Fully migrated
   - `test_plan_ros2.py` - Fully migrated

3. **Gazebo Version**: Ubuntu 26.04 uses Gazebo Sim (formerly Ignition) or Gazebo Classic 11. World files may need updates for compatibility with newer Gazebo versions.

4. **ROS 2 Version**: Use ROS 2 Jazzy (Ubuntu 24.04 LTS, works on 26.04) or Rolling Ridley (latest, for Ubuntu 26.04+).

5. **Python Version**: Ubuntu 26.04 comes with Python 3.12+. Ensure all scripts are compatible with Python 3.12 (no deprecated features).

6. **TF2 Usage**: All TF operations must use `tf2_ros` instead of old `tf` package.

7. **PEP 668**: Ubuntu 26.04 enforces PEP 668 which prevents system-wide pip installs. Use `--break-system-packages` flag or virtual environments.

---

## Additional Resources

- ROS 2 Documentation: https://docs.ros.org/en/jazzy/ or https://docs.ros.org/en/rolling/
- ROS 2 Installation for Ubuntu: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
- TurtleBot3 ROS 2: https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/
- Ollama: https://ollama.com/
- Python 3.12 Release Notes: https://docs.python.org/3.12/whatsnew/3.12.html
- PEP 668 (Externally Managed Environments): https://peps.python.org/pep-0668/
- Original Article: https://www.sciencedirect.com/science/article/abs/pii/S0957417425012229

---

## Citation

If you use this work, please cite:

```bibtex
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
