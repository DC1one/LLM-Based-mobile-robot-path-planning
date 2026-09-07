# Complete Instructions to Run LLM-Based Mobile Robot Path Planning on ROS 2 (Ubuntu 26 / ROS 2 Lyrical)

## Prerequisites

Your system should have:
- Ubuntu 26.04
- ROS 2 Lyrical installed
- Python 3.12+
- Ollama installed with required LLM models

## Step 1: Initial Setup (One-Time Installation)

### 1.1 Install ROS 2 Lyrical Dependencies

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
```

### 1.2 Add ROS 2 Repository (if not already added)

```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Lyrical Desktop Full

```bash
sudo apt update
sudo apt install -y ros-lyrical-desktop-full
source /opt/ros/lyrical/setup.bash
```

### 1.4 Install Additional ROS 2 Packages

```bash
sudo apt install -y ros-lyrical-turtlebot3-gazebo \
                   ros-lyrical-turtlebot3-description \
                   ros-lyrical-nav2-map-server \
                   ros-lyrical-slam-toolbox \
                   ros-lyrical-rviz2 \
                   ros-lyrical-tf2-tools \
                   ros-lyrical-tf2-ros \
                   ros-lyrical-tf2-geometry-msgs \
                   ros-lyrical-robot-state-publisher \
                   ros-lyrical-gazebo-ros-pkgs
```

### 1.5 Install Python Dependencies

```bash
sudo apt install -y python3-pip python3-yaml python3-numpy python3-matplotlib
pip3 install ollama speechrecognition noisereduce pyyaml
```

### 1.6 Install and Setup Ollama

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull required LLM models
ollama pull llama3.1
ollama pull qwen2.5
ollama pull mathstral
```

### 1.7 Configure TurtleBot3 Model

Add this to your `~/.bashrc`:
```bash
export TURTLEBOT3_MODEL=burger
```

Then source it:
```bash
source ~/.bashrc
```

---

## Step 2: Workspace Setup

### 2.1 Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2.2 Copy Project Files

```bash
# Copy the project packages to your workspace
cp -r /workspace/turtlebot3_simulations ~/ros2_ws/src/
cp -r /workspace/turtlebot3_description ~/ros2_ws/src/
```

### 2.3 IMPORTANT: Migrate Python Scripts to ROS 2

**CRITICAL NOTE**: Most Python scripts in this project are still written for ROS 1 (using `rospy`). You need to either:

**Option A**: Use the provided ROS 2 versions (already created):
- `llm_path_planner_ros2.py` → Rename to `llm_path_planner.py`
- `environment_data_provider_01_ros2.py` → Rename to `environment_data_provider_01.py`
- `test_plan_ros2.py` → Rename to `test_plan.py`

**Option B**: Manually migrate all scripts from `rospy` to `rclpy`

The following scripts NEED migration (still using rospy):
- `waypoint_executor.py`
- `object_marker_publisher.py`
- `metrics_collector.py`
- `voice_command.py`
- `environment_data_provider_02.py`
- `environment_data_provider_03.py`
- `utils/environment_01.py`
- `utils/environment_02.py`
- `utils/environment_03.py`

See the `ROS2_MIGRATION_GUIDE.md` for detailed migration instructions.

### 2.4 Update CMakeLists.txt for ROS 2 Python Scripts

Ensure the `CMakeLists.txt` installs the correct ROS 2 scripts. Update the install section:

```bash
cd ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo
```

Edit `CMakeLists.txt` to use `_ros2.py` versions or ensure all scripts are migrated.

### 2.5 Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

If you encounter errors, check that all Python scripts are properly migrated to ROS 2.

### 2.6 Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add this to your `~/.bashrc` for convenience:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 3: Running the Simulation

You need **5 separate terminal windows** to run the complete system.

### Terminal 1: Start Ollama Server

```bash
# Source ROS 2 (optional for Ollama, but good practice)
source ~/ros2_ws/install/setup.bash

# Start Ollama server
ollama serve
```

**Keep this terminal open** - Ollama must remain running.

### Terminal 2: Launch Gazebo Simulation

```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Launch the corridor_01 world with TurtleBot3
ros2 launch turtlebot3_gazebo corridor_01_launch.py
```

Wait for Gazebo to fully load (you should see the robot and environment).

### Terminal 3: Launch LLM Path Planning Components

```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Launch all LLM components (environment provider, planner, executor, etc.)
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

**Note**: This will fail if scripts are not properly migrated to ROS 2. Check the output for errors.

### Terminal 4: Send Commands (Voice OR Text)

#### Option A: Text Commands (Recommended for Testing)

```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Run text command interface
ros2 run turtlebot3_gazebo test_plan
```

#### Option B: Voice Commands (Requires Microphone)

```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash

# Run voice command interface
ros2 run turtlebot3_gazebo voice_command
```

**Example Commands:**
- "Go to the window"
- "Navigate to room number 1"
- "Go to main entrance"
- "Room number 2"

### Terminal 5: RViz2 Visualization (Optional)

```bash
rviz2
```

In RViz2:
1. Set Fixed Frame to `map`
2. Add displays:
   - **PoseArray** → Topic: `/llm_waypoints` (to see planned waypoints)
   - **MarkerArray** → Topic: `/visualization_marker_array` (to see objects)
   - **TF** → To see transform tree
   - **RobotModel** → To visualize the robot
   - **LaserScan** → Topic: `/scan` (to see LiDAR data)

---

## Step 4: Testing and Verification

### 4.1 Verify Nodes are Running

In a new terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 node list
```

You should see nodes like:
- `/environment_data_provider`
- `/llm_path_planner`
- `/waypoint_executor`
- `/object_marker_publisher`
- `/metrics_collector`

### 4.2 Verify Topics

```bash
ros2 topic list
```

Key topics to check:
- `/user_command` - Your commands
- `/llm_waypoints` - Generated waypoints
- `/cmd_vel` - Robot velocity commands
- `/odom` - Odometry data
- `/scan` - Laser scan data
- `/obstacle_data` - Obstacle detection status

### 4.3 Monitor Waypoint Generation

```bash
ros2 topic echo /waypoint_generation_status
```

Should show `success:1`, `success:2`, etc., indicating successful waypoint generation attempts.

### 4.4 Check Execution Status

```bash
ros2 topic echo /execution_status
```

Should show `completed` when navigation finishes.

### 4.5 View Metrics

Metrics are saved to CSV files in:
```
~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/results/mathstral/
```

Update the path in `metrics_collector.py` for ROS 2 if needed.

---

## Step 5: Troubleshooting

### Common Issues and Solutions

#### Issue 1: "Package 'turtlebot3_gazebo' not found"

**Solution:**
```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

#### Issue 2: Python Import Errors (ModuleNotFoundError)

**Solution:** Ensure all dependencies are installed:
```bash
pip3 install ollama speechrecognition noisereduce pyyaml numpy matplotlib
```

#### Issue 3: TF Transformation Failures

**Solution:** Check that the static transform publisher is running:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

#### Issue 4: Ollama Connection Refused

**Solution:** 
1. Ensure Ollama server is running (`ollama serve`)
2. Check if models are downloaded: `ollama list`
3. Restart Ollama service if needed

#### Issue 5: Script Fails with "rospy" Import Error

**Solution:** The script hasn't been migrated to ROS 2 yet. You need to:
1. Replace `import rospy` with `import rclpy`
2. Convert node initialization to class-based `Node`
3. Update publishers/subscribers syntax
4. Replace `rospy.Rate` with `create_timer()`
5. Update logging to `self.get_logger()`

See `ROS2_MIGRATION_GUIDE.md` for detailed conversion examples.

#### Issue 6: Robot Doesn't Move

**Check:**
1. Is `/cmd_vel` topic publishing? `ros2 topic echo /cmd_vel`
2. Is Gazebo simulation running?
3. Are waypoints being generated? `ros2 topic echo /llm_waypoints`
4. Check for obstacles triggering emergency stop

#### Issue 7: Waypoint Generation Fails

**Check:**
1. Ollama server is running
2. LLM models are downloaded
3. Environment YAML file exists and is valid
4. Check logs: `ros2 launch turtlebot3_gazebo corridor_1_launch.py --show-logs`

---

## Step 6: Changing Environments

The project supports multiple environments. To switch:

### Corridor 01 (Default)
```bash
# Terminal 2
ros2 launch turtlebot3_gazebo corridor_01_launch.py

# Terminal 3
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

### Corridor 02
Modify `corridor_1_launch.py` to use `corridor_02.yaml` and `environment_data_provider_02.py`

### Department/Building
Use `corridor_world_building.launch.py` (needs to be created for ROS 2)

---

## Step 7: Testing Different LLM Models

To test different LLM models:

1. Open the environment script (e.g., `utils/environment_01.py`)
2. Find the model name in the LLM client call
3. Change to desired model: `llama3.1`, `qwen2.5`, or `mathstral`
4. Rebuild and restart

Example modification in environment script:
```python
response = llm_client.chat(model='qwen2.5', messages=messages)
```

---

## Quick Reference Command Summary

```bash
# Setup (one time)
sudo ./setup_ros2.sh

# Daily usage:
# Terminal 1
ollama serve

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_01_launch.py

# Terminal 3
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo corridor_1_launch.py

# Terminal 4 (Text commands)
source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_gazebo test_plan

# Terminal 5 (Optional visualization)
rviz2

# Useful debugging commands
ros2 node list                    # List all running nodes
ros2 topic list                   # List all topics
ros2 topic echo /topic_name       # Monitor a topic
ros2 param dump /node_name        # View node parameters
ros2 run tf2_tools view_frames    # Visualize TF tree
```

---

## Important Notes

1. **Script Migration Required**: Most Python scripts still use ROS 1 (`rospy`). They MUST be migrated to ROS 2 (`rclpy`) before the system will work.

2. **Sample ROS 2 Scripts Provided**: 
   - `llm_path_planner_ros2.py` - Fully migrated example
   - `environment_data_provider_01_ros2.py` - Fully migrated example
   - `test_plan_ros2.py` - Fully migrated example

3. **Gazebo Version**: Ubuntu 26 may use Gazebo Garden/Ignition instead of classic Gazebo 11. Some world files may need updates.

4. **TF2 Usage**: All TF operations must use `tf2_ros` instead of the old `tf` package.

5. **Parameter Names**: ROS 2 parameter naming conventions differ from ROS 1. Private parameters use `~param` in ROS 1 but require explicit declaration in ROS 2.

6. **QoS Policies**: ROS 2 uses Quality of Service policies for topic communication. Ensure compatibility between publishers and subscribers.

---

## Expected Workflow

1. Start Ollama server (Terminal 1)
2. Launch Gazebo simulation with robot (Terminal 2)
3. Launch LLM planning components (Terminal 3)
4. Give a voice or text command like "Go to the window" (Terminal 4)
5. Watch as the LLM generates waypoints based on the environment
6. The robot navigates autonomously avoiding obstacles
7. Monitor progress in RViz2 (Terminal 5)
8. Check metrics in the CSV file after completion

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

---

## Additional Resources

- ROS 2 Documentation: https://docs.ros.org/en/lyrical/
- TurtleBot3 ROS 2: https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/
- Ollama Documentation: https://ollama.com/
- Original Article: https://www.sciencedirect.com/science/article/abs/pii/S0957417425012229
