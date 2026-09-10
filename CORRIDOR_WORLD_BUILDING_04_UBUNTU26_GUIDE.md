# Step-by-Step Guide: Running corridor_world_building_04 on Ubuntu 26.04

This guide provides complete instructions to run the **corridor_world_building_04** (department environment) simulation with LLM-based path planning on Ubuntu 26.04 with ROS 2 Jazzy/Rolling.

---

## 📋 Prerequisites

Before starting, ensure you have:
- **Ubuntu 26.04** installed
- **ROS 2 Jazzy or Rolling** installed
- **Python 3.12+** with pip
- **Ollama** installed with required LLM models (qwen2.5, llama3.1, or mathstral)
- **Google Speech Recognition API** (for voice commands, optional)

---

## 🔧 Step 1: Install System Dependencies

### 1.1 Install Python packages
```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-dev
pip3 install --break-system-packages \
    ollama \
    pyyaml \
    matplotlib \
    speechrecognition \
    tf2_ros \
    tf2_geometry_msgs
```

### 1.2 Install Ollama and pull LLM models
```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.com/install.sh | sh

# Pull required LLM models
ollama pull qwen2.5
ollama pull llama3.1
ollama pull mathstral
```

---

## 📁 Step 2: Set Up ROS 2 Workspace

### 2.1 Create workspace and copy files
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy project files from /workspace
cp -r /workspace/turtlebot3_simulations .
cp -r /workspace/turtlebot3_description .

# Go back to workspace root
cd ~/ros2_ws
```

### 2.2 Verify file structure
Your workspace should have:
```
~/ros2_ws/
├── src/
│   ├── turtlebot3_simulations/
│   │   └── turtlebot3_gazebo/
│   │       ├── launch/
│   │       │   └── ros2/
│   │       │       ├── corridor_01_launch.py
│   │       │       ├── corridor_1_launch.py
│   │       │       ├── corridor_world_building_04_launch.py
│   │       │       └── corridor_world_building_launch.py  ← NEW
│   │       ├── scripts/
│   │       │   ├── utils/
│   │       │   │   ├── environment_01_ros2.py
│   │       │   │   └── environment_03_ros2.py  ← NEW
│   │       │   ├── environment_data_provider_03_ros2.py  ← NEW
│   │       │   └── ... (other ROS 2 scripts)
│   │       ├── worlds/
│   │       │   └── department.world
│   │       └── object_list/
│   │           └── department.yaml
│   └── turtlebot3_description/
└── install/
└── build/
```

---

## 🔨 Step 3: Build the Workspace

```bash
cd ~/ros2_ws

# Clean previous builds (optional but recommended)
rm -rf build/ install/ log/

# Build with colcon
colcon build --symlink-install --packages-select turtlebot3_gazebo turtlebot3_description

# Source the workspace
source install/setup.bash
```

**If you encounter PEP 668 errors**, use:
```bash
pip3 install --break-system-packages <package_name>
```

Or create a virtual environment:
```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install ollama pyyaml matplotlib speechrecognition
```

---

## 🚀 Step 4: Run the Simulation (5 Terminals Required)

Open **5 separate terminal windows** and execute the following commands in each:

### Terminal 1 - Start Ollama Server
```bash
ollama serve
```
**Wait until you see:** `Listening on 127.0.0.1:11434`

---

### Terminal 2 - Launch Gazebo Simulation (Department World)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py
```
**This launches:**
- Gazebo with `department.world`
- TurtleBot3 Burger at position (0.0, -3.0, 0.0)
- Map server with `map_03.yaml`
- Static transform publisher (map → odom)

**Wait until you see:** Robot spawned in Gazebo and all nodes initialized.

---

### Terminal 3 - Launch LLM Components
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo corridor_world_building_launch.py
```
**This launches:**
- Environment Data Provider (loads `department.yaml`)
- LLM Path Planner (uses `environment_03_ros2.py`)
- Object Marker Publisher (visualizes rooms, walls, etc.)
- Waypoint Executor (executes planned paths)
- Metrics Collector (logs performance data)

**Wait until you see:** All 5 nodes show "initialized" messages.

---

### Terminal 4 - Send Commands (Choose ONE option)

#### Option A: Text Commands
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlebot3_gazebo test_plan_ros2
```

#### Option B: Voice Commands (requires microphone)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlebot3_gazebo voice_command_ros2
```

**Example commands to try:**
- "Go to room number 101"
- "Navigate to room number 205"
- "Go to the window"
- "Go to main entrance"
- "Navigate to stairs"
- "Room number 110"

---

### Terminal 5 - Visualization (Optional)
```bash
rviz2
```

**In RViz2, add these displays:**
1. **Map** → Topic: `/map`
2. **MarkerArray** → Topic: `/object_markers`
3. **TF** → Shows coordinate frames
4. **RobotModel** → Visualizes TurtleBot3
5. **Path** or **PoseArray** → Topic: `/planned_path` or `/waypoints`

---

## ✅ Step 5: Verify Everything is Working

### Check ROS 2 Nodes
```bash
ros2 node list
```
**Expected output:**
```
/environment_data_provider
/llm_path_planner
/object_marker_publisher
/waypoint_executor
/metrics_collector
/robot_state_publisher_corridor
/spawn_urdf
/map_server
/lifecycle_manager_map
```

### Check Topics
```bash
ros2 topic list
```
**Key topics to verify:**
- `/object_list` - Environment data
- `/planned_waypoints` - LLM-generated waypoints
- `/cmd_vel` - Robot velocity commands
- `/scan` - Laser scan data
- `/map` - Map data

### Check LLM Communication
```bash
ros2 topic echo /planned_waypoints
```
You should see waypoint arrays being published when you send commands.

---

## 🛠️ Step 6: Troubleshooting

### Issue: "Package 'turtlebot3_gazebo' not found"
**Solution:**
```bash
cd ~/ros2_ws
source install/setup.bash
```

### Issue: "Failed to load environment module 'environment_03_ros2'"
**Solution:** Ensure the file exists:
```bash
ls -la ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/utils/environment_03_ros2.py
```

### Issue: Ollama connection error
**Solution:**
1. Check if Ollama is running: `ps aux | grep ollama`
2. Restart Ollama: `ollama serve`
3. Test model: `ollama run qwen2.5 "Hello"`

### Issue: Robot doesn't move
**Solution:**
1. Check if waypoint executor is running: `ros2 node list | grep waypoint`
2. Verify laser scan data: `ros2 topic echo /scan --once`
3. Check for obstacles in the path

### Issue: Gazebo crashes or shows black screen
**Solution:**
```bash
# Update graphics drivers
sudo apt update
sudo apt upgrade

# Try running with software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py
```

### Issue: Python import errors
**Solution:**
```bash
# Reinstall Python dependencies
pip3 install --break-system-packages --upgrade \
    ollama \
    pyyaml \
    matplotlib \
    speechrecognition
```

---

## 📊 Step 7: Monitor Performance

### View Metrics
Metrics are saved to CSV files in:
```bash
ls -la ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/metrics/
```

### View Logs
```bash
# View latest logs
cd ~/ros2_ws/log
ls -lt | head -5

# View specific node logs
cd ~/ros2_ws/log
cat <latest_log_directory>/llm_path_planner*.log
```

---

## 🎯 Quick Reference Commands

### Stop all nodes
```bash
# In each terminal, press Ctrl+C
```

### Restart simulation
```bash
# Kill all ROS 2 nodes
killall -9 gzserver gzclient rviz2

# Clear workspace
cd ~/ros2_ws
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install
source install/setup.bash
```

### Change LLM Model
Edit `~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/utils/environment_03_ros2.py`:
```python
# Change this line:
model = "qwen2.5"  # Options: qwen2.5, llama3.1, mathstral
```

### Adjust Robot Speed
Edit `corridor_world_building_launch.py` parameters:
```python
'linear_speed': 0.2,    # Increase/decrease linear velocity
'angular_speed': 0.5,   # Increase/decrease angular velocity
```

---

## 📝 Summary of Files Created/Modified

The following files were created specifically for `corridor_world_building_04`:

1. **`environment_03_ros2.py`** - Environment utilities for department world
2. **`environment_data_provider_03_ros2.py`** - ROS 2 environment data provider
3. **`corridor_world_building_launch.py`** - ROS 2 launch file for LLM components
4. **`corridor_world_building_04_launch.py`** - ROS 2 launch file for Gazebo simulation

All files are located in:
```
~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/
```

---

## 🎉 Success!

You have successfully set up and run the **corridor_world_building_04** simulation with LLM-based path planning on Ubuntu 26.04!

**Next steps:**
- Try different destination commands
- Experiment with different LLM models
- Modify environment parameters in `department.yaml`
- Analyze performance metrics in the `metrics/` folder

For more information, see:
- [README.md](../README.md)
- [RUNNING_INSTRUCTIONS_ROS2.md](../RUNNING_INSTRUCTIONS_ROS2.md)
- [ROS2_MIGRATION_GUIDE.md](../ROS2_MIGRATION_GUIDE.md)
