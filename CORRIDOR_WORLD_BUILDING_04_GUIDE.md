# Step-by-Step Guide: Running corridor_world_building_04 on Ubuntu 26.04

This guide walks you through running the `corridor_world_building_04` simulation with ROS 2 on Ubuntu 26.04.

## Prerequisites

- **Ubuntu 26.04** (Noble Numbat or later)
- **ROS 2 Jazzy Jalisco** or **ROS 2 Rolling Ridley**
- **Python 3.12+**
- **Gazebo Classic 11+** or **Gazebo Sim**
- **Ollama** with LLM models (optional, for LLM path planning)

---

## Step 1: Install System Dependencies

### 1.1 Install ROS 2 Jazzy (if not already installed)

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop-full

# Source ROS 2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.2 Install Additional ROS 2 Packages

```bash
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager
```

### 1.3 Install Python Dependencies

```bash
# For Ubuntu 26.04, use --break-system-packages or create a virtual environment
pip3 install --break-system-packages \
    numpy \
    pandas \
    pyyaml \
    requests \
    openai \
    speechrecognition \
    pygame
```

**Alternative (Recommended): Use Virtual Environment**

```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install numpy pandas pyyaml requests openai speechrecognition pygame
```

### 1.4 Install Ollama (Optional - for LLM Path Planning)

```bash
curl -fsSL https://ollama.com/install.sh | sh

# Pull a model (e.g., llama2, mistral, or phi)
ollama pull llama2
ollama pull mistral

# Start Ollama server
ollama serve
```

---

## Step 2: Set Up Your ROS 2 Workspace

### 2.1 Create and Prepare Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the project files from /workspace
cp -r /workspace/turtlebot3_simulations .
cp -r /workspace/turtlebot3_description .

# Return to workspace root
cd ~/ros2_ws
```

### 2.2 Verify ROS 2 Launch File Exists

Ensure the ROS 2 launch file is in place:

```bash
ls -la src/turtlebot3_simulations/turtlebot3_gazebo/launch/ros2/corridor_world_building_04_launch.py
```

You should see the file listed.

---

## Step 3: Build the Workspace

```bash
cd ~/ros2_ws

# Clean previous builds (optional but recommended)
rm -rf build/ install/ log/

# Build with colcon
colcon build --symlink-install --packages-select turtlebot3_gazebo

# Source the workspace
source install/setup.bash
```

**If you encounter PEP 668 errors**, use:

```bash
colcon build --symlink-install --packages-select turtlebot3_gazebo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Step 4: Run the corridor_world_building_04 Simulation

You'll need **multiple terminal windows** to run all components.

### Terminal 1: Start Ollama Server (Optional - if using LLM)

```bash
ollama serve
```

### Terminal 2: Launch Gazebo with corridor_world_building_04

```bash
cd ~/ros2_ws
source install/setup.bash

# Launch the simulation
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py
```

**Expected behavior:**
- Gazebo window opens with the `department.world` environment
- TurtleBot3 Burger spawns at position (0.0, -3.0, 0.0)
- Map server loads `map_03.yaml`
- TF tree includes `map -> odom` transform

### Terminal 3: Run RViz2 for Visualization (Optional)

```bash
cd ~/ros2_ws
source install/setup.bash

rviz2
```

In RViz2:
1. Add **RobotModel** display
2. Add **Map** display (subscribe to `/map`)
3. Add **LaserScan** display (subscribe to `/scan`)
4. Set Fixed Frame to `map`

### Terminal 4: Run the Test Plan (LLM Path Planner)

```bash
cd ~/ros2_ws
source install/setup.bash

# Run the test plan script
ros2 run turtlebot3_gazebo test_plan_ros2
```

This will:
- Query the LLM for path planning commands
- Execute waypoint navigation
- Collect metrics

### Terminal 5: Monitor Topics (Optional)

```bash
# List all active topics
ros2 topic list

# Monitor robot pose
ros2 topic echo /odom

# Monitor laser scan
ros2 topic echo /scan --once

# Check map server status
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: ''}"
```

---

## Step 5: Verify Everything is Working

### 5.1 Check Running Nodes

```bash
ros2 node list
```

**Expected nodes:**
- `/robot_state_publisher_corridor`
- `/spawn_urdf`
- `/map_server`
- `/lifecycle_manager_map`
- `/gazebo`
- `/gazebo_gui`

### 5.2 Check TF Tree

```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

Verify that `map -> odom -> base_footprint` transforms exist.

### 5.3 Check Map Server

```bash
ros2 topic echo /map_metadata
```

Should return map resolution, width, height, and origin.

### 5.4 Test Robot Movement

```bash
# Send a simple velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

The robot should move forward slightly in Gazebo.

---

## Step 6: Troubleshooting Common Issues

### Issue 1: "Package 'turtlebot3_gazebo' not found"

**Solution:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 pkg prefix turtlebot3_gazebo
```

If the command fails, rebuild:
```bash
colcon build --symlink-install
```

### Issue 2: Map Server Fails to Load

**Error:** `Failed to load map from file`

**Solution:** The map YAML path uses a relative path. Ensure the map files are in the correct location:

```bash
# Check map files exist
ls -la ~/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/maps/
ls -la ~/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/object_list/
```

If missing, reinstall:
```bash
colcon build --symlink-install
```

### Issue 3: Gazebo Doesn't Start or Crashes

**Solution:**
```bash
# Check Gazebo version
gzserver --version

# Kill any stuck Gazebo processes
killall gzserver gzclient

# Try launching with verbose output
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py --debug
```

### Issue 4: TF Tree Missing Transforms

**Error:** `Frame [map] does not exist`

**Solution:** Verify the static transform publisher is running:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Issue 5: LLM Connection Fails

**Error:** `Connection refused to Ollama server`

**Solution:**
```bash
# Check if Ollama is running
ps aux | grep ollama

# Restart Ollama
ollama serve

# Test connection
curl http://localhost:11434/api/generate -d '{"model":"llama2","prompt":"hello"}'
```

### Issue 6: Python Import Errors (PEP 668)

**Error:** `externally-managed-environment`

**Solution A:** Use `--break-system-packages`:
```bash
pip3 install --break-system-packages <package_name>
```

**Solution B (Recommended):** Use virtual environment:
```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install <package_name>
```

Then source both environments:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_venv/bin/activate
source ~/ros2_ws/install/setup.bash
```

### Issue 7: URDF/Xacro Processing Fails

**Error:** `Failed to process xacro file`

**Solution:** Ensure xacro is installed and the URDF path is correct:
```bash
ros2 run xacro --help

# Test xacro processing
ros2 run xacro $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro
```

---

## Step 7: Quick Reference Commands

### Start Simulation Only
```bash
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py
```

### Start with Specific Robot Model
```bash
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py model:=waffle
```

### Start with Custom Position
```bash
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py x:=1.0 y:=2.0 yaw:=0.0
```

### View All Topics
```bash
ros2 topic list
```

### View Node Graph
```bash
rqt_graph
```

### Record Bag Data
```bash
ros2 bag record -a
```

### Play Bag Data
```bash
ros2 bag play <bag_directory>
```

### Stop All ROS 2 Nodes
```bash
killall -9 python3
```

---

## Step 8: Understanding the Launch File

The `corridor_world_building_04_launch.py` file:

1. **Launches Gazebo** with `department.world`
2. **Spawns TurtleBot3 Burger** at (0.0, -3.0, 0.0) with yaw 1.557343
3. **Publishes static transform** `map -> odom`
4. **Loads map server** with `map_03.yaml`
5. **Activates map server** via lifecycle manager

### Key Components:
- **World**: `department.world` (large indoor environment)
- **Map**: `map_03.pgm` + `map_03.yaml`
- **Robot**: TurtleBot3 Burger
- **TF Frames**: `map`, `odom`, `base_footprint`, `base_link`, etc.

---

## Step 9: Next Steps After Simulation Runs

Once the simulation is running successfully:

1. **Test manual control:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

2. **Run LLM path planner:**
   ```bash
   ros2 run turtlebot3_gazebo llm_path_planner_ros2
   ```

3. **Execute waypoints:**
   ```bash
   ros2 run turtlebot3_gazebo waypoint_executor_ros2
   ```

4. **Collect metrics:**
   ```bash
   ros2 run turtlebot3_gazebo metrics_collector_ros2
   ```

5. **Visualize markers:**
   ```bash
   ros2 run turtlebot3_gazebo object_marker_publisher_ros2
   ```

---

## Summary

You now have a complete setup to run `corridor_world_building_04` on Ubuntu 26.04 with ROS 2! 

**Key Files Modified/Created:**
- ✅ `launch/ros2/corridor_world_building_04_launch.py` (ROS 2 launch file)
- ✅ `object_list/map_03.yaml` (Fixed map path)
- ✅ `CMakeLists.txt` (Added ROS 2 script installation)

**Commands to Remember:**
```bash
# Build
colcon build --symlink-install

# Run
ros2 launch turtlebot3_gazebo corridor_world_building_04_launch.py

# Test
ros2 run turtlebot3_gazebo test_plan_ros2
```

Enjoy your simulation! 🤖🚀
