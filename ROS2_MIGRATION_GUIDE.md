# ROS 2 Migration Guide for LLM-Based Mobile Robot Path Planning

## Overview
This project was originally built for **ROS 1 Noetic** on Ubuntu 20.04. To run it on **ROS 2 Lyrical** (Ubuntu 26), you need to migrate all components from ROS 1 to ROS 2. This guide provides comprehensive instructions for the migration.

## Key Differences Between ROS 1 and ROS 2

| ROS 1 (Noetic) | ROS 2 (Lyrical) |
|----------------|-----------------|
| `catkin` build system | `colcon` build system |
| `rospy`, `roscpp` | `rclpy`, `rclcpp` |
| `roscore` required | No master node required |
| `roslaunch` | `ros2 launch` |
| `rosrun` | `ros2 run` |
| `tf` package | `tf2_ros` package |
| XML launch files | Python or XML launch files |
| Parameters via `~param_name` | Parameters via `declare_parameter` |
| `/msg` topics | Same, but QoS policies apply |

---

## Required Changes Summary

### 1. Package Structure Changes
- Replace `catkin` with `ament_cmake`/`ament_python`
- Update `package.xml` from format 2 to format 3
- Update `CMakeLists.txt` for ROS 2

### 2. Python Script Changes
All Python scripts using `rospy` must be rewritten to use `rclpy`:

#### Import Changes:
```python
# ROS 1
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import tf

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import tf2_ros
from tf2_ros import TransformListener, TransformBroadcaster
```

#### Node Initialization:
```python
# ROS 1
rospy.init_node('node_name', anonymous=False)

# ROS 2
class NodeName(Node):
    def __init__(self):
        super().__init__('node_name')
```

#### Publishers/Subscribers:
```python
# ROS 1
pub = rospy.Publisher('/topic', MessageType, queue_size=10)
sub = rospy.Subscriber('/topic', MessageType, callback)

# ROS 2
pub = self.create_publisher(MessageType, '/topic', 10)
sub = self.create_subscription(MessageType, '/topic', callback, 10)
```

#### Parameters:
```python
# ROS 1
param = rospy.get_param('~param_name', default_value)

# ROS 2
self.declare_parameter('param_name', default_value)
param = self.get_parameter('param_name').value
```

#### Timers:
```python
# ROS 1
timer = rospy.Timer(rospy.Duration(0.1), callback)

# ROS 2
timer = self.create_timer(0.1, callback)
```

#### Logging:
```python
# ROS 1
rospy.loginfo("message")
rospy.logwarn("message")
rospy.logerr("message")

# ROS 2
self.get_logger().info("message")
self.get_logger().warn("message")
self.get_logger().error("message")
```

### 3. Launch File Changes
Launch files need to be converted to ROS 2 Python launch format or updated XML format.

### 4. TF Changes
- Replace `tf.TransformListener()` with `tf2_ros.Buffer()` and `tf2_ros.TransformListener()`
- Replace `transformPose()` with `do_transform_pose()`
- Use `tf2_geometry_msgs` for transformations

---

## Step-by-Step Migration Instructions

### Prerequisites

1. **Install ROS 2 Lyrical:**
```bash
sudo apt update
sudo apt install ros-lyrical-desktop-full
source /opt/ros/lyrical/setup.bash
```

2. **Install Additional Dependencies:**
```bash
sudo apt install python3-pip
pip3 install ollama
pip3 install speechrecognition
pip3 install noisereduce
pip3 install numpy
pip3 install matplotlib
pip3 install pyyaml
```

3. **Install TurtleBot3 ROS 2 Packages:**
```bash
sudo apt install ros-lyrical-turtlebot3*
```

4. **Setup Ollama:**
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.1
ollama pull qwen2.5
ollama pull mathstral
```

### Workspace Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone or copy the migrated packages here
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Files That Need Modification

### 1. package.xml (turtlebot3_gazebo)

**Changes:**
- Change `<buildtool_depend>catkin</buildtool_depend>` to `<buildtool_depend>ament_cmake</buildtool_depend>`
- Update all dependencies to ROS 2 equivalents
- Add `<export><build_type>ament_cmake</build_type></export>`

### 2. CMakeLists.txt (turtlebot3_gazebo)

**Changes:**
- Replace `find_package(catkin ...)` with `find_package(ament_cmake ...)`
- Update all ROS dependencies
- Remove `catkin_package()`
- Use `ament_target_dependencies()` instead of `target_link_libraries()`
- Add `ament_package()` at the end

### 3. All Python Scripts in `scripts/`

Each script needs complete rewrite from `rospy` to `rclpy`:

- `environment_data_provider_01.py` → `environment_data_provider_01.py`
- `environment_data_provider_02.py` → `environment_data_provider_02.py`
- `environment_data_provider_03.py` → `environment_data_provider_03.py`
- `llm_path_planner.py` → `llm_path_planner.py`
- `waypoint_executor.py` → `waypoint_executor.py`
- `object_marker_publisher.py` → `object_marker_publisher.py`
- `metrics_collector.py` → `metrics_collector.py`
- `voice_command.py` → `voice_command.py`
- `test_plan.py` → `test_plan.py`
- `utils/environment_01.py` → `utils/environment_01.py`
- `utils/environment_02.py` → `utils/environment_02.py`
- `utils/environment_03.py` → `utils/environment_03.py`

### 4. Launch Files

Convert from ROS 1 XML to ROS 2 Python launch format:
- `corridor_01.launch` → `corridor_01_launch.py`
- `corridor_02.launch` → `corridor_02_launch.py`
- `corridor_1.launch` → `corridor_1_launch.py`
- `corridor_2.launch` → `corridor_2_launch.py`
- `corridor_world_building.launch` → `corridor_world_building_launch.py`
- `corridor_world_building_04.launch` → `corridor_world_building_04_launch.py`

---

## Running the Project in ROS 2

After migration, the commands change as follows:

### Start Gazebo World:
```bash
# ROS 1
roslaunch turtlebot3_gazebo corridor_01.launch

# ROS 2
ros2 launch turtlebot3_gazebo corridor_01_launch.py
```

### Start LLM Path Planner:
```bash
# ROS 1
roslaunch turtlebot3_gazebo corridor_1.launch

# ROS 2
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

### Run Voice Command:
```bash
# ROS 1
rosrun turtlebot3_gazebo voice_command.py

# ROS 2
ros2 run turtlebot3_gazebo voice_command
```

### Run Test Plan:
```bash
# ROS 1
rosrun turtlebot3_gazebo test_plan.py

# ROS 2
ros2 run turtlebot3_gazebo test_plan
```

### Start Ollama Server:
```bash
ollama serve
```

### Run RViz2:
```bash
rviz2 -d /path/to/turtlebot3_gazebo/rviz/turtlebot3_gazebo_model.rviz
```

---

## Complete Execution Sequence for ROS 2

### Terminal 1: Start Gazebo Simulation
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo corridor_01_launch.py
```

### Terminal 2: Start LLM Components
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo corridor_1_launch.py
```

### Terminal 3: Start Ollama Server
```bash
ollama serve
```

### Terminal 4: Run Voice or Text Command
```bash
# For voice commands
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlebot3_gazebo voice_command

# OR for text commands
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlebot3_gazebo test_plan
```

### Terminal 5: Run RViz2 (Optional)
```bash
rviz2
```

---

## Important Notes

1. **TF2 Usage**: All TF operations must use `tf2_ros` instead of `tf`
2. **QoS Policies**: ROS 2 uses Quality of Service policies for topics
3. **Parameter Names**: ROS 2 parameter naming conventions differ
4. **Node Names**: Must be unique in ROS 2
5. **Threading**: `rclpy` executors handle callbacks differently than `rospy`
6. **Gazebo Version**: Ubuntu 26 may use Gazebo Garden/Ignition instead of Gazebo 11

---

## Troubleshooting

### Common Issues:

1. **Import Errors**: Ensure all Python dependencies are installed
2. **TF Transformation Failures**: Check frame names and use tf2 properly
3. **Parameter Loading**: Verify parameter names and defaults
4. **Topic Communication**: Check QoS compatibility between publishers/subscribers
5. **Build Errors**: Ensure all dependencies are declared in package.xml

### Debug Commands:
```bash
# List running nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Check parameters
ros2 param dump /node_name

# View TF tree
ros2 run tf2_tools view_frames
```

---

## Conclusion

This migration requires significant changes to convert from ROS 1 to ROS 2. The key areas are:
1. Build system (catkin → colcon/ament)
2. Python API (rospy → rclpy)
3. TF library (tf → tf2_ros)
4. Launch files (XML → Python)
5. Topic communication patterns

Following this guide will help you successfully run the LLM-based path planning project on ROS 2 Lyrical.
