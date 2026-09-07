#!/bin/bash
# Setup script for running LLM-Based Mobile Robot Path Planning on ROS 2 (Ubuntu 26 / ROS 2 Lyrical)

set -e

echo "=========================================="
echo "ROS 2 Lyrical Setup for LLM Path Planning"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    print_error "Please run as root (use sudo)"
    exit 1
fi

echo ""
echo "Step 1: Installing ROS 2 Lyrical dependencies..."
apt update
apt install -y software-properties-common curl gnupg lsb-release

echo ""
echo "Step 2: Setting up ROS 2 repositories..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ""
echo "Step 3: Installing ROS 2 Lyrical Desktop Full..."
apt update
apt install -y ros-lyrical-desktop-full
print_success "ROS 2 Lyrical installed"

echo ""
echo "Step 4: Installing additional ROS 2 packages..."
apt install -y ros-lyrical-turtlebot3-gazebo \
               ros-lyrical-turtlebot3-description \
               ros-lyrical-nav2-map-server \
               ros-lyrical-slam-toolbox \
               ros-lyrical-rviz2 \
               ros-lyrical-tf2-tools \
               ros-lyrical-tf2-ros \
               ros-lyrical-tf2-geometry-msgs

echo ""
echo "Step 5: Installing Python dependencies..."
apt install -y python3-pip python3-yaml python3-numpy python3-matplotlib
pip3 install ollama speechrecognition noisereduce pyyaml

echo ""
echo "Step 6: Setting up Ollama..."
curl -fsSL https://ollama.com/install.sh | sh
print_success "Ollama installed"

echo ""
echo "Step 7: Pulling required LLM models..."
ollama pull llama3.1
ollama pull qwen2.5
ollama pull mathstral
print_success "LLM models downloaded"

echo ""
echo "Step 8: Setting up workspace..."
mkdir -p /root/ros2_ws/src
cd /root/ros2_ws/src

# Copy the project files if not already there
if [ ! -d "/root/ros2_ws/src/turtlebot3_simulations" ]; then
    cp -r /workspace/turtlebot3_simulations .
    cp -r /workspace/turtlebot3_description .
    print_success "Project files copied to workspace"
else
    print_warning "Project files already exist in workspace"
fi

cd /root/ros2_ws

echo ""
echo "Step 9: Building the workspace..."
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
print_success "Workspace built successfully"

echo ""
echo "Step 10: Creating launch directory structure..."
mkdir -p /root/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch/ros2
cp -r /workspace/turtlebot3_simulations/turtlebot3_gazebo/launch/ros2/* /root/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch/ros2/ 2>/dev/null || true

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To run the simulation, follow these steps:"
echo ""
echo "1. Source the workspace:"
echo "   source /root/ros2_ws/install/setup.bash"
echo ""
echo "2. Start Ollama server (in Terminal 1):"
echo "   ollama serve"
echo ""
echo "3. Launch Gazebo simulation (in Terminal 2):"
echo "   ros2 launch turtlebot3_gazebo corridor_01_launch.py"
echo ""
echo "4. Launch LLM path planning components (in Terminal 3):"
echo "   ros2 launch turtlebot3_gazebo corridor_1_launch.py"
echo ""
echo "5. Run voice or text commands (in Terminal 4):"
echo "   For voice: ros2 run turtlebot3_gazebo voice_command"
echo "   For text:  ros2 run turtlebot3_gazebo test_plan"
echo ""
echo "6. Open RViz2 for visualization (optional, Terminal 5):"
echo "   rviz2"
echo ""
echo "For more details, see ROS2_MIGRATION_GUIDE.md"
echo "=========================================="
