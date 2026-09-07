################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 support for ROS 2
set(CMAKE_CXX_STANDARD 17)

################################################################################
# Find packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo REQUIRED)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

################################################################################
# Build C++ executables (if any)
################################################################################
# Note: turtlebot3_drive.cpp needs to be migrated to ROS 2 if it exists
# For now, we skip building C++ nodes

################################################################################
# Install Python scripts
################################################################################
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/environment_data_provider_01.py
  scripts/environment_data_provider_02.py
  scripts/environment_data_provider_03.py
  scripts/llm_path_planner.py
  scripts/waypoint_executor.py
  scripts/object_marker_publisher.py
  scripts/metrics_collector.py
  scripts/voice_command.py
  scripts/test_plan.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install utils package
install(PROGRAMS
  scripts/utils/environment_01.py
  scripts/utils/environment_02.py
  scripts/utils/environment_03.py
  DESTINATION lib/${PROJECT_NAME}/utils
)

################################################################################
# Install launch files, worlds, and other resources
################################################################################
install(DIRECTORY
  launch
  worlds
  object_list
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# Export dependencies
################################################################################
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
  visualization_msgs
  gazebo
)

ament_package()
