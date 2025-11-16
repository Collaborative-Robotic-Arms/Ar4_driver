cmake_minimum_required(VERSION 3.10)
project(point_control_pkg)

# ==========================================================
# Dependencies
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==========================================================
# Include directories
# ==========================================================
# Default include directory for this package
include_directories(
  include
)

# Fix for ROS Jazzy control_msgs being nested incorrectly
if(EXISTS "/opt/ros/jazzy/include/control_msgs/control_msgs/msg/joint_jog.hpp")
  message(WARNING "control_msgs include path appears nested, adding manual include directory.")
  include_directories(/opt/ros/jazzy/include/control_msgs/control_msgs)
endif()

# ==========================================================
# Executables
# ==========================================================
add_executable(point_control src/point_control.cpp)
# ==========================================================
# Link dependencies
# ==========================================================
ament_target_dependencies(point_control
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  control_msgs
  std_msgs
  geometry_msgs
)

# ==========================================================
# Install targets
# ==========================================================
install(TARGETS
  point_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/jog_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
