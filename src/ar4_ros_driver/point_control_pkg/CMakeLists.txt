cmake_minimum_required(VERSION 3.10)
project(point_control_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# If curl is needed (because of resource_retriever in MoveIt)

# Add executable
add_executable(point_control src/point_control.cpp)

# Link dependencies
ament_target_dependencies(point_control
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)


# Install target
install(TARGETS point_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
