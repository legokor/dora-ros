cmake_minimum_required(VERSION 3.8)
project(teleop_control)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(launch)

# Create the executable
add_executable(drift_control src/drift.cpp)

# Link against necessary libraries
ament_target_dependencies(drift_control rclcpp geometry_msgs)

# Install
install(TARGETS drift_control DESTINATION lib/${CMAKE_PROJECT_NAME})

install(DIRECTORY launch images DESTINATION share/${CMAKE_PROJECT_NAME}/)

install(PROGRAMS
  scripts/teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_key
)

ament_export_dependencies(rosidl_default_runtime)

# Ament package
ament_package()