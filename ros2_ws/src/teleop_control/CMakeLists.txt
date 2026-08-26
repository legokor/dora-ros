cmake_minimum_required(VERSION 3.8)
project(teleop_control)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For generating messages

include_directories(include)
include_directories(launch)

# Generating messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyInputMsg.msg"
  "msg/MouseInputMsg.msg"
  )

# Create the executable
add_executable(key_control src/keyMovement.cpp)

# Link against necessary libraries
ament_target_dependencies(key_control rclcpp geometry_msgs)

# Install
install(TARGETS key_control DESTINATION lib/${CMAKE_PROJECT_NAME})

install(DIRECTORY launch images DESTINATION share/${CMAKE_PROJECT_NAME}/)

install(PROGRAMS
  scripts/teleopKey.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleopKey
)

ament_export_dependencies(rosidl_default_runtime)

# Ament package
ament_package()
