cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
include_directories(launch)
include_directories(res)

# Create the executable
add_executable(${CMAKE_PROJECT_NAME} src/controller_node.cpp src/main.cpp
                                     src/uart_handler.cpp)

# Link against necessary libraries
ament_target_dependencies(${CMAKE_PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install
install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION lib/${CMAKE_PROJECT_NAME})

install(DIRECTORY launch res DESTINATION share/${CMAKE_PROJECT_NAME}/)

ament_export_dependencies(rosidl_default_runtime)

# Ament package
ament_package()
