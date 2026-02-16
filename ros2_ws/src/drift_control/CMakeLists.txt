cmake_minimum_required(VERSION 3.8)
project(drift_control)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(launch)
include_directories(res)

# Create the executable
add_executable(${CMAKE_PROJECT_NAME} src/drift.cpp)

# Link against necessary libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC rclcpp::rclcpp
                                                   ${geometry_msgs_TARGETS})

# Install
install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION lib/${CMAKE_PROJECT_NAME})

install(DIRECTORY launch res DESTINATION share/${CMAKE_PROJECT_NAME}/)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()