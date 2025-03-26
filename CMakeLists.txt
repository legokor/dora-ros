cmake_minimum_required(VERSION 3.8)
project(robot_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the executable
add_executable(${CMAKE_PROJECT_NAME}
    src/publisher_subscriber.cpp
    src/uart_handler.cpp
)

# Link against necessary libraries
ament_target_dependencies(${CMAKE_PROJECT_NAME}
    rclcpp
    geometry_msgs
)

# Install
install(TARGETS ${CMAKE_PROJECT_NAME}
  DESTINATION lib/${CMAKE_PROJECT_NAME})

# Ament package
ament_package()
