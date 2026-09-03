cmake_minimum_required(VERSION 3.8)
project(teleop_control)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Convencience var
set(exec_name "keyControl")
set(script_name "teleopKey")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For generating messages

include_directories(include)
include_directories(launch)

# Generating messages
# It creates a new target to compile messages to.
# That's why it needs a different name
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyInputMsg.msg"
  "msg/MouseInputMsg.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# Create the executable
add_executable(${exec_name} src/keyMovement.cpp)

# Support for creating and using interfaces in single packages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link against necessary libraries
target_link_libraries(${exec_name}
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  "${cpp_typesupport_target}"
)

# Install
install(TARGETS ${exec_name} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch images DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  scripts/teleopKey.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ${script_name}
)

# Ament package
ament_package()
