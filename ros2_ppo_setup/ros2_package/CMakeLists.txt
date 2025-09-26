cmake_minimum_required(VERSION 3.5)
project(ppo_argos_ros2)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find ARGoS
pkg_check_modules(ARGOS REQUIRED argos3_simulator)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)

# Include directories
include_directories(
  include/ppo_argos_ros2
  ${ARGOS_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Link directories
link_directories(${ARGOS_LIBRARY_DIRS})

# Create the controller library
add_library(ros2_iant_controller SHARED
  src/ros2_iant_controller.cpp
)

# Set target properties
target_compile_features(ros2_iant_controller PUBLIC cxx_std_17)

# Link libraries
target_link_libraries(ros2_iant_controller
  ${ARGOS_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Install the library
install(TARGETS ros2_iant_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  ppo_ros2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(FILES
  ros2_iant_controller.h
  DESTINATION include/${PROJECT_NAME}
)

ament_package()