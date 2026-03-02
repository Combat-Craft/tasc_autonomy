cmake_minimum_required(VERSION 3.8)
project(morse_gazebo)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

# --- Include directories ---
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# --- Build plugin library ---
add_library(morse_light_plugin SHARED src/morse_light_plugin.cpp)

# Link ROS and Gazebo
ament_target_dependencies(morse_light_plugin
  rclcpp
  std_msgs
)

target_link_libraries(morse_light_plugin
  ${GAZEBO_LIBRARIES}
)

# --- Install plugin ---
install(TARGETS morse_light_plugin
  LIBRARY DESTINATION lib
)

# --- Export for ament ---
ament_package()
