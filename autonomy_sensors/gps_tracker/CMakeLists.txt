cmake_minimum_required(VERSION 3.8)
project(gps_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ===============================
# BUILD NODES
# ===============================

add_executable(fake_gps src/gps_publisher.cpp)
ament_target_dependencies(fake_gps
  rclcpp
  sensor_msgs
)

add_executable(route_logger src/route_logger.cpp)
ament_target_dependencies(route_logger
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  fake_gps
  route_logger
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
