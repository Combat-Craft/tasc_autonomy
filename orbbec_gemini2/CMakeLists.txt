cmake_minimum_required(VERSION 3.8)
project(orbbec_gemini2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find C++ and Python dependencies, like in package.xml 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# find ROS2 dependencies 
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find Orbbec depencies
set(OrbbecSDK_DIR "/opt/OrbbecSDK_v2.8.6/lib")
find_package(OrbbecSDK REQUIRED)


# ====================== C++ SECTION ======================
# Include C++ "include" directory
include_directories(include)

# Include orbbec libraries
include_directories("${OrbbecSDK_DIR}/include")

# Create C++ executables
add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp std_msgs)

# Create Orbbec C++ executables 
add_executable(color_example src/color_example.cpp)
target_link_libraries(color_example ob::OrbbecSDK)

# Install C++ executables
install(TARGETS
  main
  color_example
  DESTINATION lib/${PROJECT_NAME}
)


# ====================== PYTHON SECTION ======================
# Install Python modules from orbbec_gemini2/ folder
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)

## NOTE : installs will place both C++ and Python executables inside lib/ folder

ament_package()
