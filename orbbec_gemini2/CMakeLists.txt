cmake_minimum_required(VERSION 3.15.3)
project(orbbec_gemini2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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

# find Orbbec dependencies
set(OrbbecSDK_DIR "/opt/OrbbecSDK_v2.8.6/lib")
find_package(OrbbecSDK REQUIRED)

# find gstreamer C dependencies
find_package(PkgConfig REQUIRED)
if(PkgConfig_FOUND)
  pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
  pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
  pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
  pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)
endif()

# find gstreamer C++ dependencies
#if(PkgConfig_FOUND)  
#  pkg_search_module(GSTMM REQUIRED IMPORTED_TARGET gstreamermm-1.0)
#  pkg_search_module(GLIBMM REQUIRED IMPORTED_TARGET glibmm-2.4) 
#endif()
#

# add custom class files
add_library(MyOrbbecStreamer src/MyOrbbecStreamer.cpp)
target_link_libraries(MyOrbbecStreamer
    ob::OrbbecSDK
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

# ====================== C++ SECTION ======================
# Include C++ "include" directory
include_directories(include)

# Include orbbec libraries
include_directories("${OrbbecSDK_DIR}/include")

# Create C++ executables

add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp std_msgs)

# Create Orbbec C++ executables
add_executable(test_gstreamer3 src/test_gstreamer3.cpp)
target_link_libraries(test_gstreamer3
    ob::OrbbecSDK
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

#add_executable(MyOrbbecStreamer src/MyOrbbecStreamer.cpp)
#target_link_libraries(MyOrbbecStreamer
#    ob::OrbbecSDK
#    PkgConfig::gstreamer
#    PkgConfig::gstreamer-sdp
#    PkgConfig::gstreamer-app
#    PkgConfig::gstreamer-video
#)

add_executable(test_MyOrbbecStreamer src/testMyOrbbecStreamer.cpp)
target_link_libraries(test_MyOrbbecStreamer PRIVATE MyOrbbecStreamer)

# orbbec gstreamer
#add_executable(streamerV2 src/streamerV2.cpp)
#target_link_libraries(streamerV2
#    ob::OrbbecSDK
#    PkgConfig::GLIBMM
#    PkgConfig::GSTMM
#    PkgConfig::gstreamer
#    PkgConfig::gstreamer-sdp
#    PkgConfig::gstreamer-app
#    PkgConfig::gstreamer-video
#)


# Install C++ executables
install(TARGETS
  main
  test_gstreamer3
  MyOrbbecStreamer
  test_MyOrbbecStreamer
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
