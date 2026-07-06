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

# find gstreamer dependencies
find_package(PkgConfig REQUIRED)
if(PkgConfig_FOUND)
  pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
  pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
  pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
  pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)
endif()
#
find_package(OpenCV REQUIRED)

# ====================== C++ SECTION ======================
# Include C++ "include" directory
include_directories(include)

# Include orbbec libraries
include_directories("${OrbbecSDK_DIR}/include")

# Create C++ executables

#add_executable(main src/main.cpp)
#ament_target_dependencies(main rclcpp std_msgs)

# Create Orbbec C++ executables

#add_executable(gemini2_color_gstreamer src/color_gstreamer.cpp)
#target_link_libraries(gemini2_color_gstreamer
#    ob::OrbbecSDK
#    PkgConfig::gstreamer
#    PkgConfig::gstreamer-sdp
#    PkgConfig::gstreamer-app
#    PkgConfig::gstreamer-video
#)

#add_executable(test_gstreamer src/test_gstreamer.cpp)
#target_link_libraries(test_gstreamer 
#    ob::OrbbecSDK
#    PkgConfig::gstreamer
##    PkgConfig::gstreamer-sdp
 #   PkgConfig::gstreamer-app
#    PkgConfig::gstreamer-video
#)

#add_executable(test_gstreamer_v2 src/test_gstreamer_v2.cpp)
#target_link_libraries(test_gstreamer_v2
#    ob::OrbbecSDK
#    PkgConfig::gstreamer
#    PkgConfig::gstreamer-sdp
#    PkgConfig::gstreamer-app
#    PkgConfig::gstreamer-video
#)
add_executable(orbbec_gstreamer src/test_gstreamer3.cpp)
target_link_libraries(orbbec_gstreamer
    ob::OrbbecSDK
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

add_executable(streamer src/gemini2_streamer.cpp)
target_link_libraries(streamer
    ob::OrbbecSDK
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

# orbbec gstreamer


#add_executable(my-gstreamer-app main.cpp)

#target_link_libraries(my-gstreamer-app
#    PkgConfig::gstreamer
 #   PkgConfig::gstreamer-sdp
 #   PkgConfig::gstreamer-app
 #   PkgConfig::gstreamer-video
#)

# Install C++ executables
install(TARGETS
  #main
  orbbec_gstreamer
  #test_gstreamer_v2
  streamer
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
