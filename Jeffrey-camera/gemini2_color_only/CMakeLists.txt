cmake_minimum_required(VERSION 3.10)
project(gemini2_color_only)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OrbbecSDK_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/lib")

find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)


# Build a small utils library from Orbbec example utils
add_library(ob_utils STATIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils/utils_opencv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils/utils_c.c
)

target_include_directories(ob_utils PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils
    ${CMAKE_CURRENT_LIST_DIR}/utils
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ob_utils PUBLIC ${OpenCV_LIBS} ob::OrbbecSDK)

add_executable(gemini2_color_only src/color_only.cpp)
target_include_directories(gemini2_color_only PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils
    ${CMAKE_CURRENT_LIST_DIR}/utils
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gemini2_color_only ob::OrbbecSDK ${OpenCV_LIBS} ob_utils)

add_executable(gemini2_color_gstreamer src/color_gstreamer.cpp)
target_include_directories(gemini2_color_gstreamer PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/lib/orbbecsdk/examples/src/utils
    ${CMAKE_CURRENT_LIST_DIR}/utils
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
)

target_link_libraries(gemini2_color_gstreamer
    ob::OrbbecSDK
    ${OpenCV_LIBS}
    ob_utils
    ${GSTREAMER_LIBRARIES}
)

# Propagate required compile flags from pkg-config
target_compile_options(gemini2_color_gstreamer PRIVATE ${GSTREAMER_CFLAGS_OTHER})