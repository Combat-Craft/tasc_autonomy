cmake_minimum_required(VERSION 3.8)
project(autonomy_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# copies your config and launch folders to the executable directory
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()