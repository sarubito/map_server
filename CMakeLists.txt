cmake_minimum_required(VERSION 3.8)
project(map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(map_server_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME} src/map_server.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  nav_msgs
  std_msgs
  yaml-cpp
  OpenCV
  map_server_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
