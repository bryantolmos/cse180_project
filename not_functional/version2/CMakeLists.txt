cmake_minimum_required(VERSION 3.8)
project(commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(commander src/commander.cpp)

ament_target_dependencies(
commander
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs       # <-- ADD THIS
  nav_msgs          # <-- required for OccupancyGrid
  visualization_msgs  # <-- for RViz markers
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
