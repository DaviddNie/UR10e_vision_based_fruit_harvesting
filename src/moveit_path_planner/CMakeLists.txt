cmake_minimum_required(VERSION 3.10)
project(moveit_path_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tm_msgs REQUIRED)

add_executable(moveit_path_planning_server
  src/moveit_path_planning_server.cpp
)

ament_target_dependencies(moveit_path_planning_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  tm_msgs
)

install(TARGETS moveit_path_planning_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()