cmake_minimum_required(VERSION 3.8)
project(my_moveit_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(my_planner src/my_planner.cpp)
ament_target_dependencies(my_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS my_planner
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
