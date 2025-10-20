cmake_minimum_required(VERSION 3.8)
project(grasp_pose_executor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(grasp_pose_executor src/grasp_pose_executor.cpp)
ament_target_dependencies(grasp_pose_executor
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS grasp_pose_executor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
