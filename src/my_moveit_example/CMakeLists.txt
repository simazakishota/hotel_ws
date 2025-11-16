cmake_minimum_required(VERSION 3.8)
project(my_moveit_example)

# --- 依存パッケージ ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# --- C++バージョン指定 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 実行ファイル追加 ---
add_executable(my_planner_node src/my_planner_node.cpp)

ament_target_dependencies(my_planner_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
)

# --- インストール ---
install(TARGETS my_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
