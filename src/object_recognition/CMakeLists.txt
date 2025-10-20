cmake_minimum_required(VERSION 3.5)
project(object_recognition)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  object_recognition/yolo_rgb_node.py
  object_recognition/depth_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
