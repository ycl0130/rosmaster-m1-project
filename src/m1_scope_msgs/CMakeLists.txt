cmake_minimum_required(VERSION 3.8)
project(m1_scope_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScopePredictionSlice.msg"
  "msg/ScopePredictionSequence.msg"
  DEPENDENCIES builtin_interfaces nav_msgs std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
