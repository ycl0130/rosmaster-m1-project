cmake_minimum_required(VERSION 3.8)
project(m1_fast_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fast2d_planner.cpp
  src/kinodynamic_astar.cpp
  src/straight_line.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  diagnostic_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_core m1_fast_planner_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES m1_fast_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(kinodynamic_smoke test/kinodynamic_smoke.cpp)
  target_link_libraries(kinodynamic_smoke ${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_straight_line test/test_straight_line.cpp)
  if(TARGET test_straight_line)
    target_link_libraries(test_straight_line ${PROJECT_NAME})
    ament_target_dependencies(test_straight_line
      geometry_msgs
      nav2_costmap_2d
      nav_msgs)
  endif()
  ament_add_gtest(test_kinodynamic_astar test/test_kinodynamic_astar.cpp)
  if(TARGET test_kinodynamic_astar)
    target_link_libraries(test_kinodynamic_astar ${PROJECT_NAME})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  diagnostic_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs)

ament_package()
