cmake_minimum_required(VERSION 3.8)
project(m1_scope_costmap_layer)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/scope_layer.cpp src/master_snapshot_layer.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros)

pluginlib_export_plugin_description_file(nav2_costmap_2d scope_layer.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES scope_layer.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scope_layer test/test_scope_layer.cpp)
  if(TARGET test_scope_layer)
    target_include_directories(test_scope_layer PRIVATE include)
    target_link_libraries(test_scope_layer ${PROJECT_NAME})
    ament_target_dependencies(test_scope_layer
      nav2_costmap_2d
      nav_msgs
      pluginlib
      rclcpp
      rclcpp_lifecycle
      tf2_ros)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros)

ament_package()
