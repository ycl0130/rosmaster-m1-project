cmake_minimum_required(VERSION 3.8)
project(m1_scope_risk)

find_package(ament_cmake REQUIRED)
find_package(m1_scope_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/risk_field_snapshot.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} m1_scope_msgs nav_msgs)

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_risk_field_snapshot test/test_risk_field_snapshot.cpp)
  if(TARGET test_risk_field_snapshot)
    target_link_libraries(test_risk_field_snapshot ${PROJECT_NAME})
    ament_target_dependencies(test_risk_field_snapshot m1_scope_msgs nav_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(m1_scope_msgs nav_msgs)
ament_package()
