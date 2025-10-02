cmake_minimum_required(VERSION 3.5)
project(blimp_navigation)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_library(blimp_navigation_lib
  src/pid.cpp
  src/actuator_interface.cpp
  src/path_follower.cpp
)
target_include_directories(blimp_navigation_lib
  PUBLIC
    include)
ament_target_dependencies(blimp_navigation_lib
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs)

add_executable(blimp_navigator_node
  src/blimp_navigator_node.cpp)
target_include_directories(blimp_navigator_node
  PUBLIC
    include)
ament_target_dependencies(blimp_navigator_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)
target_link_libraries(blimp_navigator_node
  blimp_navigation_lib)

add_executable(actuator_exerciser
  src/actuator_exerciser.cpp)
target_include_directories(actuator_exerciser
  PUBLIC
    include)
ament_target_dependencies(actuator_exerciser
  rclcpp)
target_link_libraries(actuator_exerciser
  blimp_navigation_lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckCollision.srv"
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(blimp_navigator_node "${cpp_typesupport_target}")
target_link_libraries(blimp_navigation_lib "${cpp_typesupport_target}")

install(TARGETS
  blimp_navigation_lib
  blimp_navigator_node
  actuator_exerciser
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_follower test/test_path_follower.cpp)
  target_link_libraries(test_path_follower blimp_navigation_lib)
  ament_target_dependencies(test_path_follower geometry_msgs nav_msgs)

  ament_add_gtest(test_motor_mapping test/test_motor_mapping.cpp)
  if(TARGET test_motor_mapping)
    target_link_libraries(test_motor_mapping blimp_navigation_lib)
  endif()

  ament_add_gtest(test_actuator_sequence test/test_actuator_sequence.cpp)
  if(TARGET test_actuator_sequence)
    target_link_libraries(test_actuator_sequence blimp_navigation_lib)
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
