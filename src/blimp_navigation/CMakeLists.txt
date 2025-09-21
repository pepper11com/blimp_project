cmake_minimum_required(VERSION 3.5)
project(blimp_navigation)

# Use modern find_package behavior for *_ROOT variables when available
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(std_msgs REQUIRED)
find_package(type_description_interfaces REQUIRED)
# ASIO is header-only, no need for find_package



if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(blimp_navigation_lib
  src/pid.cpp
  src/blimp_controller.cpp
  src/path_planner.cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})


link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckCollision.srv"
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs
)

# C++ node for OctoMap collision checking
add_executable(octomap_checker_node src/octomap_checker_node.cpp)
add_executable(blimp_navigator_node src/blimp_navigator_node.cpp)


ament_target_dependencies(octomap_checker_node rclcpp octomap_msgs)
target_include_directories(octomap_checker_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_checker_node ${OCTOMAP_LIBRARIES})
target_link_libraries(octomap_checker_node blimp_navigation_lib)
# Ensure generated interfaces are available to this target
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(octomap_checker_node "${cpp_typesupport_target}")

# Add dependencies for blimp_navigator_node
ament_target_dependencies(blimp_navigator_node rclcpp geometry_msgs tf2_ros tf2_geometry_msgs std_msgs)
target_link_libraries(blimp_navigator_node blimp_navigation_lib)
target_link_libraries(blimp_navigator_node "${cpp_typesupport_target}")

# Add dependencies for the library
ament_target_dependencies(blimp_navigation_lib geometry_msgs)
target_include_directories(blimp_navigation_lib PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(blimp_navigation_lib ${OCTOMAP_LIBRARIES})

# ament_export_libraries(lidar_localization_component)

install(TARGETS
  octomap_checker_node
  blimp_navigator_node
  blimp_navigation_lib
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  blimp_navigation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
