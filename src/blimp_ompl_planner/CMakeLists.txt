cmake_minimum_required(VERSION 3.8)
project(blimp_ompl_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find OMPL
find_package(ompl REQUIRED)

# Find OctoMap
find_package(octomap REQUIRED)

# Include directories
include_directories(include)

# Create the ompl_planner_node executable
add_executable(ompl_planner_node 
  src/ompl_planner_node.cpp
  src/octomap_validity_checker.cpp
)

# Dependencies for the executable
ament_target_dependencies(ompl_planner_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link OMPL and OctoMap libraries
target_link_libraries(ompl_planner_node 
  ompl
  ${OCTOMAP_LIBRARIES}
)

# Include OMPL and OctoMap directories  
target_include_directories(ompl_planner_node PRIVATE
  /usr/include/ompl-1.5
  ${OCTOMAP_INCLUDE_DIRS}
)

# Install the executable
install(TARGETS
  ompl_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
