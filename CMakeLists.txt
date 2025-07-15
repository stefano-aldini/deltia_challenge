cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(deltia_challenge)

#============================================================================
#| COMPILE OPTIONS
#============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#============================================================================
#| FIND DEPENDENCIES
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PCL REQUIRED)

#============================================================================
#| BUILD TARGETS
#============================================================================

# Sorting Node (Perception)
add_executable(sorting_node src/sorting_node.cpp)
ament_target_dependencies(sorting_node
  rclcpp geometry_msgs depthai_ros_msgs tf2_ros tf2_geometry_msgs
  sensor_msgs pcl_conversions tf2_eigen
)
target_include_directories(sorting_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sorting_node ${PCL_LIBRARIES})

# Action Node (Control)
add_executable(action_node src/action_node.cpp)
ament_target_dependencies(action_node
  rclcpp rclcpp_action moveit_ros_planning_interface franka_msgs
  tf2_ros geometry_msgs std_msgs sensor_msgs
)
target_include_directories(action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#============================================================================
#| INSTALL RULES
#============================================================================

# Install executables
install(TARGETS
  sorting_node
  action_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#============================================================================
#| FINAL PACKAGE CALL
#============================================================================
ament_package()