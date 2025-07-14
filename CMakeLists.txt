cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(deltia_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(franka_msgs REQUIRED)

add_executable(sorting_node src/sorting_node.cpp)
ament_target_dependencies(sorting_node
    rclcpp
    geometry_msgs
    depthai_ros_msgs
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    pcl_conversions
    tf2_eigen
)

target_link_libraries(sorting_node
    ${PCL_LIBRARIES}
)

add_executable(action_node src/action_node.cpp)
ament_target_dependencies(action_node
    rclcpp
    rclcpp_action
    moveit_ros_planning_interface
    franka_msgs
    tf2_ros
    geometry_msgs
    std_msgs
)

install(TARGETS
    sorting_node
    action_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS scripts/gui_node.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
