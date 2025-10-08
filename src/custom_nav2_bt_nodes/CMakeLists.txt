cmake_minimum_required(VERSION 3.8)
project(custom_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED) # the BT library
find_package(pluginlib REQUIRED)


include_directories(include)

add_library(
  nav2_bt_nodes_check_path_alignment
  SHARED
  src/check_path_alignment.cpp
)

target_compile_features(nav2_bt_nodes_check_path_alignment PUBLIC cxx_std_17)
ament_target_dependencies(
  nav2_bt_nodes_check_path_alignment
  rclcpp
  nav2_behavior_tree
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  behaviortree_cpp_v3
  pluginlib
)

install(
  TARGETS nav2_bt_nodes_check_path_alignment
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(nav2_bt_nodes_check_path_alignment)
ament_export_dependencies(rclcpp nav2_behavior_tree nav2_util nav_msgs geometry_msgs tf2_ros tf2 behaviortree_cpp_v3)

# install the XML definition file for the BT factory
install(
  FILES behavior_tree_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# suppress the creation/installation of setup files that cause warnings
ament_package()