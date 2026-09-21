cmake_minimum_required(VERSION 3.8)
project(ffw_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(is_path_length_under_plugin SHARED
  src/is_path_length_under.cpp
)

target_include_directories(is_path_length_under_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(is_path_length_under_plugin
  behaviortree_cpp
  nav_msgs
  geometry_msgs
  rclcpp
)

add_library(sim_time_navigators SHARED
  src/sim_time_navigators.cpp
)

target_compile_features(sim_time_navigators PUBLIC cxx_std_17)

ament_target_dependencies(sim_time_navigators
  builtin_interfaces
  nav2_bt_navigator
  nav2_core
  nav2_msgs
  pluginlib
)

add_executable(initial_pose_stamp_adapter
  src/initial_pose_stamp_adapter.cpp
)

target_compile_features(initial_pose_stamp_adapter PUBLIC cxx_std_17)

ament_target_dependencies(initial_pose_stamp_adapter
  builtin_interfaces
  geometry_msgs
  rclcpp
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

install(TARGETS is_path_length_under_plugin
  EXPORT export_is_path_length_under_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  sim_time_navigators
  initial_pose_stamp_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

target_compile_definitions(is_path_length_under_plugin PRIVATE BT_PLUGIN_EXPORT)

ament_export_libraries(is_path_length_under_plugin)
ament_export_dependencies(
  behaviortree_cpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  nav2_bt_navigator
  nav2_core
  nav2_msgs
  pluginlib
  rclcpp
)

ament_package()
