cmake_minimum_required(VERSION 3.16)
project(omslam_navigation_contracts)
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp nav2_util nav2_core nav2_controller nav2_costmap_2d nav2_smac_planner dwb_critics dwb_core dwb_plugins dwb_msgs nav_2d_utils omslam_operator_msgs tf2_ros pluginlib)
  find_package(${dep} REQUIRED)
endforeach()
add_library(speed_aware_navigation SHARED src/speed_aware_navigation.cpp)
target_compile_features(speed_aware_navigation PUBLIC cxx_std_17)
ament_target_dependencies(speed_aware_navigation rclcpp nav2_util nav2_core nav2_controller
  dwb_core dwb_plugins nav_2d_utils omslam_operator_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core navigation_plugins.xml)
pluginlib_export_plugin_description_file(dwb_core navigation_plugins.xml)
install(TARGETS speed_aware_navigation LIBRARY DESTINATION lib)
add_executable(plugin_boundary_probe src/plugin_boundary_probe.cpp)
target_compile_features(plugin_boundary_probe PUBLIC cxx_std_17)
ament_target_dependencies(plugin_boundary_probe rclcpp nav2_util nav2_core nav2_costmap_2d dwb_critics dwb_core dwb_msgs tf2_ros pluginlib)
# Direct checker calls and the plugin loader must use the same lattice library.
# Linking the package's first (Hybrid) library as well duplicates its static
# search tables when the Lattice plugin is loaded and destroys them twice.
find_library(OMSLAM_LATTICE_LIBRARY nav2_smac_planner_lattice
  PATHS "${nav2_smac_planner_DIR}/../../../lib" NO_DEFAULT_PATH REQUIRED)
target_include_directories(plugin_boundary_probe PRIVATE ${nav2_smac_planner_INCLUDE_DIRS})
target_link_libraries(plugin_boundary_probe ${OMSLAM_LATTICE_LIBRARY})
install(TARGETS plugin_boundary_probe DESTINATION lib/${PROJECT_NAME})
add_executable(operator_motion_probe src/operator_motion_probe.cpp)
target_compile_features(operator_motion_probe PUBLIC cxx_std_17)
ament_target_dependencies(operator_motion_probe rclcpp nav2_util nav2_core nav2_costmap_2d
  dwb_core omslam_operator_msgs tf2_ros pluginlib)
install(TARGETS operator_motion_probe DESTINATION lib/${PROJECT_NAME})
ament_package()
