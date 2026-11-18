cmake_minimum_required(VERSION 3.16)
project(omslam_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(omslam_airport_pedestrians SHARED src/airport_pedestrians.cpp)
target_compile_features(omslam_airport_pedestrians PUBLIC cxx_std_17)
target_compile_options(omslam_airport_pedestrians PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(omslam_airport_pedestrians PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::core
  gz-msgs10::gz-msgs10)

install(TARGETS omslam_airport_pedestrians LIBRARY DESTINATION lib)
install(DIRECTORY launch config worlds models maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gazebo_runner.py DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_runner)

install(PROGRAMS scripts/sim_base.py DESTINATION lib/${PROJECT_NAME} RENAME sim_base)
install(PROGRAMS scripts/sensor_fault_relay.py DESTINATION lib/${PROJECT_NAME} RENAME sensor_fault_relay)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_guard test/test_sim_guard.py)
endif()

ament_package()
