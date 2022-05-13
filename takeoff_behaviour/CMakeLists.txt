cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME takeoff_behaviour VERSION 0.1.0)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  takeoff_plugin_base
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_executable(takeoff_behaviour_node src/takeoff_behaviour_node.cpp)
target_include_directories(takeoff_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_behaviour_node ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS takeoff_behaviour_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
