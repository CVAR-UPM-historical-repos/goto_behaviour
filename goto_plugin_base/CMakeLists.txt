cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME goto_plugin_base)
project(${PROJECT_NAME} VERSION 0.1.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  Eigen3
  message_filters
  geometry_msgs
  tf2_geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
  include/${PROJECT_NAME}
)

ament_package()
