cmake_minimum_required(VERSION 3.8)
project(stereo_depth_yolo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Required Build Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Build the C++ Shared Library Component
add_library(stereo_header_sync_node SHARED
  src/stereo_header_sync_node.cpp
)

target_include_directories(stereo_header_sync_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stereo_header_sync_node
  rclcpp
  rclcpp_components
  message_filters
  sensor_msgs
)

# 3. Register Component with ROS 2 ClassLoader (Enables Intra-Process Zero-Copy)
rclcpp_components_register_node(stereo_header_sync_node
  PLUGIN "stereo_depth_yolo::StereoHeaderSyncNode"
  EXECUTABLE stereo_header_sync_node_exe
)

# 4. Installation Directives
install(TARGETS stereo_header_sync_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python Node Executables
install(PROGRAMS
  stereo_depth_yolo/spatial_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()