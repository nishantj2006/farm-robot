cmake_minimum_required(VERSION 3.8)
project(stereo_depth_yolo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Required Build Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================================================
# 2. Build the C++ Shared Library Components
# =========================================================

# Component A: Stereo Header Sync Node
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

# Component B: Stereo Sync Spoofer (Merged)
add_library(stereo_sync_spoofer_component SHARED 
  src/stereo_sync_spoofer.cpp
)
target_include_directories(stereo_sync_spoofer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stereo_sync_spoofer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
)

# =========================================================
# 3. Register Components with ROS 2 ClassLoader
# =========================================================

rclcpp_components_register_node(stereo_header_sync_node
  PLUGIN "stereo_depth_yolo::StereoHeaderSyncNode"
  EXECUTABLE stereo_header_sync_node_exe
)

rclcpp_components_register_nodes(stereo_sync_spoofer_component 
  "stereo_sync_spoofer::StereoSyncSpoofer"
)

# =========================================================
# 4. Installation Directives
# =========================================================

install(TARGETS 
  stereo_header_sync_node
  stereo_sync_spoofer_component
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
