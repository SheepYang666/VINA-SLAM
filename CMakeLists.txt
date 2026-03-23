cmake_minimum_required(VERSION 3.8)
project(vina_slam)

cmake_policy(SET CMP0074 NEW)

# ===============================
# CMake Configuration
# ===============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===============================
# Default Build Type
# ===============================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ===============================
# Find Dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

message(STATUS "Eigen version: ${EIGEN_VERSION}")

# ===============================
# Source Files
# ===============================
set(SOURCES
  src/core/point_utils.cpp
  src/estimation/imu_preintegration.cpp
  src/estimation/imu_ekf.cpp
  src/sensor/lidar_pointcloud_decoder.cpp
  src/sensor/sync.cpp
  src/sensor/lidar_decoder.cpp
  src/mapping/factors.cpp
  src/mapping/optimizers.cpp
  src/mapping/octree.cpp
  src/mapping/voxel_map.cpp
  src/pipeline/initialization.cpp
  src/pipeline/odometry.cpp
  src/pipeline/local_mapping.cpp
  src/platform/ros2/publishers.cpp
  src/platform/ros2/subscribers.cpp
  src/platform/ros2/io.cpp
  src/platform/ros2/node.cpp
)

# ===============================
# Executable Target
# ===============================
add_executable(${PROJECT_NAME}
  ${SOURCES}
)

# ===============================
# Include Directories
# ===============================
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ===============================
# Compile Options
# ===============================
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -g
    -O1
    -fsanitize=address
    -fno-omit-frame-pointer
    -w
  )
else()
  target_compile_options(${PROJECT_NAME} PRIVATE
    -O3
    -ffast-math
    -w
  )
endif()

# ===============================
# Target Dependencies
# ===============================
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  tf2_ros
  livox_ros_driver2

  # livox_ros_driver
)

# ===============================
# Link Libraries
# ===============================
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# ===============================
# Install Rules
# ===============================
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# ===============================
# Tests
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ===============================
# Export Package Information
# ===============================
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  Eigen3
  tf2_ros
)

ament_package()
