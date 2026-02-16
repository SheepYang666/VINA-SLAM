cmake_minimum_required(VERSION 3.8)
project(vina_slam LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===============================
# CMake Policies
# ===============================
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# ===============================
# Build Options
# ===============================
option(VINA_SLAM_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(VINA_SLAM_ENABLE_ASAN "Enable AddressSanitizer (Debug only)" OFF)

# ===============================
# Default Build Type
# ===============================
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "\nBuild type: ${CMAKE_BUILD_TYPE}\n")

# ===============================
# Find Dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# ===============================
# Source Files
# ===============================
# Core module sources
set(CORE_SOURCES
  src/core/types.cpp
  src/core/sensor_context.cpp
)

# Sensor module sources
set(SENSOR_SOURCES
  src/sensor/lidar_decoder.cpp
)

# Estimation module sources
set(ESTIMATION_SOURCES
  src/estimation/imu_ekf.cpp
  src/estimation/imu_preintegration.cpp
  src/estimation/state_estimator.cpp
  src/estimation/sliding_window_manager.cpp
)

# Mapping module sources
set(MAPPING_SOURCES
  src/mapping/factors.cpp
  src/mapping/optimizers.cpp
  src/mapping/octree.cpp
  src/mapping/voxel_map.cpp
  src/mapping/voxel_map_manager.cpp
)

# Pipeline module sources
set(PIPELINE_SOURCES
  src/pipeline/initialization.cpp
  src/pipeline/odometry.cpp
  src/pipeline/local_mapping.cpp
)

# Platform module sources (ROS2)
set(PLATFORM_SOURCES
  src/platform/ros2/node.cpp
  src/platform/ros2/publishers.cpp
  src/platform/ros2/subscribers.cpp
  src/platform/ros2/io.cpp
)

# Main sources (to be refactored in future phases)
# Note: BTC.cpp and loop_detection.cpp removed - not needed for localization mode
set(MAIN_SOURCES
  src/VINASlam.cpp
  src/voxel_map.cpp
)

set(SOURCES
  ${CORE_SOURCES}
  ${SENSOR_SOURCES}
  ${ESTIMATION_SOURCES}
  ${MAPPING_SOURCES}
  ${PIPELINE_SOURCES}
  ${PLATFORM_SOURCES}
  ${MAIN_SOURCES}
)

# ===============================
# Executable Target
# ===============================
add_executable(${PROJECT_NAME} ${SOURCES})

# ===============================
# Include Directories
# ===============================
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# ===============================
# Compile Options
# ===============================
if(VINA_SLAM_ENABLE_WARNINGS)
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
  )
endif()

if(VINA_SLAM_ENABLE_ASAN)
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address;-fno-omit-frame-pointer>
  )
  target_link_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address>
  )
endif()

# ===============================
# Dependencies
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
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
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
