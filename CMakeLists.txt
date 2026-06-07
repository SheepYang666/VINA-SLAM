cmake_minimum_required(VERSION 3.0.2)
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
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  tf
  livox_ros_driver
)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

message(STATUS "Eigen version: ${EIGEN_VERSION}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs nav_msgs visualization_msgs pcl_conversions pcl_ros tf
                 livox_ros_driver
  DEPENDS Eigen3 PCL
)

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
  src/platform/ros1/publishers.cpp
  src/platform/ros1/subscribers.cpp
  src/platform/ros1/io.cpp
  src/platform/ros1/node.cpp
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
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

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
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ===============================
# Tests
# ===============================
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(lidar_mid360_pointcloud_decoder_test
    test/lidar_mid360_pointcloud_decoder_test.cpp
    src/sensor/lidar_pointcloud_decoder.cpp
  )
  if(TARGET lidar_mid360_pointcloud_decoder_test)
    target_include_directories(lidar_mid360_pointcloud_decoder_test PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(lidar_mid360_pointcloud_decoder_test
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
endif()

# ===============================
# Install Rules
# ===============================
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
