// Backward-compatibility wrapper: includes all the new module headers
// that were split from the original monolithic VINASlam.hpp
#pragma once

// Sensor module
#include "vina_slam/sensor/imu_buffer.hpp"
#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/sensor/sync.hpp"

// Pipeline module
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/pipeline/odometry.hpp"
#include "vina_slam/pipeline/local_mapping.hpp"

// Platform/ROS2 module
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/subscribers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
#include "vina_slam/platform/ros2/node.hpp"
