/**
 * @file sync.hpp
 * @brief IMU-LiDAR synchronization utilities
 *
 * This module provides synchronization logic for pairing IMU
 * measurements with LiDAR point cloud frames.
 */

#pragma once

#include "vina_slam/sensor/imu_buffer.hpp"
#include "vina_slam/core/types.hpp"
#include <rclcpp/time.hpp>

namespace vina_slam {
namespace sensor {

/**
 * @brief Synchronized sensor data package
 */
struct SyncedData {
  pcl::PointCloud<PointType>::Ptr cloud;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imus;
  double pcl_beg_time;
  double pcl_end_time;
  bool valid;
};

/**
 * @brief Data synchronizer for IMU-LiDAR pairing
 *
 * Manages the synchronization between IMU measurements and LiDAR frames.
 * Ensures that each LiDAR frame has corresponding IMU measurements.
 */
class DataSynchronizer {
public:
  using ImuPtr = sensor_msgs::msg::Imu::SharedPtr;

  DataSynchronizer()
    : point_notime_(false), last_pcl_time_(-1.0), pl_ready_(false) {}

  /**
   * @brief Set point no-time mode
   * @param enabled If true, simulate point timestamps from frame intervals
   */
  void setPointNoTime(bool enabled) { point_notime_ = enabled; }

  /**
   * @brief Synchronize point cloud with IMU data
   *
   * This function implements the sync_packages logic:
   * 1. Get a point cloud from buffer
   * 2. Determine the time range [pcl_beg_time, pcl_end_time]
   * 3. Extract corresponding IMU measurements
   *
   * @param pcl_buffer Point cloud buffer
   * @param imu_buffer IMU buffer
   * @param output Output synchronized data
   * @return true if synchronization successful
   */
  bool synchronize(PointCloudBuffer& pcl_buffer, ImuBuffer& imu_buffer,
                   SyncedData& output) {
    // Step 1: Get point cloud if not ready
    if (!pl_ready_) {
      if (pcl_buffer.empty()) {
        return false;
      }

      auto entry = pcl_buffer.popFront();
      if (!entry.cloud) {
        return false;
      }

      output.cloud = entry.cloud;
      output.pcl_beg_time = entry.timestamp;
      output.pcl_end_time = output.pcl_beg_time;

      // Calculate end time from last point's curvature
      if (!output.cloud->empty()) {
        output.pcl_end_time += output.cloud->back().curvature;
      }

      // Handle no-time mode
      if (point_notime_) {
        if (last_pcl_time_ < 0) {
          last_pcl_time_ = output.pcl_beg_time;
          return false;
        }
        double temp = output.pcl_end_time;
        output.pcl_end_time = output.pcl_beg_time;
        output.pcl_beg_time = last_pcl_time_;
        last_pcl_time_ = temp;
      }

      pl_ready_ = true;
      current_beg_time_ = output.pcl_beg_time;
      current_end_time_ = output.pcl_end_time;
    }

    // Step 2: Check if IMU data covers the point cloud time range
    if (imu_buffer.getLatestTime() <= current_end_time_) {
      return false;
    }

    // Step 3: Extract IMU measurements in range
    imu_buffer.extractInRange(current_beg_time_, current_end_time_, output.imus);

    // Check if we have enough IMU data
    if (imu_buffer.empty()) {
      // Data flow broken
      output.valid = false;
      return false;
    }

    pl_ready_ = false;  // Reset for next frame

    // Valid if we have enough IMU measurements
    output.valid = output.imus.size() > 4;
    output.pcl_beg_time = current_beg_time_;
    output.pcl_end_time = current_end_time_;

    return output.valid;
  }

  /**
   * @brief Reset synchronizer state
   */
  void reset() {
    pl_ready_ = false;
    last_pcl_time_ = -1.0;
    current_beg_time_ = 0.0;
    current_end_time_ = 0.0;
  }

private:
  bool point_notime_;
  double last_pcl_time_;
  bool pl_ready_;
  double current_beg_time_;
  double current_end_time_;
};

/**
 * @brief Utility function for point cloud sorting
 * @param a First point
 * @param b Second point
 * @return true if a.curvature < b.curvature
 */
inline bool comparePointCurvature(const PointType& a, const PointType& b) {
  return a.curvature < b.curvature;
}

/**
 * @brief Sort point cloud by curvature (timestamp)
 * @param cloud Point cloud to sort in-place
 */
inline void sortPointCloudByTime(pcl::PointCloud<PointType>& cloud) {
  std::sort(cloud.begin(), cloud.end(), comparePointCurvature);
}

/**
 * @brief Filter points with curvature > threshold
 * @param cloud Point cloud to filter in-place
 * @param max_curvature Maximum allowed curvature (default 0.11)
 */
inline void filterPointCloudByTime(pcl::PointCloud<PointType>& cloud,
                                   double max_curvature = 0.11) {
  while (!cloud.empty() && cloud.back().curvature > max_curvature) {
    cloud.points.pop_back();
  }
}

} // namespace sensor
} // namespace vina_slam

// Backward compatibility
using vina_slam::sensor::SyncedData;
using vina_slam::sensor::DataSynchronizer;
using vina_slam::sensor::comparePointCurvature;
using vina_slam::sensor::sortPointCloudByTime;
using vina_slam::sensor::filterPointCloudByTime;
