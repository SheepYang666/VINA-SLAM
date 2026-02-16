/**
 * @file imu_buffer.hpp
 * @brief Thread-safe IMU data buffer for sensor synchronization
 *
 * This module provides a thread-safe buffer for IMU measurements
 * and synchronization utilities for IMU-LiDAR data pairing.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <deque>
#include <mutex>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vina_slam {
namespace sensor {

/**
 * @brief Thread-safe IMU data buffer
 *
 * Stores IMU measurements in a thread-safe deque with mutex protection.
 * Used for IMU-LiDAR synchronization in the SLAM pipeline.
 */
class ImuBuffer {
public:
  using ImuPtr = sensor_msgs::msg::Imu::SharedPtr;
  using ImuConstPtr = sensor_msgs::msg::Imu::SharedPtr;

  ImuBuffer() = default;
  ~ImuBuffer() = default;

  // Non-copyable
  ImuBuffer(const ImuBuffer&) = delete;
  ImuBuffer& operator=(const ImuBuffer&) = delete;

  /**
   * @brief Push an IMU measurement into the buffer
   * @param imu IMU measurement shared pointer
   */
  void push(const ImuPtr& imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(imu);
    updateLatestTime();
  }

  /**
   * @brief Pop the front IMU measurement
   * @return Front IMU measurement, or nullptr if empty
   */
  ImuPtr popFront() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
      return nullptr;
    }
    auto imu = buffer_.front();
    buffer_.pop_front();
    return imu;
  }

  /**
   * @brief Get the front IMU measurement without removing
   * @return Front IMU measurement, or nullptr if empty
   */
  ImuPtr front() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
      return nullptr;
    }
    return buffer_.front();
  }

  /**
   * @brief Check if buffer is empty
   */
  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty();
  }

  /**
   * @brief Get buffer size
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  /**
   * @brief Clear the buffer
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

  /**
   * @brief Get the timestamp of the latest IMU measurement
   * @return Latest timestamp in seconds, or -1 if empty
   */
  double getLatestTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_time_;
  }

  /**
   * @brief Extract IMU measurements within a time range
   * @param start_time Start time (seconds)
   * @param end_time End time (seconds)
   * @param output Output deque to store extracted IMUs
   * @return Number of IMUs extracted
   */
  size_t extractInRange(double /*start_time*/, double end_time,
                        std::deque<ImuPtr>& output) {
    std::lock_guard<std::mutex> lock(mutex_);

    size_t count = 0;
    while (!buffer_.empty()) {
      double imu_time = getImuTime(buffer_.front());
      if (imu_time < end_time) {
        output.push_back(buffer_.front());
        buffer_.pop_front();
        count++;
      } else {
        break;
      }
    }
    return count;
  }

private:
  mutable std::mutex mutex_;
  std::deque<ImuPtr> buffer_;
  double latest_time_ = -1.0;

  void updateLatestTime() {
    if (!buffer_.empty()) {
      latest_time_ = getImuTime(buffer_.back());
    }
  }

  static double getImuTime(const ImuPtr& imu) {
    return rclcpp::Time(imu->header.stamp).seconds();
  }
};

/**
 * @brief Point cloud buffer entry with timestamp
 */
struct PointCloudEntry {
  pcl::PointCloud<PointType>::Ptr cloud;
  double timestamp;
};

/**
 * @brief Thread-safe point cloud buffer
 */
class PointCloudBuffer {
public:
  void push(const pcl::PointCloud<PointType>::Ptr& cloud, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back({cloud, timestamp});
  }

  PointCloudEntry popFront() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
      return {nullptr, 0.0};
    }
    auto entry = buffer_.front();
    buffer_.pop_front();
    return entry;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

private:
  mutable std::mutex mutex_;
  std::deque<PointCloudEntry> buffer_;
};

} // namespace sensor
} // namespace vina_slam

// Backward compatibility aliases
using vina_slam::sensor::ImuBuffer;
using vina_slam::sensor::PointCloudBuffer;
