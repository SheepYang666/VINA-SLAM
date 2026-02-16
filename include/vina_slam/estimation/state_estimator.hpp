/**
 * @file state_estimator.hpp
 * @brief LIO state estimation interface
 *
 * This module defines the interface for LIO (LiDAR-Inertial Odometry)
 * state estimation, combining IMU propagation with LiDAR constraints.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/estimation/imu_ekf.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <pcl/point_cloud.h>

namespace vina_slam {
namespace estimation {

/**
 * @brief Configuration for state estimation
 */
struct StateEstimatorConfig {
  // IMU noise parameters
  double cov_gyr = 0.01;
  double cov_acc = 1.0;
  double rdw_gyr = 0.0001;  // Random walk
  double rdw_acc = 0.0001;

  // LiDAR parameters
  double down_size = 0.1;
  double voxel_size = 1.0;
  double dept_err = 0.02;
  double beam_err = 0.05;
  double min_eigen_value = 0.0025;
  double degrade_bound = 10.0;

  // Processing options
  int point_notime = 0;
};

/**
 * @brief Result of state estimation
 */
struct EstimationResult {
  bool success = false;
  core::IMUST state;
  int num_matched_points = 0;
  double residual = 0.0;
};

/**
 * @brief Interface for LIO state estimation
 *
 * Defines the common interface for different state estimation algorithms.
 */
class IStateEstimator {
public:
  virtual ~IStateEstimator() = default;

  /**
   * @brief Estimate state from sensor data
   * @param cloud Point cloud
   * @param imus IMU measurements
   * @param state Output state estimate
   * @return Estimation result
   */
  virtual EstimationResult estimate(pcl::PointCloud<core::PointType>& cloud,
                                    std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                                    core::IMUST& state) = 0;

  /**
   * @brief Get current state estimate
   */
  virtual const core::IMUST& getCurrentState() const = 0;

  /**
   * @brief Reset estimator state
   */
  virtual void reset() = 0;

  /**
   * @brief Check if initialized
   */
  virtual bool isInitialized() const = 0;
};

/**
 * @brief LIO state estimator using IEKF
 *
 * Implements LIO state estimation using an Iterated Extended Kalman Filter
 * with point-to-plane matching constraints.
 */
class LioStateEstimator : public IStateEstimator {
public:
  LioStateEstimator();
  explicit LioStateEstimator(const StateEstimatorConfig& config);

  EstimationResult estimate(pcl::PointCloud<core::PointType>& cloud,
                            std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                            core::IMUST& state) override;

  const core::IMUST& getCurrentState() const override { return current_state_; }
  void reset() override;
  bool isInitialized() const override { return imu_ekf_.init_flag; }

  /**
   * @brief Set configuration
   */
  void setConfig(const StateEstimatorConfig& config) { config_ = config; }

  /**
   * @brief Get IMU EKF (for direct access if needed)
   */
  ImuEkf& getImuEkf() { return imu_ekf_; }

private:
  StateEstimatorConfig config_;
  core::IMUST current_state_;
  ImuEkf imu_ekf_;
};

} // namespace estimation
} // namespace vina_slam

// Backward compatibility
using vina_slam::estimation::StateEstimatorConfig;
using vina_slam::estimation::EstimationResult;
using vina_slam::estimation::IStateEstimator;
using vina_slam::estimation::LioStateEstimator;
