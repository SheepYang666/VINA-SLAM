/**
 * @file state_estimator.cpp
 * @brief Implementation of LIO state estimator
 */

#include "vina_slam/estimation/state_estimator.hpp"

namespace vina_slam {
namespace estimation {

LioStateEstimator::LioStateEstimator() {
  current_state_.setZero();
}

LioStateEstimator::LioStateEstimator(const StateEstimatorConfig& config)
    : config_(config) {
  current_state_.setZero();

  // Configure IMU EKF
  imu_ekf_.cov_acc = Eigen::Vector3d::Constant(config_.cov_acc);
  imu_ekf_.cov_gyr = Eigen::Vector3d::Constant(config_.cov_gyr);
  imu_ekf_.cov_bias_acc = Eigen::Vector3d::Constant(config_.rdw_acc);
  imu_ekf_.cov_bias_gyr = Eigen::Vector3d::Constant(config_.rdw_gyr);
  imu_ekf_.point_notime = config_.point_notime;
}

EstimationResult LioStateEstimator::estimate(pcl::PointCloud<core::PointType>& cloud,
                                             std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                                             core::IMUST& state) {
  EstimationResult result;

  // Process IMU data
  int process_result = imu_ekf_.process(state, cloud, imus);

  if (process_result == 0) {
    // Still initializing
    result.success = false;
    result.state = state;
    return result;
  }

  // State has been propagated by IMU
  result.success = true;
  result.state = state;
  current_state_ = state;

  return result;
}

void LioStateEstimator::reset() {
  current_state_.setZero();
  imu_ekf_.init_flag = false;
  imu_ekf_.init_num = 0;
  imu_ekf_.mean_acc.setZero();
  imu_ekf_.mean_gyr.setZero();
}

} // namespace estimation
} // namespace vina_slam
