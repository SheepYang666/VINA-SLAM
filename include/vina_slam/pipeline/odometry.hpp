/**
 * @file odometry.hpp
 * @brief Odometry pipeline for VINA-SLAM
 *
 * Handles:
 * - EKF state propagation
 * - LiDAR-Inertial state estimation
 * - Point-to-plane matching
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/estimation/imu_ekf.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <deque>
#include <pcl/point_cloud.h>

namespace vina_slam {
namespace pipeline {

/**
 * @brief Odometry pipeline for state estimation
 *
 * Processes incoming LiDAR frames and IMU measurements to
 * estimate robot trajectory using LiDAR-Inertial fusion.
 */
class OdometryPipeline {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// IMU EKF for propagation
  estimation::ImuEkf imu_ekf;

  /// Current state estimate
  core::IMUST x_curr;

  /// State buffer for sliding window
  std::vector<core::IMUST> x_buf;

  /// IMU preintegration factors
  std::deque<estimation::ImuPreintegration*> imu_pre_buf;

  /// Window size for optimization
  int win_size = 0;

  /// Maximum window size before marginalization
  int max_win_size = 20;

  /// Whether system is initialized
  bool is_initialized = false;

  /// LiDAR-IMU extrinsics
  core::IMUST extrin_para;

  /**
   * @brief Constructor
   */
  OdometryPipeline();

  /**
   * @brief Initialize pipeline with window size
   * @param window_size Optimization window size
   */
  void initialize(int window_size);

  /**
   * @brief Set IMU noise parameters
   * @param cov_acc Accelerometer noise covariance
   * @param cov_gyr Gyroscope noise covariance
   * @param cov_bias_acc Accelerometer bias random walk
   * @param cov_bias_gyr Gyroscope bias random walk
   */
  void setImuNoise(const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyr,
                   const Eigen::Vector3d& cov_bias_acc, const Eigen::Vector3d& cov_bias_gyr);

  /**
   * @brief Set LiDAR-IMU extrinsics
   * @param rotation Rotation from LiDAR to IMU
   * @param translation Translation from LiDAR to IMU
   */
  void setExtrinsics(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

  /**
   * @brief Process IMU data for propagation
   * @param imus IMU measurements
   * @param pcl_in Point cloud for motion blur correction
   * @return 0 if initializing, 1 if processed
   */
  int processImu(std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                 pcl::PointCloud<core::PointType>& pcl_in);

  /**
   * @brief LiDAR-Inertial state estimation
   * @param pvec Points with variance
   * @param surf_map Surface voxel map
   * @param sws Sliding window pools
   * @return Number of matched points
   */
  int stateEstimation(core::PVec& pvec,
                      std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                      std::vector<mapping::SlideWindow*>& sws);

  /**
   * @brief Add new frame to sliding window
   * @param pvec Points for new frame
   */
  void addFrame(core::PVecPtr pvec);

  /**
   * @brief Get current pose estimate
   * @return Current state
   */
  const core::IMUST& getCurrentState() const { return x_curr; }

  /**
   * @brief Check if pipeline is initialized
   * @return true if initialized
   */
  bool isInitialized() const { return is_initialized; }

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  int lio_state_estimation(core::PVec& pvec,
                           std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                           std::vector<mapping::SlideWindow*>& sws) {
    return stateEstimation(pvec, surf_map, sws);
  }
};

} // namespace pipeline
} // namespace vina_slam

// Backward compatibility
using vina_slam::pipeline::OdometryPipeline;
