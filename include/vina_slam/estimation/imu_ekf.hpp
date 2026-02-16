/**
 * @file imu_ekf.hpp
 * @brief IMU EKF state propagation and motion blur correction
 *
 * This module implements IMU-based state propagation using an
 * Iterated Extended Kalman Filter (IEKF) with motion deblurring.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <deque>
#include <memory>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vina_slam {
namespace estimation {

/**
 * @brief IMU EKF for state propagation and motion blur correction
 *
 * Handles:
 * - IMU initialization (gravity alignment)
 * - State propagation between LiDAR frames
 * - Motion blur correction for point clouds
 */
class ImuEkf {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // State flags
  bool init_flag = false;
  int init_num = 0;
  int min_init_num = 30;

  // Timing
  double pcl_beg_time = 0.0;
  double pcl_end_time = 0.0;
  double last_pcl_end_time = 0.0;

  // IMU statistics for initialization
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();

  // Last IMU measurement
  std::shared_ptr<sensor_msgs::msg::Imu> last_imu;
  Eigen::Vector3d angvel_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_s_last = Eigen::Vector3d::Zero();

  // Noise parameters
  Eigen::Vector3d cov_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d cov_gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d cov_bias_gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d cov_bias_acc = Eigen::Vector3d::Zero();

  // LiDAR-IMU extrinsics
  Eigen::Matrix3d Lid_rot_to_IMU = Eigen::Matrix3d::Identity();
  Eigen::Vector3d Lid_offset_to_IMU = Eigen::Vector3d::Zero();

  // Gravity scaling
  double scale_gravity = 1.0;

  // IMU poses for motion blur correction
  std::vector<core::IMUST> imu_poses;

  // Configuration
  int point_notime = 0;
  std::string imu_topic = "";

  ImuEkf();

  /**
   * @brief Motion blur correction for point cloud
   * @param xc Current state (will be updated)
   * @param pcl_in Point cloud to correct (will be modified in-place)
   * @param imus IMU measurements for the frame
   */
  void motionBlur(core::IMUST& xc, pcl::PointCloud<core::PointType>& pcl_in,
                  std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus);

  /**
   * @brief Initialize IMU from static measurements
   * @param imus IMU measurements
   */
  void imuInit(std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus);

  /**
   * @brief Process IMU data and update state
   * @param x_curr Current state (will be updated)
   * @param pcl_in Point cloud (for motion blur correction)
   * @param imus IMU measurements
   * @return 0 if initializing, 1 if processed
   */
  int process(core::IMUST& x_curr, pcl::PointCloud<core::PointType>& pcl_in,
              std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus);

  /**
   * @brief Set noise parameters
   */
  void setNoiseParameters(const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyr,
                          const Eigen::Vector3d& cov_bias_acc, const Eigen::Vector3d& cov_bias_gyr);

  /**
   * @brief Set LiDAR-IMU extrinsics
   */
  void setExtrinsics(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void motion_blur(core::IMUST& xc, pcl::PointCloud<core::PointType>& pcl_in,
                   std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus) {
    motionBlur(xc, pcl_in, imus);
  }
  void IMU_init(std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus) { imuInit(imus); }
};

} // namespace estimation
} // namespace vina_slam

// Backward compatibility
using vina_slam::estimation::ImuEkf;
using IMUEKF = vina_slam::estimation::ImuEkf;
