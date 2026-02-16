/**
 * @file initialization.hpp
 * @brief System initialization for VINA-SLAM
 *
 * Handles:
 * - Motion-based initialization
 * - Gravity alignment
 * - Motion blur correction for initialization frames
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/voxel_map.hpp"  // Use global types (LidarFactor, OctoTree, SlideWindow)
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <deque>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vina_slam {
namespace pipeline {

/**
 * @brief System initialization handler
 *
 * Performs motion-based initialization using multiple frames
 * to estimate initial poses and gravity direction.
 */
class Initialization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer for parameter access and logging
  rclcpp::Node::SharedPtr node;

  /// Whether point cloud has time information
  int point_notime = 0;

  /**
   * @brief Constructor with ROS node
   * @param node_in ROS node pointer
   */
  explicit Initialization(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance with node initialization
   * @param node_in ROS node pointer
   * @return Reference to singleton instance
   */
  static Initialization& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance (must be initialized first)
   * @return Reference to singleton instance
   */
  static Initialization& instance();

  /**
   * @brief Align gravity direction to vertical
   * @param xs State vector to align (modified in place)
   *
   * Rotates all states so that gravity points along z-axis.
   */
  void alignGravity(std::vector<core::IMUST>& xs);

  /**
   * @brief Motion blur correction for initialization
   * @param pl Input point cloud
   * @param pvec Output points with variance
   * @param xc Current IMU state
   * @param xl Last IMU state (for bias)
   * @param imus IMU measurements
   * @param pcl_beg_time Point cloud start time
   * @param extrin_para LiDAR-IMU extrinsics
   */
  void motionBlur(pcl::PointCloud<core::PointType>& pl, core::PVec& pvec,
                  core::IMUST xc, core::IMUST xl,
                  std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                  double pcl_beg_time, core::IMUST& extrin_para);

  /**
   * @brief Motion-based initialization
   * @param pl_origs Original point clouds for each frame
   * @param vec_imus IMU measurements for each frame
   * @param beg_times Start times for each frame
   * @param hess Output Hessian matrix
   * @param voxhess LidarFactor for optimization
   * @param x_buf State buffer (modified)
   * @param surf_map Surface voxel map
   * @param surf_map_slide Sliding surface voxel map
   * @param pvec_buf Point vector buffer
   * @param win_size Window size
   * @param sws Sliding window pools
   * @param x_curr Current state
   * @param imu_pre_buf IMU preintegration buffer
   * @param extrin_para LiDAR-IMU extrinsics
   * @return 1 if converged, 0 otherwise
   */
  int motionInit(std::vector<pcl::PointCloud<core::PointType>::Ptr>& pl_origs,
                 std::vector<std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>>& vec_imus,
                 std::vector<double>& beg_times, Eigen::MatrixXd* hess,
                 LidarFactor& voxhess, std::vector<core::IMUST>& x_buf,
                 std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map,
                 std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map_slide,
                 std::vector<core::PVecPtr>& pvec_buf, int win_size,
                 std::vector<std::vector<SlideWindow*>>& sws,
                 core::IMUST& x_curr, std::deque<estimation::ImuPreintegration*>& imu_pre_buf,
                 core::IMUST& extrin_para);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void align_gravity(std::vector<core::IMUST>& xs) { alignGravity(xs); }
  void motion_blur(pcl::PointCloud<core::PointType>& pl, core::PVec& pvec,
                   core::IMUST xc, core::IMUST xl,
                   std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                   double pcl_beg_time, core::IMUST& extrin_para) {
    motionBlur(pl, pvec, xc, xl, imus, pcl_beg_time, extrin_para);
  }
  int motion_init(std::vector<pcl::PointCloud<core::PointType>::Ptr>& pl_origs,
                  std::vector<std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>>& vec_imus,
                  std::vector<double>& beg_times, Eigen::MatrixXd* hess,
                  LidarFactor& voxhess, std::vector<core::IMUST>& x_buf,
                  std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map,
                  std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map_slide,
                  std::vector<core::PVecPtr>& pvec_buf, int win_size,
                  std::vector<std::vector<SlideWindow*>>& sws,
                  core::IMUST& x_curr, std::deque<estimation::ImuPreintegration*>& imu_pre_buf,
                  core::IMUST& extrin_para) {
    return motionInit(pl_origs, vec_imus, beg_times, hess, voxhess, x_buf,
                      surf_map, surf_map_slide, pvec_buf, win_size, sws,
                      x_curr, imu_pre_buf, extrin_para);
  }
};

} // namespace pipeline
} // namespace vina_slam

// Backward compatibility
using vina_slam::pipeline::Initialization;
