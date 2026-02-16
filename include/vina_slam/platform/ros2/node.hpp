/**
 * @file node.hpp
 * @brief ROS2 node wrapper for VINA-SLAM
 *
 * Handles:
 * - Parameter loading
 * - Node lifecycle management
 * - Configuration management
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace vina_slam {
namespace platform {
namespace ros2 {

/**
 * @brief Configuration parameters for VINA-SLAM
 */
struct VinaSlamConfig {
  // General parameters
  std::string bagname;
  std::string save_path;
  std::string lid_topic;
  std::string imu_topic;
  int lidar_type = 0;
  double blind = 0.1;
  int point_filter_num = 3;
  int is_save_map = 0;
  int if_loop_dect = 0;
  int if_BA = 0;

  // Extrinsics
  Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();

  // Odometry parameters
  double cov_gyr = 0.1;
  double cov_acc = 0.1;
  double rand_walk_gyr = 0.1;
  double rand_walk_acc = 0.1;

  // Mapping parameters
  double voxel_size = 1.0;
  int max_layer = 2;
  int max_points = 100;
  double plane_threshold = 0.01;
  int win_size = 20;

  // Localization mode
  bool localization_mode = false;
  std::string map_path;

  /**
   * @brief Print configuration to console
   */
  void print() const;
};

/**
 * @brief ROS2 node wrapper for VINA-SLAM
 *
 * Encapsulates all ROS2-specific functionality including
 * parameter handling, subscriber/publisher management.
 */
class VinaSlamNode {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// Configuration
  VinaSlamConfig config;

  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit VinaSlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Get singleton instance
   * @param node_in ROS node pointer
   * @return Reference to singleton
   */
  static VinaSlamNode& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton (must be initialized first)
   * @return Reference to singleton
   */
  static VinaSlamNode& instance();

  /**
   * @brief Load configuration from ROS parameters
   * @return Loaded configuration
   */
  VinaSlamConfig loadConfig();

  /**
   * @brief Get LiDAR-IMU extrinsics as IMUST
   * @return Extrinsics state
   */
  core::IMUST getExtrinsics() const;

  /**
   * @brief Get ROS node pointer
   * @return Node pointer
   */
  rclcpp::Node::SharedPtr getNode() const { return node; }
};

} // namespace ros2
} // namespace platform
} // namespace vina_slam

// Backward compatibility
using vina_slam::platform::ros2::VinaSlamConfig;
using vina_slam::platform::ros2::VinaSlamNode;
