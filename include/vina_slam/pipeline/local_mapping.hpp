/**
 * @file local_mapping.hpp
 * @brief Local mapping pipeline for VINA-SLAM
 *
 * Handles:
 * - Voxel map management (recut, marginalization)
 * - Bundle adjustment optimization
 * - Point cloud insertion and organization
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <deque>
#include <unordered_map>

namespace vina_slam {
namespace pipeline {

/**
 * @brief Local mapping pipeline for map maintenance
 *
 * Manages the voxel map, performs recut operations for
 * tree subdivision, and handles marginalization of old frames.
 */
class LocalMapping {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// LiDAR factor for optimization
  mapping::LidarFactor lidar_factor;

  /// Normal factor for optimization
  mapping::NormalFactor normal_factor;

  /// LiDAR-Inertial BA optimizer
  mapping::LIBAOptimizer li_optimizer;

  /// LiDAR-Inertial BA optimizer with gravity
  mapping::LIBAOptimizerGravity li_optimizer_gravity;

  /// Window size for optimization
  int win_size = 0;

  /**
   * @brief Constructor
   */
  LocalMapping();

  /**
   * @brief Initialize with window size
   * @param window_size Optimization window size
   */
  void initialize(int window_size);

  /**
   * @brief Multi-threaded recut operation
   * @param win_count Current frame count in window
   * @param x_buf State buffer
   * @param surf_map Surface voxel map
   * @param sws Sliding window pools
   *
   * Subdivides voxels based on point distribution
   * for better plane fitting.
   */
  void multiRecut(int win_count, std::vector<core::IMUST>& x_buf,
                  std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                  std::vector<std::vector<mapping::SlideWindow*>>& sws);

  /**
   * @brief Multi-threaded marginalization
   * @param win_count Current frame count
   * @param mgsize Number of frames to marginalize
   * @param x_buf State buffer
   * @param surf_map Surface voxel map
   *
   * Removes oldest frames from the sliding window
   * to maintain fixed window size.
   */
  void multiMargi(int win_count, int mgsize, std::vector<core::IMUST>& x_buf,
                  std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map);

  /**
   * @brief Collect factors for optimization
   * @param surf_map Surface voxel map
   * @param x_buf State buffer
   * @param sws Sliding window pools
   */
  void collectFactors(std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                      std::vector<core::IMUST>& x_buf,
                      std::vector<std::vector<mapping::SlideWindow*>>& sws);

  /**
   * @brief Run bundle adjustment optimization
   * @param x_buf State buffer (modified in place)
   * @param imu_pre_buf IMU preintegration factors
   * @param hess Output Hessian matrix
   * @param with_gravity Whether to optimize gravity direction
   */
  void runOptimization(std::vector<core::IMUST>& x_buf,
                       std::deque<estimation::ImuPreintegration*>& imu_pre_buf,
                       Eigen::MatrixXd* hess, bool with_gravity = false);

  /**
   * @brief Clear factor storage
   */
  void clearFactors();

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void multi_recut(int win_count, std::vector<core::IMUST>& x_buf,
                   std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                   std::vector<std::vector<mapping::SlideWindow*>>& sws) {
    multiRecut(win_count, x_buf, surf_map, sws);
  }
  void multi_margi(int win_count, int mgsize, std::vector<core::IMUST>& x_buf,
                   std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map) {
    multiMargi(win_count, mgsize, x_buf, surf_map);
  }
};

} // namespace pipeline
} // namespace vina_slam

// Backward compatibility
using vina_slam::pipeline::LocalMapping;
