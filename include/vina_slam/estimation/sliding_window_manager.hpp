/**
 * @file sliding_window_manager.hpp
 * @brief Sliding window management for VINA-SLAM optimization
 *
 * Handles:
 * - State buffer management (x_buf)
 * - Point cloud buffer management (pvec_buf)
 * - IMU preintegration buffer management (imu_pre_buf)
 * - Sliding window operations (add, shift, cleanup)
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/voxel_map.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <deque>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace vina_slam {
namespace estimation {

/// Maximum size for slide window pool before cleanup
constexpr int kSlideWindowMaxSize = 100000;

/// Number of slide windows to delete in batch cleanup
constexpr int kSlideWindowBatchDelete = 5000;

/**
 * @brief Sliding window manager for state estimation
 *
 * Manages buffers for sliding window optimization including:
 * - State estimates (poses, velocities, biases)
 * - Point cloud data
 * - IMU preintegration factors
 * - Slide window objects for plane tracking
 */
class SlidingWindowManager {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// Window size (maximum frames in window)
  int win_size = 10;

  /// Current frame count in window
  int win_count = 0;

  /// Window base index (for path tracking)
  int win_base = 0;

  /// Thread count for parallel processing
  int thread_num = 5;

  /// State buffer (poses, velocities, biases)
  std::vector<core::IMUST> x_buf;

  /// Point cloud buffer
  std::vector<core::PVecPtr> pvec_buf;

  /// IMU preintegration buffer
  std::deque<ImuPreintegration*> imu_pre_buf;

  /// Sliding window pools (one per thread)
  std::vector<std::vector<SlideWindow*>> sws;

  /**
   * @brief Default constructor
   */
  SlidingWindowManager() = default;

  /**
   * @brief Constructor with ROS node
   * @param node_in ROS node pointer
   */
  explicit SlidingWindowManager(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance with node initialization
   * @param node_in ROS node pointer
   * @return Reference to singleton instance
   */
  static SlidingWindowManager& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance (must be initialized first)
   * @return Reference to singleton instance
   */
  static SlidingWindowManager& instance();

  /**
   * @brief Initialize the sliding window
   * @param window_size Maximum frames in window
   * @param num_threads Number of threads for parallel processing
   */
  void initialize(int window_size, int num_threads);

  /**
   * @brief Add a new frame to the sliding window
   * @param x_curr Current state
   * @param pptr Point cloud data
   * @param imus IMU measurements for preintegration
   * @return true if window is now full
   */
  bool addFrame(const core::IMUST& x_curr, core::PVecPtr pptr,
                std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus);

  /**
   * @brief Check if window is full
   * @return true if window is full
   */
  bool isWindowFull() const { return win_count >= win_size; }

  /**
   * @brief Check if window is empty
   * @return true if window is empty
   */
  bool isWindowEmpty() const { return win_count == 0; }

  /**
   * @brief Shift sliding window by removing old frames
   * @param mgsize Number of frames to marginalize
   */
  void shiftWindow(int mgsize);

  /**
   * @brief Cleanup oversized slide window pool
   */
  void cleanupSlideWindows();

  /**
   * @brief Reset all buffers
   */
  void reset();

  /**
   * @brief Get current state
   * @return Current IMU state
   */
  core::IMUST getCurrentState() const;

  /**
   * @brief Get last state in buffer
   * @return Last IMU state
   */
  core::IMUST getLastState() const;

  /**
   * @brief Get slide window pool for thread 0
   * @return Reference to main slide window pool
   */
  std::vector<SlideWindow*>& getSlideWindowPool() { return sws[0]; }

  /**
   * @brief Get all slide window pools
   * @return Reference to all slide window pools
   */
  std::vector<std::vector<SlideWindow*>>& getAllSlideWindowPools() { return sws; }

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  bool add_frame(const core::IMUST& x_curr, core::PVecPtr pptr,
                 std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus) {
    return addFrame(x_curr, pptr, imus);
  }
  bool is_window_full() const { return isWindowFull(); }
  bool is_window_empty() const { return isWindowEmpty(); }
  void shift_window(int mgsize) { shiftWindow(mgsize); }
  void cleanup_slide_windows() { cleanupSlideWindows(); }
  core::IMUST get_current_state() const { return getCurrentState(); }
  core::IMUST get_last_state() const { return getLastState(); }
};

} // namespace estimation
} // namespace vina_slam

// Backward compatibility
using vina_slam::estimation::SlidingWindowManager;
