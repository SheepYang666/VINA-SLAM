/**
 * @file slide_window.hpp
 * @brief Sliding window for temporal point storage in voxels
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <vector>

namespace vina_slam {
namespace mapping {

/**
 * @brief Sliding window storing points per frame in a voxel
 *
 * Each voxel maintains a sliding window of points from multiple frames.
 * This enables temporal tracking of points for plane fitting and
 * optimization.
 */
class SlideWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Points for each frame in the sliding window
   * Each element is a vector of pointVar (point + covariance)
   */
  std::vector<core::PVec> points;

  /**
   * @brief Point cluster statistics for each frame
   * Contains mean, covariance, and count information
   */
  std::vector<core::PointCluster> pcrs_local;

  /**
   * @brief Constructor with window size
   * @param wdsize Number of frames in the sliding window
   */
  explicit SlideWindow(int wdsize) { resize(wdsize); }

  /**
   * @brief Default constructor
   */
  SlideWindow() = default;

  /**
   * @brief Resize the sliding window
   * @param wdsize New window size
   */
  void resize(int wdsize) {
    points.resize(wdsize);
    pcrs_local.resize(wdsize);
    clear();
  }

  /**
   * @brief Clear all points in the window
   */
  void clear() {
    for (size_t i = 0; i < points.size(); i++) {
      points[i].clear();
      pcrs_local[i].clear();
    }
  }

  /**
   * @brief Get window size
   * @return Number of frames in window
   */
  int size() const { return static_cast<int>(points.size()); }

  /**
   * @brief Add point to a specific frame
   * @param frame_idx Frame index in window
   * @param pv Point with variance
   */
  void pushPoint(int frame_idx, const core::pointVar& pv) {
    if (frame_idx >= 0 && frame_idx < static_cast<int>(points.size())) {
      points[frame_idx].push_back(pv);
      pcrs_local[frame_idx].push(pv.pnt);
    }
  }

  /**
   * @brief Get total point count across all frames
   * @return Total number of points
   */
  int totalPoints() const {
    int total = 0;
    for (const auto& frame_points : points) {
      total += static_cast<int>(frame_points.size());
    }
    return total;
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility alias removed - use voxel_map.hpp for global SlideWindow
