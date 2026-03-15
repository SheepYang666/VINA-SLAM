/**
 * @file keyframe.hpp
 * @brief Keyframe structure for mapping
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <pcl/point_cloud.h>

namespace vina_slam {
namespace mapping {

/**
 * @brief Keyframe containing a collection of scans
 *
 * A keyframe aggregates multiple consecutive scans (typically 10)
 * into a single local map for loop closure and global mapping.
 */
struct Keyframe {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Initial pose of the keyframe
  core::IMUST x0;

  /// Point cloud pointer (in world frame)
  pcl::PointCloud<core::PointType>::Ptr plptr;

  /// Whether keyframe is valid/active
  int exist = 0;

  /// Keyframe ID
  int id = 0;

  /// Matching index for loop closure
  int mp = 0;

  /// Journey/distance traveled
  float jour = 0.0f;

  /**
   * @brief Constructor with initial pose
   * @param _x0 Initial pose
   */
  explicit Keyframe(core::IMUST& _x0) : x0(_x0), exist(0) {
    plptr.reset(new pcl::PointCloud<core::PointType>());
  }

  /**
   * @brief Generate point cloud for publishing
   * @param pl_send Output point cloud (points are appended)
   * @param rot Additional rotation (default identity)
   * @param tra Additional translation (default zero)
   */
  void generate(pcl::PointCloud<core::PointType>& pl_send,
                Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0)) {
    Eigen::Vector3d v3;
    for (core::PointType ap : plptr->points) {
      v3 << ap.x, ap.y, ap.z;
      v3 = rot * v3 + tra;
      ap.x = v3[0];
      ap.y = v3[1];
      ap.z = v3[2];
      pl_send.push_back(ap);
    }
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility
using vina_slam::mapping::Keyframe;
