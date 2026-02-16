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
  explicit Keyframe(core::IMUST& _x0) : x0(_x0) {
    plptr = pcl::PointCloud<core::PointType>::Ptr(new pcl::PointCloud<core::PointType>);
  }

  /**
   * @brief Generate point cloud for publishing
   * @param pl_send Output point cloud
   * @param rot Additional rotation (default identity)
   * @param tra Additional translation (default zero)
   */
  void generate(pcl::PointCloud<core::PointType>& pl_send,
                Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0)) {
    pl_send.clear();
    pl_send.reserve(plptr->size());

    Eigen::Matrix3d R_total = rot * x0.R;
    Eigen::Vector3d t_total = rot * x0.p + tra;

    for (const auto& pt : plptr->points) {
      core::PointType pt_new;
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      p = R_total * p + t_total;
      pt_new.x = p.x();
      pt_new.y = p.y();
      pt_new.z = p.z();
      pt_new.intensity = pt.intensity;
      pl_send.push_back(pt_new);
    }
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility
using vina_slam::mapping::Keyframe;
