/**
 * @file plane.hpp
 * @brief Plane representation and utilities for mapping
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <Eigen/Eigenvalues>

namespace vina_slam {
namespace mapping {

// Global configuration parameters (defined in voxel_map.cpp)
extern Eigen::Vector4d min_point;
extern double min_eigen_value;
extern std::vector<double> plane_eigen_value_thre;

/**
 * @brief Check if eigenvalues indicate a valid plane
 * @param eig_values Eigenvalues from covariance decomposition
 * @return true if the points form a plane
 *
 * A plane is characterized by one small eigenvalue (normal direction)
 * and two larger eigenvalues (plane directions).
 */
inline bool planeJudge(const Eigen::Vector3d& eig_values) {
  // Plane must have small eigenvalue in normal direction
  // and meet minimum eigenvalue threshold
  return eig_values(0) < min_eigen_value;
}

/**
 * @brief Compute covariance and eigenvalues for point cluster
 * @param cluster Point cluster with accumulated statistics
 * @param eig_values Output eigenvalues (sorted ascending)
 * @param eig_vectors Output eigenvectors (columns correspond to eigenvalues)
 */
inline void computePlaneEigen(const core::PointCluster& cluster,
                               Eigen::Vector3d& eig_values,
                               Eigen::Matrix3d& eig_vectors) {
  if (cluster.N == 0) {
    eig_values.setZero();
    eig_vectors.setIdentity();
    return;
  }

  Eigen::Vector3d vBar = cluster.v / cluster.N;
  Eigen::Matrix3d cov = cluster.P / cluster.N - vBar * vBar.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
  eig_values = saes.eigenvalues();
  eig_vectors = saes.eigenvectors();
}

/**
 * @brief Update plane from point cluster
 * @param plane Plane to update
 * @param cluster Point cluster statistics
 */
inline void updatePlaneFromCluster(core::Plane& plane, const core::PointCluster& cluster) {
  if (cluster.N == 0) {
    return;
  }

  plane.center = cluster.v / cluster.N;
  Eigen::Matrix3d cov = cluster.P / cluster.N - plane.center * plane.center.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
  Eigen::Vector3d eig_values = saes.eigenvalues();
  Eigen::Matrix3d eig_vectors = saes.eigenvectors();

  plane.normal = eig_vectors.col(0);
  plane.min_eig_value = eig_values(0);
  plane.mid_eig_value = eig_values(1);
  plane.max_eig_value = eig_values(2);
  plane.center_norm = plane.center.norm();
  plane.radius = sqrt(eig_values(2));
}

/**
 * @brief Check if point is close enough to plane
 * @param plane Plane to check against
 * @param point Point in world coordinates
 * @param threshold Maximum distance threshold
 * @return true if point is within threshold of plane
 */
inline bool isPointNearPlane(const core::Plane& plane, const Eigen::Vector3d& point, double threshold) {
  double dist = std::abs(plane.normal.dot(point - plane.center));
  return dist < threshold;
}

} // namespace mapping
} // namespace vina_slam

// Backward compatibility
using vina_slam::mapping::planeJudge;
using vina_slam::mapping::computePlaneEigen;
using vina_slam::mapping::updatePlaneFromCluster;
