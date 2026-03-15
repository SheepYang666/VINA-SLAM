/**
 * @file point_utils.hpp
 * @brief Point cloud utility functions for variance computation
 *
 * Contains functions for computing point variances in body/world frame,
 * initializing point vectors with covariance, and propagating state
 * uncertainty to point positions.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/core/constants.hpp"
#include "vina_slam/core/math.hpp"
#include <pcl/point_cloud.h>

namespace vina_slam {
namespace core {

/**
 * @brief Compute point variance in body frame using range/angle error model
 * @param pb Point in body frame (modified: z clamped if zero)
 * @param range_inc Range noise standard deviation
 * @param degree_inc Angular noise standard deviation (degrees)
 * @param var Output 3x3 covariance matrix
 */
void calcBodyVar(Eigen::Vector3d& pb, float range_inc, float degree_inc, Eigen::Matrix3d& var);

/**
 * @brief Initialize point vector with covariance from point cloud
 *
 * Transforms points from LiDAR frame to extrinsic frame and computes
 * per-point covariance using the range/angle error model.
 *
 * @param ext LiDAR-IMU extrinsic parameters
 * @param pl_cur Input point cloud (body frame)
 * @param pptr Output point vector with variance
 * @param dept_err Range error parameter
 * @param beam_err Beam angle error parameter
 */
void varInit(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr,
             double dept_err, double beam_err);

/**
 * @brief Update point variances and compute world-frame positions
 *
 * Propagates pose uncertainty (rotation + translation covariance)
 * to each point's variance and computes world-frame coordinates.
 *
 * @param pptr Point vector with variance (modified in place)
 * @param x_curr Current state with covariance
 * @param pwld Output world-frame point positions
 */
void pvecUpdate(PVecPtr pptr, IMUST& x_curr, PLV(3)& pwld);

/**
 * @brief Get current process memory usage in GB
 * @return RSS memory in GB, or -1 on failure
 */
double getMemoryUsage();

// Backward compatibility aliases
inline void var_init(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr,
                     double dept_err, double beam_err) {
  varInit(ext, pl_cur, pptr, dept_err, beam_err);
}

inline void pvec_update(PVecPtr pptr, IMUST& x_curr, PLV(3)& pwld) {
  pvecUpdate(pptr, x_curr, pwld);
}

inline double get_memory() { return getMemoryUsage(); }

} // namespace core
} // namespace vina_slam

// Global-scope backward compatibility
using vina_slam::core::calcBodyVar;
using vina_slam::core::var_init;
using vina_slam::core::pvec_update;
using vina_slam::core::get_memory;
