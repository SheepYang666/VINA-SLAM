/**
 * @file voxel_map.hpp
 * @brief Voxel hash map and point cloud processing utilities
 *
 * Provides voxel-based spatial hashing and point cloud operations
 * for efficient nearest neighbor queries and map management.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/octree.hpp"
#include <unordered_map>
#include <pcl/point_cloud.h>

namespace vina_slam {
namespace mapping {

/**
 * @brief Insert points into voxel map with sliding window
 * @param feat_map Output voxel hash map
 * @param pvec Input points with variance
 * @param win_count Current frame index in window
 * @param feat_tem_map Temporary voxel map (for optimization)
 * @param wdsize Sliding window size
 * @param pwld Output world points
 * @param sws Sliding window pool
 */
void cutVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
              core::PVecPtr pvec, int win_count,
              std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_tem_map,
              int wdsize, PLV(3)& pwld, std::vector<SlideWindow*>& sws);

/**
 * @brief Multi-threaded voxel insertion
 * @param feat_map Output voxel hash map
 * @param pvec Input points with variance
 * @param win_count Current frame index in window
 * @param feat_tem_map Temporary voxel map
 * @param wdsize Sliding window size
 * @param pwld Output world points
 * @param sws Multi-threaded sliding window pools
 */
void cutVoxelMulti(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::PVecPtr pvec, int win_count,
                   std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_tem_map,
                   int wdsize, PLV(3)& pwld,
                   std::vector<std::vector<SlideWindow*>>& sws);

/**
 * @brief Insert points into voxel map (simple version)
 * @param feat_map Output voxel hash map
 * @param pvec Input points
 * @param wdsize Sliding window size
 * @param jour Journey distance
 */
void cutVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
              core::PVec& pvec, int wdsize, double jour);

/**
 * @brief Generate voxel map from points
 * @param feat_map Output voxel hash map
 * @param pvec Input points
 * @param voxel_size Voxel edge length
 */
void generateVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::PVec& pvec, double voxel_size);

/**
 * @brief Generate voxel map with pose transformation
 * @param feat_map Output voxel hash map
 * @param x_curr Current pose
 * @param pvec Input points (in body frame)
 * @param voxel_size Voxel edge length
 */
void generateVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::IMUST& x_curr, core::PVec& pvec, double voxel_size);

/**
 * @brief Match point against planes in voxel map
 * @param feat_map Voxel hash map
 * @param wld World point to match
 * @param pla Output matched plane
 * @param var_wld Point variance in world frame
 * @param sigma_d Output distance sigma
 * @param oc Output matching OctoTree node
 * @return Match quality (0 = no match)
 */
int matchVoxelMap(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                  Eigen::Vector3d& wld, core::Plane*& pla,
                  Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc);

/**
 * @brief Downsample points using voxel grid
 * @param pvec Input/output points
 * @param voxel_size Voxel edge length
 * @param pl_keep Output downsampled point cloud
 */
void downSamplingPvec(core::PVec& pvec, double voxel_size,
                      pcl::PointCloud<core::PointType>& pl_keep);

// ========================================================================
// Backward compatibility functions (snake_case aliases)
// ========================================================================
inline void cut_voxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                      core::PVecPtr pvec, int win_count,
                      std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_tem_map,
                      int wdsize, PLV(3)& pwld, std::vector<SlideWindow*>& sws) {
  cutVoxel(feat_map, pvec, win_count, feat_tem_map, wdsize, pwld, sws);
}

inline void cut_voxel_multi(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                            core::PVecPtr pvec, int win_count,
                            std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_tem_map,
                            int wdsize, PLV(3)& pwld,
                            std::vector<std::vector<SlideWindow*>>& sws) {
  cutVoxelMulti(feat_map, pvec, win_count, feat_tem_map, wdsize, pwld, sws);
}

inline void cut_voxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                      core::PVec& pvec, int wdsize, double jour) {
  cutVoxel(feat_map, pvec, wdsize, jour);
}

inline void generate_voxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                           core::PVec& pvec, double voxel_size) {
  generateVoxel(feat_map, pvec, voxel_size);
}

inline void generate_voxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                           core::IMUST& x_curr, core::PVec& pvec, double voxel_size) {
  generateVoxel(feat_map, x_curr, pvec, voxel_size);
}

inline int match(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                 Eigen::Vector3d& wld, core::Plane*& pla,
                 Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc) {
  return matchVoxelMap(feat_map, wld, pla, var_wld, sigma_d, oc);
}

inline void down_sampling_pvec(core::PVec& pvec, double voxel_size,
                               pcl::PointCloud<core::PointType>& pl_keep) {
  downSamplingPvec(pvec, voxel_size, pl_keep);
}

} // namespace mapping
} // namespace vina_slam

// Backward compatibility - bring functions into global namespace
using vina_slam::mapping::cutVoxel;
using vina_slam::mapping::cutVoxelMulti;
using vina_slam::mapping::generateVoxel;
using vina_slam::mapping::matchVoxelMap;
using vina_slam::mapping::downSamplingPvec;
using vina_slam::mapping::cut_voxel;
using vina_slam::mapping::cut_voxel_multi;
using vina_slam::mapping::generate_voxel;
using vina_slam::mapping::match;
using vina_slam::mapping::down_sampling_pvec;
