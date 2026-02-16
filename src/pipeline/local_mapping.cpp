/**
 * @file local_mapping.cpp
 * @brief Implementation of local mapping pipeline
 */

#include "vina_slam/pipeline/local_mapping.hpp"
#include "vina_slam/core/math.hpp"
#include "vina_slam/mapping/voxel_map.hpp"
#include <thread>

namespace vina_slam {
namespace pipeline {

LocalMapping::LocalMapping() : lidar_factor(20), normal_factor(20) {}

void LocalMapping::initialize(int window_size) {
  win_size = window_size;
  lidar_factor = mapping::LidarFactor(window_size);
  normal_factor = mapping::NormalFactor(window_size);
  li_optimizer.win_size = window_size;
  li_optimizer_gravity.win_size = window_size;
}

void LocalMapping::multiRecut(int win_count, std::vector<core::IMUST>& x_buf,
                              std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                              std::vector<std::vector<mapping::SlideWindow*>>& sws) {
  // Multi-threaded recut
  int thd_num = sws.size();
  if (thd_num <= 0)
    return;

  std::vector<std::thread> threads;
  std::vector<std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>*> maps;
  std::vector<int> counts;

  // Split voxel map across threads
  int voxel_count = surf_map.size();
  int part = voxel_count / thd_num;

  auto iter = surf_map.begin();
  for (int i = 0; i < thd_num && iter != surf_map.end(); i++) {
    int start_idx = i * part;
    int end_idx = (i == thd_num - 1) ? voxel_count : (i + 1) * part;

    // Process voxels in this thread's range
    auto it = surf_map.begin();
    std::advance(it, start_idx);
    auto end_it = surf_map.begin();
    std::advance(end_it, end_idx);

    for (; it != end_it; ++it) {
      it->second->recut(win_count, x_buf, sws[i]);
    }
  }
}

void LocalMapping::multiMargi(int win_count, int mgsize, std::vector<core::IMUST>& x_buf,
                              std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map) {
  // Marginalize old frames from voxel map
  for (auto& kv : surf_map) {
    kv.second->margi(win_count, mgsize, x_buf, lidar_factor);
  }
}

void LocalMapping::collectFactors(std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                                  std::vector<core::IMUST>& /*x_buf*/,
                                  std::vector<std::vector<mapping::SlideWindow*>>& /*sws*/) {
  lidar_factor.clear();
  normal_factor.clear();

  for (auto& kv : surf_map) {
    kv.second->trasOpt(lidar_factor);
  }
}

void LocalMapping::runOptimization(std::vector<core::IMUST>& x_buf,
                                   std::deque<estimation::ImuPreintegration*>& imu_pre_buf,
                                   Eigen::MatrixXd* hess, bool with_gravity) {
  if (with_gravity) {
    li_optimizer_gravity.jac_leng = win_size * DIM;
    li_optimizer_gravity.imu_leng = (win_size - 1) * DIM;

    std::vector<double> resis;
    li_optimizer_gravity.dampingIter(x_buf, lidar_factor, imu_pre_buf, resis, hess, 2);
  } else {
    li_optimizer.jac_leng = win_size * DIM;
    li_optimizer.imu_leng = (win_size - 1) * DIM;

    li_optimizer.dampingIter(x_buf, lidar_factor, imu_pre_buf, hess);
  }
}

void LocalMapping::clearFactors() {
  lidar_factor.clear();
  normal_factor.clear();
}

} // namespace pipeline
} // namespace vina_slam
