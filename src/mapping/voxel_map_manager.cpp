/**
 * @file voxel_map_manager.cpp
 * @brief Implementation of voxel map management
 */

#include "vina_slam/mapping/voxel_map_manager.hpp"
#include <algorithm>
#include <malloc.h>

namespace vina_slam {
namespace mapping {

VoxelMapManager::VoxelMapManager(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {
}

VoxelMapManager::~VoxelMapManager() {
  // Delete all OctoTrees in the release queue
  for (auto* octo : octos_release) {
    delete octo;
  }
  octos_release.clear();

  // Delete all OctoTrees in main map
  for (auto& kv : surf_map) {
    delete kv.second;
  }
  surf_map.clear();

  // Delete all OctoTrees in sliding map
  for (auto& kv : surf_map_slide) {
    delete kv.second;
  }
  surf_map_slide.clear();
}

VoxelMapManager& VoxelMapManager::instance(const rclcpp::Node::SharedPtr& node_in) {
  static VoxelMapManager inst(node_in);
  return inst;
}

VoxelMapManager& VoxelMapManager::instance() {
  return instance(rclcpp::Node::SharedPtr());
}

void VoxelMapManager::releaseOldVoxels(double jour) {
  std::vector<OctoTree*> octos;
  for (auto iter = surf_map.begin(); iter != surf_map.end();) {
    double dis = jour - iter->second->jour;
    if (dis < kVoxelReleaseDistThresh) {
      ++iter;
      continue;
    }

    octos.push_back(iter->second);
    iter->second->tras_ptr(octos);
    iter = surf_map.erase(iter);
  }

  for (auto* octo : octos) {
    delete octo;
  }
  malloc_trim(0);
}

void VoxelMapManager::cleanupReleasedOctos() {
  if (octos_release.empty())
    return;

  int msize = std::min(static_cast<int>(octos_release.size()), kMaxOctosBatchDelete);
  for (int i = 0; i < msize; i++) {
    delete octos_release.back();
    octos_release.pop_back();
  }
  malloc_trim(0);
}

void VoxelMapManager::addToReleaseQueue(OctoTree* octo) {
  octos_release.push_back(octo);
}

void VoxelMapManager::addToReleaseQueue(std::vector<OctoTree*>& octos) {
  octos_release.insert(octos_release.end(), octos.begin(), octos.end());
}

void VoxelMapManager::clearMainMap() {
  for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++) {
    iter->second->tras_ptr(octos_release);
    delete iter->second;
  }
  surf_map.clear();
}

void VoxelMapManager::clearSlidingMap() {
  for (auto iter = surf_map_slide.begin(); iter != surf_map_slide.end(); iter++) {
    iter->second->tras_ptr(octos_release);
    delete iter->second;
  }
  surf_map_slide.clear();
}

void VoxelMapManager::clearAllMaps() {
  clearMainMap();
  clearSlidingMap();
}

} // namespace mapping
} // namespace vina_slam
