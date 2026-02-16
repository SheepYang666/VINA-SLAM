/**
 * @file voxel_map.cpp
 * @brief Implementation of voxel map utilities
 */

#include "vina_slam/mapping/voxel_map.hpp"
#include "vina_slam/core/math.hpp"
#include <thread>

namespace vina_slam {
namespace mapping {

namespace {
// Generate a stable voxel ID from center coordinates (for visualization markers)
[[maybe_unused]] static int voxel_id_from_center(const double center[3], int layer) {
  const int64_t ix = llround(center[0] * 1000.0);
  const int64_t iy = llround(center[1] * 1000.0);
  const int64_t iz = llround(center[2] * 1000.0);
  int64_t h = ix * 73856093LL ^ iy * 19349663LL ^ iz * 83492791LL ^ (int64_t(layer) * 2654435761LL);
  if (h < 0)
    h = -h;
  return static_cast<int>(h & 0x7fffffff);
}
} // namespace

void cutVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
              core::PVecPtr pvec, int win_count,
              std::unordered_map<core::VOXEL_LOC, OctoTree*>& /*feat_tem_map*/,
              int wdsize, PLV(3)& pwld, std::vector<SlideWindow*>& sws) {
  double loc_xyz[3];
  for (core::pointVar& pv : *pvec) {
    Eigen::Vector3d pw(pv.pnt[0], pv.pnt[1], pv.pnt[2]);
    pwld.push_back(pw);

    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pw[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->voxel_center[0] = (0.5 + loc_xyz[0]) * voxel_size;
      ot->voxel_center[1] = (0.5 + loc_xyz[1]) * voxel_size;
      ot->voxel_center[2] = (0.5 + loc_xyz[2]) * voxel_size;
      ot->quater_length = voxel_size / 2;
      ot->push(win_count, pv, pw, sws);
      feat_map[position] = ot;
    } else {
      iter->second->push(win_count, pv, pw, sws);
    }
  }
}

void cutVoxelMulti(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::PVecPtr pvec, int win_count,
                   std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_tem_map,
                   int wdsize, PLV(3)& pwld,
                   std::vector<std::vector<SlideWindow*>>& sws) {
  // Simple single-threaded implementation for now
  // TODO: implement proper multi-threading if needed
  cutVoxel(feat_map, pvec, win_count, feat_tem_map, wdsize, pwld, sws[0]);
}

void cutVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
              core::PVec& pvec, int wdsize, double jour) {
  double loc_xyz[3];
  for (core::pointVar& pv : pvec) {
    Eigen::Vector3d pw(pv.pnt[0], pv.pnt[1], pv.pnt[2]);

    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pw[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->voxel_center[0] = (0.5 + loc_xyz[0]) * voxel_size;
      ot->voxel_center[1] = (0.5 + loc_xyz[1]) * voxel_size;
      ot->voxel_center[2] = (0.5 + loc_xyz[2]) * voxel_size;
      ot->quater_length = voxel_size / 2;
      ot->jour = jour;
      std::vector<SlideWindow*> sws;
      ot->pushFix(pv);
      feat_map[position] = ot;
    } else {
      iter->second->pushFix(pv);
    }
  }
}

void generateVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::PVec& pvec, double vox_size) {
  double loc_xyz[3];
  for (core::pointVar& pv : pvec) {
    Eigen::Vector3d pw(pv.pnt[0], pv.pnt[1], pv.pnt[2]);

    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pw[j] / vox_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      OctoTree* ot = new OctoTree(0, 1);
      ot->voxel_center[0] = (0.5 + loc_xyz[0]) * vox_size;
      ot->voxel_center[1] = (0.5 + loc_xyz[1]) * vox_size;
      ot->voxel_center[2] = (0.5 + loc_xyz[2]) * vox_size;
      ot->quater_length = vox_size / 2;
      ot->pushFix(pv);
      feat_map[position] = ot;
    } else {
      iter->second->pushFix(pv);
    }
  }
}

void generateVoxel(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                   core::IMUST& x_curr, core::PVec& pvec, double vox_size) {
  double loc_xyz[3];
  for (core::pointVar& pv : pvec) {
    Eigen::Vector3d p_body(pv.pnt[0], pv.pnt[1], pv.pnt[2]);
    Eigen::Vector3d pw = x_curr.R * p_body + x_curr.p;

    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pw[j] / vox_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      OctoTree* ot = new OctoTree(0, 1);
      ot->voxel_center[0] = (0.5 + loc_xyz[0]) * vox_size;
      ot->voxel_center[1] = (0.5 + loc_xyz[1]) * vox_size;
      ot->voxel_center[2] = (0.5 + loc_xyz[2]) * vox_size;
      ot->quater_length = vox_size / 2;
      ot->pushFix(pv);
      feat_map[position] = ot;
    } else {
      iter->second->pushFix(pv);
    }
  }
}

int matchVoxelMap(std::unordered_map<core::VOXEL_LOC, OctoTree*>& feat_map,
                  Eigen::Vector3d& wld, core::Plane*& pla,
                  Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc) {
  double loc_xyz[3];
  double vx = voxel_size;

  for (int j = 0; j < 3; j++) {
    loc_xyz[j] = wld[j] / vx;
    if (loc_xyz[j] < 0)
      loc_xyz[j] -= 1.0;
  }

  core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

  // Check nearby voxels
  int min_range = -1;
  int max_range = 1;
  double max_prob = 0;
  oc = nullptr;
  pla = nullptr;

  for (int ix = min_range; ix <= max_range; ix++) {
    for (int iy = min_range; iy <= max_range; iy++) {
      for (int iz = min_range; iz <= max_range; iz++) {
        core::VOXEL_LOC loc(position.x + ix, position.y + iy, position.z + iz);

        auto iter = feat_map.find(loc);
        if (iter != feat_map.end()) {
          OctoTree* octo = iter->second;
          core::Plane* plane_temp = nullptr;
          double prob_temp = 0;
          OctoTree* oc_temp = nullptr;

          if (octo->match(wld, plane_temp, prob_temp, var_wld, sigma_d, oc_temp) > 0) {
            if (prob_temp > max_prob) {
              max_prob = prob_temp;
              pla = plane_temp;
              oc = oc_temp;
            }
          }
        }
      }
    }
  }

  return (max_prob > 0) ? 1 : 0;
}

void downSamplingPvec(core::PVec& pvec, double vox_size,
                      pcl::PointCloud<core::PointType>& pl_keep) {
  std::unordered_map<core::VOXEL_LOC, core::pointVar> vox_map;

  double loc_xyz[3];
  for (core::pointVar& pv : pvec) {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pv.pnt[j] / vox_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    core::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = vox_map.find(position);

    if (iter == vox_map.end()) {
      vox_map[position] = pv;
    } else {
      // Average points in the same voxel
      iter->second.pnt = (iter->second.pnt + pv.pnt) / 2.0;
    }
  }

  pl_keep.clear();
  pl_keep.reserve(vox_map.size());
  for (auto& kv : vox_map) {
    core::PointType pt;
    pt.x = kv.second.pnt.x();
    pt.y = kv.second.pnt.y();
    pt.z = kv.second.pnt.z();
    pt.intensity = kv.second.intensity;
    pl_keep.push_back(pt);
  }
}

} // namespace mapping
} // namespace vina_slam
