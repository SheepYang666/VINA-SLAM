/**
 * @file octree.cpp
 * @brief Implementation of OctoTree and related utilities
 */

#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/core/math.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>

namespace vina_slam {
namespace mapping {

// Global configuration parameters
Eigen::Vector4d min_point;
double min_eigen_value = 0.01;
int max_layer = 2;
int max_points = 100;
double voxel_size = 1.0;
int min_ba_point = 20;
std::vector<double> plane_eigen_value_thre;
std::vector<int> mp;

void bfVar(const core::pointVar& pv, Eigen::Matrix<double, 9, 9>& bcov, const Eigen::Vector3d& vec) {
  Eigen::Matrix<double, 6, 3> Bi;
  Bi << 2 * vec(0), 0, 0,
        vec(1), vec(0), 0,
        vec(2), 0, vec(0),
        0, 2 * vec(1), 0,
        0, vec(2), vec(1),
        0, 0, 2 * vec(2);
  Eigen::Matrix<double, 6, 3> Biup = Bi * pv.var;
  bcov.block<6, 6>(0, 0) = Biup * Bi.transpose();
  bcov.block<6, 3>(0, 6) = Biup;
  bcov.block<3, 6>(6, 0) = Biup.transpose();
  bcov.block<3, 3>(6, 6) = pv.var;
}

OctoTree::OctoTree(int _l, int _w) : layer(_l), octo_state(0), wdsize(_w) {
  for (int i = 0; i < 8; i++)
    leaves[i] = nullptr;
  cov_add.setZero();
}

void OctoTree::push(int ord, const core::pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws) {
  mVox.lock();
  if (sw == nullptr) {
    if (sws.size() != 0) {
      sw = sws.back();
      sws.pop_back();
      sw->resize(wdsize);
    } else {
      sw = new SlideWindow(wdsize);
    }
  }
  if (!isexist)
    isexist = true;

  int mord = mp[ord];
  if (layer < max_layer)
    sw->points[mord].push_back(pv);
  sw->pcrs_local[mord].push(pv.pnt);
  pcr_add.push(pw);
  Eigen::Matrix<double, 9, 9> Bi;
  bfVar(pv, Bi, pw);
  cov_add += Bi;
  mVox.unlock();
}

void OctoTree::pushFix(core::pointVar& pv) {
  if (layer < max_layer)
    point_fix.push_back(pv);
  pcr_fix.push(pv.pnt);
  pcr_add.push(pv.pnt);
  Eigen::Matrix<double, 9, 9> Bi;
  bfVar(pv, Bi, pv.pnt);
  cov_add += Bi;
}

void OctoTree::pushFixNovar(core::pointVar& pv) {
  if (layer < max_layer)
    point_fix.push_back(pv);
  pcr_fix.push(pv.pnt);
  pcr_add.push(pv.pnt);
}

bool OctoTree::planeJudge(Eigen::Vector3d& eig_values) {
  return (eig_values[0] < min_eigen_value && (eig_values[0] / eig_values[2]) < plane_eigen_value_thre[layer]);
}

void OctoTree::allocate(int ord, const core::pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws) {
  if (octo_state == 0) {
    push(ord, pv, pw, sws);
  } else {
    int xyz[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++)
      if (pw[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (leaves[leafnum] == nullptr) {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->allocate(ord, pv, pw, sws);
  }
}

void OctoTree::allocateFix(core::pointVar& pv) {
  if (octo_state == 0) {
    pushFixNovar(pv);
  } else if (layer < max_layer) {
    int xyz[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++)
      if (pv.pnt[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (leaves[leafnum] == nullptr) {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->allocateFix(pv);
  }
}

void OctoTree::fixDivide(std::vector<SlideWindow*>& /*sws*/) {
  for (core::pointVar& pv : point_fix) {
    int xyz[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++)
      if (pv.pnt[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves[leafnum] == nullptr) {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->pushFix(pv);
  }
}

void OctoTree::subdivide(int si, core::IMUST& xx, std::vector<SlideWindow*>& sws) {
  for (core::pointVar& pv : sw->points[mp[si]]) {
    Eigen::Vector3d pw = xx.R * pv.pnt + xx.p;
    int xyz[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++)
      if (pw[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves[leafnum] == nullptr) {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->push(si, pv, pw, sws);
  }
}

void OctoTree::planeUpdate() {
  if (pcr_add.N == 0)
    return;

  plane.center = pcr_add.v / pcr_add.N;
  int l = 0;
  Eigen::Vector3d u[3] = {eig_vector.col(0), eig_vector.col(1), eig_vector.col(2)};
  double nv = 1.0 / pcr_add.N;

  Eigen::Matrix<double, 3, 9> u_c;
  u_c.setZero();
  for (int k = 0; k < 3; k++)
    if (k != l) {
      Eigen::Matrix3d ukl = u[k] * u[l].transpose();
      Eigen::Matrix<double, 1, 9> fkl;
      fkl.head(6) << ukl(0, 0), ukl(1, 0) + ukl(0, 1), ukl(2, 0) + ukl(0, 2), ukl(1, 1), ukl(1, 2) + ukl(2, 1),
          ukl(2, 2);
      fkl.tail(3) = -(u[k].dot(plane.center) * u[l] + u[l].dot(plane.center) * u[k]);

      u_c += nv / (eig_value[l] - eig_value[k]) * u[k] * fkl;
    }

  Eigen::Matrix<double, 3, 9> Jc = u_c * cov_add;
  plane.plane_var.block<3, 3>(0, 0) = Jc * u_c.transpose();
  Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
  plane.plane_var.block<3, 3>(0, 3) = Jc_N;
  plane.plane_var.block<3, 3>(3, 0) = Jc_N.transpose();
  plane.plane_var.block<3, 3>(3, 3) = nv * nv * cov_add.block<3, 3>(6, 6);
  plane.normal = u[0];
  plane.radius = eig_value[2];
  plane.is_update = true;
  plane.is_normal_update = true;
}

bool OctoTree::inside(Eigen::Vector3d& wld) {
  for (int k = 0; k < 3; k++) {
    if (fabs(wld[k] - voxel_center[k]) > quater_length + 1e-6)
      return false;
  }
  return true;
}

void OctoTree::deletePtr() {
  for (int i = 0; i < 8; i++) {
    if (leaves[i] != nullptr) {
      leaves[i]->deletePtr();
      delete leaves[i];
      leaves[i] = nullptr;
    }
  }

  if (sw != nullptr) {
    delete sw;
    sw = nullptr;
  }
}

void OctoTree::trasPtr(std::vector<OctoTree*>& octos_release) {
  for (int i = 0; i < 8; i++) {
    if (leaves[i] != nullptr) {
      leaves[i]->trasPtr(octos_release);
    }
  }
  octos_release.push_back(this);
}

void OctoTree::clearSlwd(std::vector<SlideWindow*>& sws) {
  for (int i = 0; i < 8; i++) {
    if (leaves[i] != nullptr) {
      leaves[i]->clearSlwd(sws);
    }
  }

  if (sw != nullptr) {
    sw->clear();
    sws.push_back(sw);
    sw = nullptr;
  }
}

int OctoTree::match(Eigen::Vector3d& wld, core::Plane*& pla, double& max_prob,
                    Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc) {
  if (!isexist)
    return 0;

  // Check if this is a valid plane
  if (plane.is_plane && plane.normal.squaredNorm() > 1e-12) {
    // Check if point is within voxel bounds
    if (!inside(wld))
      return 0;

    // Compute distance to plane
    double dist = std::abs(plane.normal.dot(wld - plane.center));

    // Simple probability based on distance and radius
    if (dist < plane.radius * 0.5) {
      max_prob = std::exp(-dist * dist / (plane.radius * plane.radius));
      pla = &plane;
      sigma_d = plane.radius;
      oc = this;
      return 1;
    }
  }

  // Search in children if subdivided
  if (octo_state != 0) {
    for (int i = 0; i < 8; i++) {
      if (leaves[i] != nullptr) {
        double child_prob = 0;
        core::Plane* child_pla = nullptr;
        OctoTree* child_oc = nullptr;
        double child_sigma = 0;

        if (leaves[i]->match(wld, child_pla, child_prob, var_wld, child_sigma, child_oc) > 0) {
          if (child_prob > max_prob) {
            max_prob = child_prob;
            pla = child_pla;
            oc = child_oc;
            sigma_d = child_sigma;
          }
        }
      }
    }
    return (max_prob > 0) ? 1 : 0;
  }

  return 0;
}

void OctoTree::recut(int win_count, std::vector<core::IMUST>& x_buf, std::vector<SlideWindow*>& sws) {
  if (octo_state == 0) {
    if (plane.is_plane && plane.normal.squaredNorm() > 1e-12) {
      plane.normal_prev = plane.normal;
    } else {
      plane.normal_prev.setZero();
    }
  }

  if (octo_state == 0) {
    if (layer >= 0) {
      opt_state = -1;
      if (pcr_add.N <= min_point[layer]) {
        plane.is_plane = false;
        return;
      }
      if (!isexist || sw == nullptr)
        return;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
      eig_value = saes.eigenvalues();
      eig_vector = saes.eigenvectors();
      plane.is_plane = planeJudge(eig_value);

      if (plane.is_plane) {
        return;
      } else if (layer >= max_layer)
        return;
    }

    if (pcr_fix.N != 0) {
      fixDivide(sws);
      PVec().swap(point_fix);
    }

    for (int i = 0; i < win_count; i++)
      subdivide(i, x_buf[i], sws);

    sw->clear();
    sws.push_back(sw);
    sw = nullptr;
    octo_state = 1;
  }

  for (int i = 0; i < 8; i++)
    if (leaves[i] != nullptr)
      leaves[i]->recut(win_count, x_buf, sws);
}

void OctoTree::margi(int win_count, int mgsize, std::vector<core::IMUST>& x_buf, const LidarFactor& vox_opt) {
  if (octo_state == 0 && layer >= 0) {
    if (!isexist || sw == nullptr)
      return;
    mVox.lock();
    std::vector<core::PointCluster> pcrs_world(wdsize);

    if (opt_state >= int(vox_opt.pcr_adds.size())) {
      printf("Error: opt_state out of range: %d >= %zu\n", opt_state, vox_opt.pcr_adds.size());
      opt_state = -1;
      mVox.unlock();
      return;
    }

    if (opt_state >= 0) {
      pcr_add = vox_opt.pcr_adds[opt_state];
      eig_value = vox_opt.eig_values[opt_state];
      eig_vector = vox_opt.eig_vectors[opt_state];
      opt_state = -1;

      for (int i = 0; i < mgsize; i++)
        if (sw->pcrs_local[mp[i]].N != 0) {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
        }
    } else {
      pcr_add = pcr_fix;
      for (int i = 0; i < win_count; i++)
        if (sw->pcrs_local[mp[i]].N != 0) {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
          pcr_add += pcrs_world[i];
        }

      if (plane.is_plane) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        eig_value = saes.eigenvalues();
        eig_vector = saes.eigenvectors();
      }
    }

    if (pcr_fix.N < max_points && plane.is_plane)
      if (pcr_add.N - last_num >= 5 || last_num <= 10) {
        planeUpdate();
        last_num = pcr_add.N;
      }

    if (pcr_fix.N < max_points) {
      for (int i = 0; i < mgsize; i++)
        if (pcrs_world[i].N != 0) {
          pcr_fix += pcrs_world[i];
          for (pointVar pv : sw->points[mp[i]]) {
            pv.pnt = x_buf[i].R * pv.pnt + x_buf[i].p;
            point_fix.push_back(pv);
          }
        }
    } else {
      for (int i = 0; i < mgsize; i++)
        if (pcrs_world[i].N != 0)
          pcr_add -= pcrs_world[i];

      if (point_fix.size() != 0)
        PVec().swap(point_fix);
    }

    for (int i = 0; i < mgsize; i++)
      if (sw->pcrs_local[mp[i]].N != 0) {
        sw->pcrs_local[mp[i]].clear();
        sw->points[mp[i]].clear();
      }

    if (pcr_fix.N >= pcr_add.N)
      isexist = false;
    else
      isexist = true;

    mVox.unlock();
  } else {
    isexist = false;
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr) {
        leaves[i]->margi(win_count, mgsize, x_buf, vox_opt);
        isexist = isexist || leaves[i]->isexist;
      }
  }
}

void OctoTree::trasOpt(LidarFactor& vox_opt) {
  if (octo_state == 0) {
    if (layer >= 0 && isexist && plane.is_plane && sw != nullptr) {
      if (eig_value[0] / eig_value[1] > 0.12)
        return;

      double coe = 1;
      std::vector<core::PointCluster> pcrs(wdsize);
      for (int i = 0; i < wdsize; i++)
        pcrs[i] = sw->pcrs_local[mp[i]];
      opt_state = vox_opt.plvec_voxels.size();
      vox_opt.pushVoxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);
    }
  } else {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->trasOpt(vox_opt);
  }
}

void OctoTree::trasOpt(NormalFactor& vox_opt) {
  if (octo_state == 0) {
    if (layer >= 0 && isexist && plane.is_plane && sw != nullptr) {
      if (eig_value[0] / eig_value[1] > 0.12)
        return;

      double coe = 1;
      std::vector<core::PointCluster> pcrs(wdsize);
      for (int i = 0; i < wdsize; i++)
        pcrs[i] = sw->pcrs_local[mp[i]];
      opt_state = vox_opt.plvec_voxels.size();
      // NormalFactor uses plane normal as reference
      vox_opt.pushVoxel(pcrs, pcr_fix, coe, plane.normal, pcr_add);
    }
  } else {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->trasOpt(vox_opt);
  }
}

// ============================================================================
// Visualization helpers (anonymous namespace)
// ============================================================================
namespace {

[[maybe_unused]] static int voxel_id_from_center(const double center[3], int layer) {
  const int64_t ix = llround(center[0] * 1000.0);
  const int64_t iy = llround(center[1] * 1000.0);
  const int64_t iz = llround(center[2] * 1000.0);
  int64_t h = ix * 73856093LL ^ iy * 19349663LL ^ iz * 83492791LL ^ (int64_t(layer) * 2654435761LL);
  if (h < 0) h = -h;
  return static_cast<int>(h & 0x7fffffff);
}

void map_jet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;

  double dr = 1.0, dg = 1.0, db = 1.0;
  if (v < 0.1242) {
    db = 0.504 + ((1.0 - 0.504) / 0.1242) * v;
    dg = dr = 0.0;
  } else if (v < 0.3747) {
    db = 1.0; dr = 0.0;
    dg = (v - 0.1242) * (1.0 / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1.0 / (0.6253 - 0.3747));
    dg = 1.0;
    dr = (v - 0.3747) * (1.0 / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.0; dr = 1.0;
    dg = (0.8758 - v) * (1.0 / (0.8758 - 0.6253));
  } else {
    db = 0.0; dg = 0.0;
    dr = 1.0 - (v - 0.8758) * ((1.0 - 0.504) / (1.0 - 0.8758));
  }

  r = static_cast<uint8_t>(255 * dr);
  g = static_cast<uint8_t>(255 * dg);
  b = static_cast<uint8_t>(255 * db);
}

} // anonymous namespace

// ============================================================================
// Additional OctoTree methods ported from legacy voxel_map.cpp
// ============================================================================

bool OctoTree::fitScanPlane(const Eigen::Vector3d& sensor_pos) {
  plane.is_plane = false;
  isexist = (pcr_add.N != 0);

  if (octo_state == 1) {
    bool has_plane = false;
    for (int i = 0; i < 8; i++) {
      if (leaves[i] != nullptr) {
        has_plane = leaves[i]->fitScanPlane(sensor_pos) || has_plane;
      }
    }
    isexist = has_plane;
    return has_plane;
  }

  if (pcr_add.N < 3) {
    return false;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
  eig_value = saes.eigenvalues();
  eig_vector = saes.eigenvectors();
  plane.is_plane = planeJudge(eig_value);

  if (plane.is_plane) {
    plane.center = pcr_add.v / pcr_add.N;
    plane.normal = eig_vector.col(0);
    plane.radius = eig_value[2];

    if (plane.normal.dot(plane.center - sensor_pos) > 0) {
      plane.normal = -plane.normal;
    }

    return true;
  }

  int min_points_subdivide = 3;
  if (layer >= 0 && layer < static_cast<int>(min_point.size())) {
    min_points_subdivide = std::max(3, static_cast<int>(min_point[layer]));
  }

  if (layer >= max_layer || pcr_add.N < min_points_subdivide || point_fix.empty()) {
    return false;
  }

  octo_state = 1;
  std::vector<SlideWindow*> scan_sws;
  fixDivide(scan_sws);
  core::PVec().swap(point_fix);

  bool has_plane = false;
  for (int i = 0; i < 8; i++) {
    if (leaves[i] != nullptr) {
      has_plane = leaves[i]->fitScanPlane(sensor_pos) || has_plane;
    }
  }

  isexist = has_plane;
  return has_plane;
}

OctoTree* OctoTree::findFinePlane(Eigen::Vector3d& wld) {
  if (!inside(wld)) {
    return nullptr;
  }

  if (octo_state == 1) {
    for (int i = 0; i < 8; i++) {
      if (leaves[i] != nullptr && leaves[i]->inside(wld)) {
        OctoTree* finer = leaves[i]->findFinePlane(wld);
        if (finer != nullptr) {
          return finer;
        }
      }
    }
  }

  if (plane.is_plane) {
    return this;
  }

  return nullptr;
}

void OctoTree::trasDisplay(int win_count, pcl::PointCloud<core::PointType>& pl_fixd,
                           pcl::PointCloud<core::PointType>& pl_wind, std::vector<core::IMUST>& x_buf) {
  if (octo_state == 0) {
    if (sw == nullptr) return;
    const int sw_size = static_cast<int>(sw->points.size());
    if (sw_size == 0) return;
    const int cnt = std::min(win_count, sw_size);
    (void)pcr_add;
    (void)pl_fixd;

    core::PointType ap;

    if (plane.is_plane) {
      for (int i = 0; i < cnt; i++) {
        const int idx = mp[i];
        if (idx < 0 || idx >= sw_size) continue;
        if (i >= static_cast<int>(x_buf.size())) break;
        for (core::pointVar& pv : sw->points[idx]) {
          Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;
          ap.x = pvec[0];
          ap.y = pvec[1];
          ap.z = pvec[2];
          pl_wind.push_back(ap);
        }
      }
    }
  } else {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->trasDisplay(win_count, pl_fixd, pl_wind, x_buf);
  }
}

void OctoTree::collectPlaneMarkers(visualization_msgs::msg::MarkerArray& out, int max_layer_param,
                                   std::unordered_set<int>& used_ids, float alpha,
                                   double max_trace, double pow_num) {
  if (layer > max_layer_param) return;

  if (octo_state == 0) {
    if (!plane.is_plane && plane.is_published) {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (used_ids.insert(id).second) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.ns = "plane";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        out.markers.push_back(marker);
      }
      plane.is_published = false;
      plane.is_update = false;
    }

    if (plane.is_plane && plane.is_update) {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (!used_ids.insert(id).second) return;
      const Eigen::Vector3d cov_diag = plane.plane_var.block<3, 3>(0, 0).diagonal();
      double trace = cov_diag.sum();
      if (trace >= max_trace) trace = max_trace;
      trace = trace * (1.0 / max_trace);
      trace = std::pow(trace, pow_num);

      uint8_t r = 255, g = 255, b = 255;
      map_jet(trace, 0.0, 1.0, r, g, b);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "plane";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = plane.center[0];
      marker.pose.position.y = plane.center[1];
      marker.pose.position.z = plane.center[2];

      const Eigen::Vector3d n = plane.normal.normalized();
      const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n);
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      const double ev0 = std::max(0.0, eig_value[0]);
      const double ev1 = std::max(0.0, eig_value[1]);
      const double ev2 = std::max(0.0, eig_value[2]);
      marker.scale.x = 3.0 * std::sqrt(ev2);
      marker.scale.y = 3.0 * std::sqrt(ev1);
      marker.scale.z = 2.0 * std::sqrt(ev0);

      marker.color.a = alpha;
      marker.color.r = r / 255.0f;
      marker.color.g = g / 255.0f;
      marker.color.b = b / 255.0f;

      out.markers.push_back(marker);
      plane.is_update = false;
      plane.is_published = true;
    }
    return;
  } else if (plane.is_published) {
    const int id = voxel_id_from_center(voxel_center, layer);
    if (used_ids.insert(id).second) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "plane";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(marker);
    }
    plane.is_published = false;
    plane.is_update = false;
  }

  for (int i = 0; i < 8; ++i)
    if (leaves[i] != nullptr)
      leaves[i]->collectPlaneMarkers(out, max_layer_param, used_ids, alpha, max_trace, pow_num);
}

void OctoTree::collectNormalMarkers(visualization_msgs::msg::MarkerArray& out, int max_layer_param,
                                    std::unordered_set<int>& used_ids, float alpha,
                                    double max_trace, double pow_num) {
  if (layer > max_layer_param) return;

  if (octo_state == 0) {
    if (!plane.is_plane && plane.is_normal_published) {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (used_ids.insert(id).second) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.ns = "normal";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        out.markers.push_back(marker);
      }
      plane.is_normal_published = false;
      plane.is_normal_update = false;
    }

    if (plane.is_plane && plane.is_normal_update) {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (!used_ids.insert(id).second) return;

      const Eigen::Vector3d cov_diag = plane.plane_var.block<3, 3>(0, 0).diagonal();
      double trace = cov_diag.sum();
      if (trace >= max_trace) trace = max_trace;
      trace = trace * (1.0 / max_trace);
      trace = std::pow(trace, pow_num);

      uint8_t r = 255, g = 255, b = 255;
      map_jet(trace, 0.0, 1.0, r, g, b);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "normal";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      const double length = 2.0 * quater_length;
      geometry_msgs::msg::Point p0;
      p0.x = plane.center[0];
      p0.y = plane.center[1];
      p0.z = plane.center[2];

      const Eigen::Vector3d n = plane.normal.normalized();
      geometry_msgs::msg::Point p1;
      p1.x = plane.center[0] + n[0] * length;
      p1.y = plane.center[1] + n[1] * length;
      p1.z = plane.center[2] + n[2] * length;

      marker.points.push_back(p0);
      marker.points.push_back(p1);

      marker.scale.x = 0.1 * length;
      marker.scale.y = 0.2 * length;
      marker.scale.z = 0.0;

      marker.color.a = alpha;
      marker.color.r = r / 255.0f;
      marker.color.g = g / 255.0f;
      marker.color.b = b / 255.0f;

      out.markers.push_back(marker);
      plane.is_normal_update = false;
      plane.is_normal_published = true;
    }
    return;
  } else if (plane.is_normal_published) {
    const int id = voxel_id_from_center(voxel_center, layer);
    if (used_ids.insert(id).second) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "normal";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(marker);
    }
    plane.is_normal_published = false;
    plane.is_normal_update = false;
  }

  for (int i = 0; i < 8; ++i)
    if (leaves[i] != nullptr)
      leaves[i]->collectNormalMarkers(out, max_layer_param, used_ids, alpha, max_trace, pow_num);
}

} // namespace mapping
} // namespace vina_slam
