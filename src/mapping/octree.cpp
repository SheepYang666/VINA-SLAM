/**
 * @file octree.cpp
 * @brief Implementation of OctoTree and related utilities
 */

#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/core/math.hpp"
#include <Eigen/Eigenvalues>

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
  bcov.setZero();

  // Covariance contribution from point variance
  Eigen::Matrix3d pvar = pv.var;

  // Second moment contribution
  bcov.block<3, 3>(0, 0) = pvar;
  bcov.block<3, 3>(6, 6) = pvar;

  // Cross terms
  bcov.block<3, 3>(0, 6) = vec * pvar.col(0).transpose();
  bcov.block<3, 3>(6, 0) = pvar.col(0) * vec.transpose();
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
      printf("Error: opt_state: %d %zu\n", opt_state, vox_opt.pcr_adds.size());
      exit(0);
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

} // namespace mapping
} // namespace vina_slam
