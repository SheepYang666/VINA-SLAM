// Odometry methods of VINA_SLAM class
// Moved from VINASlam.cpp: lio_state_estimation(), VNC_lio(), lio_state_estimation_kdtree()

#include "vina_slam/platform/ros2/node.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/mapping/voxel_map.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/logging.hpp>

namespace
{
struct ScanPlaneInfo
{
  Eigen::Vector3d center_body;
  Eigen::Vector3d normal_body;
  double quality;   // eigenvalue ratio: ev1 / ev0
  double sigma_n;   // sqrt of smallest eigenvalue
};

void collectScanPlanes(OctoTree* node, std::vector<ScanPlaneInfo>& out)
{
  if (node == nullptr)
    return;

  if (node->octo_state == 0)
  {
    if (node->plane.is_plane && node->eig_value[1] > 1e-12 && node->eig_value[0] / node->eig_value[1] <= 0.12)
    {
      double lambda_min = node->eig_value[0];
      double lambda_mid = node->eig_value[1];
      double lambda_max = node->eig_value[2];
      double lambda_sum = lambda_min + lambda_mid + lambda_max + 1e-10;
      double quality = 1.0 - lambda_min / lambda_sum;

      if (quality > 0.5)
      {
        Eigen::Vector3d normal = node->plane.normal;
        double norm = normal.norm();
        if (norm >= 1e-12)
        {
          ScanPlaneInfo sp;
          sp.center_body = node->plane.center;
          sp.normal_body = normal / norm;
          sp.quality = quality;
          sp.sigma_n = std::sqrt(std::max(0.0, lambda_min / lambda_sum));
          out.push_back(sp);
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      if (node->leaves[i] != nullptr)
        collectScanPlanes(node->leaves[i], out);
    }
  }
}
}  // namespace

bool VINA_SLAM::LioStateEstimation(PVecPtr pptr, bool use_vnc)
{
  IMUST x_prop = x_curr;

  const int num_max_iter = use_vnc ? 4 : 20;
  bool EKF_stop_flg = false;
  bool flg_EKF_converged = false;
  Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();
  int rematch_num = 0;
  int match_num = 0;

  int psize = pptr->size();
  vector<OctoTree*> octos(psize, nullptr);

  Eigen::Matrix3d nnt;
  Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

  // VNC preprocessing: build scan voxels and extract scan planes
  unordered_map<VOXEL_LOC, OctoTree*> scan_voxels;
  std::vector<ScanPlaneInfo> scan_planes;
  if (use_vnc)
  {
    PVec pvec_body = *pptr;
    generate_voxel(scan_voxels, pvec_body, voxel_size);
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (auto& kv : scan_voxels)
      kv.second->fitScanPlane(origin);
    for (auto& kv : scan_voxels)
      collectScanPlanes(kv.second, scan_planes);
  }

  for (int iterCount = 0; iterCount < num_max_iter; iterCount++)
  {
    Eigen::Matrix<double, 6, 6> HTH;
    HTH.setZero();
    Eigen::Matrix<double, 6, 1> HTz;
    HTz.setZero();

    Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
    Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

    match_num = 0;
    nnt.setZero();

    for (int i = 0; i < psize; i++)
    {
      pointVar& pv = pptr->at(i);
      Eigen::Matrix3d phat = hat(pv.pnt);

      Eigen::Matrix3d var_world =
          x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
      Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

      double sigma_d = 0;
      Plane* pla = nullptr;
      int flag = 0;

      if (octos[i] != nullptr && octos[i]->inside(wld))
      {
        double max_prob = 0;
        flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
      }
      else
      {
        flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
      }

      if (flag)
      {
        Plane& pp = *pla;
        double R_inv = 1.0 / (0.0005 + sigma_d);
        double resi = pp.normal.dot(wld - pp.center);

        Eigen::Matrix<double, 6, 1> jac;
        jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
        jac.tail(3) = pp.normal;
        HTH += R_inv * jac * jac.transpose();
        HTz -= R_inv * jac * resi;
        nnt += pp.normal * pp.normal.transpose();
        match_num++;
      }
    }

    // VNC residuals: normal consistency between scan planes and map planes
    if (use_vnc)
    {
      static const Eigen::Matrix3d var_dummy = Eigen::Matrix3d::Identity() * 0.01;
      for (const ScanPlaneInfo& sp : scan_planes)
      {
        Eigen::Vector3d center_world = x_curr.R * sp.center_body + x_curr.p;
        Eigen::Vector3d n_scan_world = (x_curr.R * sp.normal_body).normalized();

        Plane* map_plane = nullptr;
        double sigma_vnc = 0;
        OctoTree* oc_temp = nullptr;
        Eigen::Matrix3d var_tmp = var_dummy;
        int found = matchVoxelMap(surf_map, center_world, map_plane, var_tmp, sigma_vnc, oc_temp);
        if (!found || map_plane == nullptr)
          continue;

        Eigen::Vector3d n_map = map_plane->normal.normalized();
        if (n_map.squaredNorm() < 1e-12)
          continue;

        double dot = std::abs(n_scan_world.dot(n_map));
        if (dot < 0.7)
          continue;

        Eigen::Matrix3d S = Eigen::Matrix3d::Identity() - n_map * n_map.transpose();
        Eigen::Vector3d r = S * n_scan_world;

        // Right-perturbation Jacobian: J_rot = -S * R * [n_body]x, J_pos = 0
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = -S * x_curr.R * hat(sp.normal_body);
        J.block<3, 3>(0, 3).setZero();

        double w = 0.1 * sp.quality / (sp.sigma_n * sp.sigma_n + 0.01);
        if (!std::isfinite(w))
          continue;

        HTH += w * J.transpose() * J;
        HTz -= w * J.transpose() * r;
      }
    }

    H_T_H.block<6, 6>(0, 0) = HTH;

    Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();

    G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;

    Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;

    Eigen::Matrix<double, DIM, 1> solution =
        K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

    x_curr += solution;

    Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    EKF_stop_flg = false;
    flg_EKF_converged = false;

    if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
    {
      flg_EKF_converged = true;
    }

    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == num_max_iter - 2)))
    {
      rematch_num++;
    }

    if (rematch_num >= 2 || (iterCount == num_max_iter - 1))
    {
      x_curr.cov = (I_STATE - G) * x_curr.cov;
      EKF_stop_flg = true;
    }

    if (EKF_stop_flg)
    {
      break;
    }
  }

  // Cleanup VNC scan voxels
  if (use_vnc)
  {
    for (auto& kv : scan_voxels)
    {
      kv.second->delete_ptr();
      delete kv.second;
    }
    scan_voxels.clear();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  Eigen::Vector3d evalue = saes.eigenvalues();

  if (evalue[0] < 14)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool VINA_SLAM::lio_state_estimation(PVecPtr pptr)
{
  return LioStateEstimation(pptr, false);
}

bool VINA_SLAM::VNC_lio(PVecPtr pptr)
{
  return LioStateEstimation(pptr, true);
}

void VINA_SLAM::lio_state_estimation_kdtree(PVecPtr pptr)
{
  static pcl::KdTreeFLANN<PointType> kd_map;

  if (!pptr || pptr->empty())
  {
    RCLCPP_WARN(node->get_logger(), "Empty point cloud, skipping kdtree estimation");
    return;
  }

  if (pl_tree->size() < 100)
  {
    for (pointVar pv : *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType pp;
      pp.x = pv.pnt[0];
      pp.y = pv.pnt[1];
      pp.z = pv.pnt[2];
      pl_tree->push_back(pp);
    }

    if (pl_tree->empty())
    {
      RCLCPP_WARN(node->get_logger(),
                  "[KdTree] pl_tree is empty after point insertion");
      return;
    }

    try
    {
      kd_map.setInputCloud(pl_tree);
      RCLCPP_DEBUG(node->get_logger(), "[KdTree] setInputCloud success, size: %zu", pl_tree->size());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "[KdTree] Exception: %s", e.what());
    }

    return;
  }

  const int num_max_iter = 4;
  IMUST x_prop = x_curr;
  int psize = pptr->size();
  bool EKF_stop_flg = false;
  bool flg_EKF_converged = false;
  Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  double max_dis = 4.0;
  vector<float> sqdis(NMATCH);
  vector<int> nearInd(NMATCH);
  PLV(3) vecs(NMATCH);
  int rematch_num = 0;
  Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

  Eigen::Matrix<double, NMATCH, 1> b;
  b.setOnes();
  b *= -1.0f;

  vector<double> ds(psize, -1);
  PLV(3) directs(psize);
  bool refind = true;

  for (int iterCount = 0; iterCount < num_max_iter; iterCount++)
  {
    Eigen::Matrix<double, 6, 6> HTH;
    HTH.setZero();
    Eigen::Matrix<double, 6, 1> HTz;
    HTz.setZero();
    int valid = 0;

    for (int i = 0; i < psize; i++)
    {
      pointVar& pv = pptr->at(i);
      Eigen::Matrix3d phat = hat(pv.pnt);
      Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

      if (refind)
      {
        PointType apx;
        apx.x = wld[0];
        apx.y = wld[1];
        apx.z = wld[2];
        kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

        Eigen::Matrix<double, NMATCH, 3> A;
        for (int i = 0; i < NMATCH; i++)
        {
          PointType& pp = pl_tree->points[nearInd[i]];
          A.row(i) << pp.x, pp.y, pp.z;
        }
        Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);

        bool check_flag = false;
        for (int i = 0; i < NMATCH; i++)
        {
          if (fabs(direct.dot(A.row(i)) + 1.0) > 0.1)
            check_flag = true;
        }
        if (check_flag)
        {
          ds[i] = -1;
          continue;
        }

        double d = 1.0 / direct.norm();
        ds[i] = d;
        directs[i] = direct * d;
      }

      if (ds[i] >= 0)
      {
        double pd2 = directs[i].dot(wld) + ds[i];

        Eigen::Matrix<double, 6, 1> jac_s;
        jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
        jac_s.tail(3) = directs[i];

        HTH += jac_s * jac_s.transpose();
        HTz += jac_s * (-pd2);
        valid++;
      }
    }

    H_T_H.block<6, 6>(0, 0) = HTH;
    Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
    G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
    Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
    Eigen::Matrix<double, DIM, 1> solution =
        K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

    x_curr += solution;
    Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    refind = false;
    if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
    {
      refind = true;
      flg_EKF_converged = true;
      rematch_num++;
    }
    if (iterCount == num_max_iter - 2 && !flg_EKF_converged)
    {
      refind = true;
    }
    if (rematch_num >= 2 || (iterCount == num_max_iter - 1))
    {
      x_curr.cov = (I_STATE - G) * x_curr.cov;
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg)
    {
      break;
    }
  }

  for (pointVar pv : *pptr)
  {
    pv.pnt = x_curr.R * pv.pnt + x_curr.p;
    PointType ap;
    ap.x = pv.pnt[0];
    ap.y = pv.pnt[1];
    ap.z = pv.pnt[2];
    pl_tree->push_back(ap);
  }
  down_sampling_voxel(*pl_tree, 0.5);
  kd_map.setInputCloud(pl_tree);
}
