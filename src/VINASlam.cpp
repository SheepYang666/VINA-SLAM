#include "vina_slam/VINASlam.hpp"
#include "vina_slam/voxel_map.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <ios>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sstream>

VINA_SLAM& VINA_SLAM::Instance(const rclcpp::Node::SharedPtr& node_in)
{
  static VINA_SLAM inst(node_in);
  return inst;
}

VINA_SLAM& VINA_SLAM::Instance()
{
  rclcpp::Node::SharedPtr node_temp;
  return instance(node_temp);
}

VINA_SLAM::VINA_SLAM(const rclcpp::Node::SharedPtr& node_in) : node(node_in)
{
  double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
  vector<double> vecR(9), vecT(3);
  // scanPoses removed - not needed for localization mode
  keyframes = new vector<Keyframe*>();

  bagname = node->declare_parameter("General.bagname", "noNameBag");
  node->get_parameter("General.bagname", bagname);

  savepath = node->declare_parameter("General.save_path", "");
  node->get_parameter("General.save_path", savepath);

  lid_topic = node->declare_parameter("General.lid_topic", "/rslidar_points");
  node->get_parameter("General.lid_topic", lid_topic);

  imu_topic = node->declare_parameter("General.imu_topic", "/imu");
  node->get_parameter("General.imu_topic", imu_topic);

  feat.lidar_type = node->declare_parameter("General.lidar_type", 0);
  node->get_parameter("General.lidar_type", feat.lidar_type);

  feat.blind = node->declare_parameter("General.blind", 0.1);
  node->get_parameter("General.blind", feat.blind);

  feat.point_filter_num = node->declare_parameter("General.point_filter_num", 3);
  node->get_parameter("General.point_filter_num", feat.point_filter_num);

  vecT = node->declare_parameter("General.extrinsic_tran", std::vector<double>(3, 0.0));
  node->get_parameter("General.extrinsic_tran", vecT);

  vecR = node->declare_parameter("General.extrinsic_rota", std::vector<double>(9, 0.0));
  node->get_parameter("General.extrinsic_rota", vecR);

  is_save_map = node->declare_parameter<int>("General.is_save_map", false);
  node->get_parameter("General.is_save_map", is_save_map);

  if_loop_dect = node->declare_parameter<int>("General.if_loop_dect", false);
  node->get_parameter("General.if_loop_dect", if_loop_dect);

  if_BA = node->declare_parameter<int>("General.if_BA", false);
  node->get_parameter("General.if_BA", if_BA);

  // Log configuration
  RCLCPP_INFO(node->get_logger(), "is_save_map: %s", is_save_map ? "ON" : "OFF");
  RCLCPP_INFO(node->get_logger(), "if_loop_dect: %s", if_loop_dect ? "ON" : "OFF");
  RCLCPP_INFO(node->get_logger(), "if_BA: %s", if_BA ? "ON" : "OFF");

  // 订阅器初始化

  rclcpp::QoS imu_qos(8000);
  imu_qos.keep_last(8000);
  imu_qos.best_effort();

  rclcpp::QoS pcl_qos(1000);
  pcl_qos.keep_last(1000);
  pcl_qos.best_effort();  // 通常用于激光雷达/点云等高频数据

  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_handler(msg); });

  if (feat.lidar_type == LIVOX)
  {
    // sub_pcl = node->create_subscription<livox_ros_driver::msg::CustomMsg>(
    //     lid_topic, rclcpp::SensorDataQoS(),
    //     [](const livox_ros_driver::msg::CustomMsg::SharedPtr msg) { pcl_handler(msg); });

    sub_pcl = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { pcl_handler(msg); });
  }
  else
  {
    // sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     lid_topic, pcl_qos, [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcl_handler(msg); });
  }
  odom_ekf.imu_topic = imu_topic;

  // Odometry param
  cov_gyr = node->declare_parameter<double>("Odometry.cov_gyr", 0.1);
  node->get_parameter("Odometry.cov_gyr", cov_gyr);

  cov_acc = node->declare_parameter<double>("Odometry.cov_acc", 0.1);
  node->get_parameter("Odometry.cov_acc", cov_acc);

  rand_walk_gyr = node->declare_parameter<double>("Odometry.rdw_gyr", 1e-4);
  node->get_parameter("Odometry.rdw_gyr", rand_walk_gyr);

  rand_walk_acc = node->declare_parameter<double>("Odometry.rdw_acc", 1e-4);
  node->get_parameter("Odometry.rdw_acc", rand_walk_acc);

  down_size = node->declare_parameter<double>("Odometry.down_size", 0.1);
  node->get_parameter("Odometry.down_size", down_size);

  dept_err = node->declare_parameter<double>("Odometry.dept_err", 0.02);
  node->get_parameter("Odometry.dept_err", dept_err);

  beam_err = node->declare_parameter<double>("Odometry.beam_err", 0.05);
  node->get_parameter("Odometry.beam_err", beam_err);

  voxel_size = node->declare_parameter<double>("Odometry.voxel_size", 1.0);
  node->get_parameter("Odometry.voxel_size", voxel_size);

  min_eigen_value = node->declare_parameter<double>("Odometry.min_eigen_value", 0.0025);
  node->get_parameter("Odometry.min_eigen_value", min_eigen_value);

  degrade_bound = node->declare_parameter<int>("Odometry.degrade_bound", 100);
  node->get_parameter("Odometry.degrade_bound", degrade_bound);

  point_notime = node->declare_parameter<int>("Odometry.point_notime", 0);
  node->get_parameter("Odometry.point_notime", point_notime);

  odom_ekf.point_notime = point_notime;

  // Other parameters and initialization
  feat.blind = feat.blind * feat.blind;
  odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
  odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
  odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
  odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
  odom_ekf.Lid_offset_to_IMU << vecT[0], vecT[1], vecT[2];
  odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2], vecR[3], vecR[4], vecR[5], vecR[6], vecR[7], vecR[8];
  extrin_para.R = odom_ekf.Lid_rot_to_IMU;
  extrin_para.p = odom_ekf.Lid_offset_to_IMU;
  min_point << 20, 20, 15, 10;

  // LocalBA param
  win_size = node->declare_parameter<int>("LocalBA.win_size", 10);
  node->get_parameter("LocalBA.win_size", win_size);

  max_layer = node->declare_parameter<int>("LocalBA.max_layer", 2);
  node->get_parameter("LocalBA.max_layer", max_layer);

  cov_gyr = node->declare_parameter<double>("LocalBA.cov_gyr", 0.1);
  node->get_parameter("LocalBA.cov_gyr", cov_gyr);

  cov_acc = node->declare_parameter<double>("LocalBA.cov_acc", 0.1);
  node->get_parameter("LocalBA.cov_acc", cov_acc);

  rand_walk_gyr = node->declare_parameter<double>("LocalBA.rdw_gyr", 1e-4);
  node->get_parameter("LocalBA.rdw_gyr", rand_walk_gyr);

  rand_walk_acc = node->declare_parameter<double>("LocalBA.rdw_acc", 1e-4);
  node->get_parameter("LocalBA.rdw_acc", rand_walk_acc);

  min_ba_point = node->declare_parameter<int>("LocalBA.min_ba_point", 20);
  node->get_parameter("LocalBA.min_ba_point", min_ba_point);

  plane_eigen_value_thre = node->declare_parameter<std::vector<double>>("LocalBA.plane_eigen_value_thre",
                                                                        std::vector<double>({ 1, 1, 1, 1 }));
  node->get_parameter("LocalBA.plane_eigen_value_thre", plane_eigen_value_thre);

  imu_coef = node->declare_parameter<double>("LocalBA.imu_coef", 1e-4);
  node->get_parameter("LocalBA.imu_coef", imu_coef);

  thread_num = node->declare_parameter<int>("LocalBA.thread_num", 5);
  node->get_parameter("LocalBA.thread_num", thread_num);

  is_finish = node->declare_parameter<bool>("finish", false);
  node->get_parameter("finish", is_finish);

  for (double& iter : plane_eigen_value_thre)
  {
    iter = 1.0 / iter;
  }

  // Noise matrix initialization
  noiseMeas.setZero();
  noiseWalk.setZero();
  noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, cov_acc, cov_acc, cov_acc;
  noiseWalk.diagonal() << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, rand_walk_acc, rand_walk_acc, rand_walk_acc;

  if (is_save_map == 1)
  {
    std::filesystem::path save_dir = std::filesystem::path(savepath) / bagname;
    if (std::filesystem::exists(save_dir))
    {
      RCLCPP_WARN(node->get_logger(), "Save directory already exists: %s", save_dir.c_str());
    }
    else
    {
      std::error_code ec;
      std::filesystem::create_directories(save_dir, ec);
      if (ec)
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to create save directory: %s (%s)", save_dir.c_str(),
                     ec.message().c_str());
      }
    }
  }

  sws.resize(thread_num);
  RCLCPP_INFO(node->get_logger(), "VINA_SLAM initialized: bag=%s, win=%d, voxel=%.2f, BA=%d", bagname.c_str(), win_size,
              voxel_size, if_BA);
}

// ============================================================================
// VNC Helper: Scan plane extraction for normal consistency constraints
// ============================================================================
namespace
{

template <typename Derived>
std::string matrixToString(const Eigen::MatrixBase<Derived>& value)
{
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

struct ScanPlaneInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center_body;       ///< Plane center in body frame
  Eigen::Vector3d normal_body;       ///< Plane normal in body frame (unit vector)
  double quality;                    ///< Plane quality in (0,1], higher = flatter
  double sigma_n;                    ///< Normal estimation uncertainty
};

void collectScanPlanes(OctoTree* ot, std::vector<ScanPlaneInfo>& planes)
{
  if (ot == nullptr)
    return;

  if (ot->octo_state == 0)
  {
    if (ot->plane.is_plane && ot->eig_value[1] > 1e-12 && ot->eig_value[0] / ot->eig_value[1] <= 0.12)
    {
      double lambda_min = ot->eig_value[0];
      double lambda_mid = ot->eig_value[1];
      double lambda_max = ot->eig_value[2];
      double lambda_sum = lambda_min + lambda_mid + lambda_max + 1e-10;
      double quality = 1.0 - lambda_min / lambda_sum;

      if (quality > 0.5)
      {
        Eigen::Vector3d normal = ot->plane.normal;
        double norm = normal.norm();
        if (norm >= 1e-12)
        {
          ScanPlaneInfo sp;
          sp.center_body = ot->plane.center;
          sp.normal_body = normal / norm;
          sp.quality = quality;
          sp.sigma_n = std::sqrt(std::max(0.0, lambda_min / lambda_sum));  // clamp for eigensolver float noise
          planes.push_back(sp);
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < 8; ++i)
      collectScanPlanes(ot->leaves[i], planes);
  }
}

}  // anonymous namespace

// ============================================================================
// Unified LIO State Estimation (point-to-plane + optional VNC normal constraint)
// ============================================================================
bool VINA_SLAM::LioStateEstimation(PVecPtr pptr, bool use_vnc)
{
  if (pptr == nullptr || pptr->empty())
  {
    RCLCPP_WARN(node->get_logger(), "LioStateEstimation: empty point vector, use_vnc=%d", use_vnc);
    return false;
  }

  IMUST x_prop = x_curr;
  bool numeric_failure = false;
  std::string numeric_failure_context;

  auto stateFinite = [](const IMUST& state) {
    return state.R.allFinite() && state.p.allFinite() && state.v.allFinite() && state.bg.allFinite() &&
           state.ba.allFinite() && state.g.allFinite() && state.cov.allFinite();
  };

  auto markNumericFailure = [&](const std::string& stage, const std::string& details) {
    if (!numeric_failure)
    {
      numeric_failure = true;
      std::ostringstream oss;
      oss << "LioStateEstimation numeric failure at " << stage << ", use_vnc=" << use_vnc << ", details: " << details;
      numeric_failure_context = oss.str();
      RCLCPP_ERROR(node->get_logger(), "%s", numeric_failure_context.c_str());
    }
  };

  const int num_max_iter = 4;
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

  if (!cov_inv.allFinite())
  {
    RCLCPP_WARN(node->get_logger(), "LioStateEstimation: cov inverse NaN/Inf, resetting");
    x_curr.cov = Eigen::Matrix<double, DIM, DIM>::Identity() * 1e-3;
    cov_inv = x_curr.cov.inverse();
  }

  // ======================================================================
  // VNC pre-processing: extract scan planes in body frame (only if enabled)
  // ======================================================================
  const double VNC_ALPHA = 0.1;

  unordered_map<VOXEL_LOC, OctoTree*> scan_voxels;
  std::vector<ScanPlaneInfo> scan_planes;
  int num_scan_planes = 0;

  if (use_vnc)
  {
    PVec pvec_body = *pptr;
    generate_voxel(scan_voxels, pvec_body, voxel_size);

    Eigen::Vector3d sensor_origin_body = Eigen::Vector3d::Zero();
    for (auto& kv : scan_voxels)
      kv.second->fit_scan_plane(sensor_origin_body);

    scan_planes.reserve(scan_voxels.size());
    for (auto& kv : scan_voxels)
      collectScanPlanes(kv.second, scan_planes);
    num_scan_planes = static_cast<int>(scan_planes.size());

    RCLCPP_INFO(node->get_logger(), "VNCLio preprocessing: points=%d scan_voxels=%zu scan_planes=%d", psize,
                scan_voxels.size(), num_scan_planes);
  }

  // ======================================================================
  // IEKF iterative update
  // ======================================================================
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

    // -- Point-to-plane residuals --
    for (int i = 0; i < psize; i++)
    {
      pointVar& pv = pptr->at(i);
      if (!pv.pnt.allFinite() || !pv.var.allFinite())
      {
        markNumericFailure("point_residual_input", "point_idx=" + std::to_string(i) + ", pnt=[" +
                                                       matrixToString(pv.pnt.transpose()) + "], var=\n" +
                                                       matrixToString(pv.var));
        break;
      }
      Eigen::Matrix3d phat = hat(pv.pnt);

      Eigen::Matrix3d var_world =
          x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
      Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;
      if (!var_world.allFinite() || !wld.allFinite())
      {
        markNumericFailure("point_residual_projection",
                           "point_idx=" + std::to_string(i) + ", pnt=[" + matrixToString(pv.pnt.transpose()) +
                               "], x_curr.p=[" + matrixToString(x_curr.p.transpose()) + "], wld=[" +
                               matrixToString(wld.transpose()) + "], var_world=\n" + matrixToString(var_world));
        break;
      }

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

    if (numeric_failure)
      break;

    // -- VNC normal consistency residuals (only if enabled) --
    if (use_vnc)
    {
      static const Eigen::Matrix3d var_dummy = Eigen::Matrix3d::Identity() * 0.01;

      for (int sp_idx = 0; sp_idx < num_scan_planes; sp_idx++)
      {
        const ScanPlaneInfo& sp = scan_planes[sp_idx];

        // 1. Transform scan plane to world frame using current IEKF estimate
        Eigen::Vector3d center_world = x_curr.R * sp.center_body + x_curr.p;
        Eigen::Vector3d n_scan_world = (x_curr.R * sp.normal_body).normalized();
        if (!center_world.allFinite() || !n_scan_world.allFinite())
        {
          markNumericFailure("vnc_projection", "iter=" + std::to_string(iterCount) +
                                                   ", sp=" + std::to_string(sp_idx));
          break;
        }

        // 2. Find nearest map plane via 27-neighbor search (replaces single-voxel findFinePlane)
        Plane* map_plane = nullptr;
        double sigma_d = 0;
        OctoTree* oc_temp = nullptr;
        Eigen::Matrix3d var_tmp = var_dummy;
        int found = matchVoxelMap(surf_map, center_world, map_plane, var_tmp, sigma_d, oc_temp);
        if (!found || map_plane == nullptr) continue;

        Eigen::Vector3d n_map = map_plane->normal.normalized();
        if (!n_map.allFinite() || n_map.squaredNorm() < 1e-12) continue;

        // 3. Normal consistency check: reject if angle > 45 degrees
        double dot = std::abs(n_scan_world.dot(n_map));
        if (dot < 0.7) continue;

        // 4-5-6. 3D VNC residual, Jacobian, and normal equation accumulation
        //
        // Residual derivation:
        //   n_scan_world = R * n_body  (scan normal in world frame)
        //   For a perfect match: n_scan_world ∥ n_map, i.e. n_scan_world × n_map = 0
        //   Equivalently: the component of n_scan_world perpendicular to n_map should be zero.
        //   Projection matrix S = I - n_map * n_map^T projects onto n_map's null space.
        //   Residual: r = S * n_scan_world  (3×1, zero when normals are parallel)
        //
        // Jacobian w.r.t. rotation error δθ (SO3 right perturbation, consistent with IMUST::operator+=):
        //   n_scan_world(δθ) = R * Exp(δθ) * n_body ≈ R * (I + [δθ]×) * n_body
        //                    = n_scan_world + R * ([δθ]× * n_body)
        //                    = n_scan_world - R * [n_body]× * δθ
        //   ∂r/∂δθ = -S * R * [n_body]×
        //   ∂r/∂δp = 0  (normals are translation-invariant)
        //
        Eigen::Matrix3d S = Eigen::Matrix3d::Identity() - n_map * n_map.transpose();
        Eigen::Vector3d r = S * n_scan_world;

        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = -S * x_curr.R * hat(sp.normal_body);
        J.block<3, 3>(0, 3).setZero();

        double w = VNC_ALPHA * sp.quality / (sp.sigma_n * sp.sigma_n + 0.01);
        if (!std::isfinite(w)) continue;

        HTH += w * J.transpose() * J;
        HTz -= w * J.transpose() * r;
      }
    }

    if (numeric_failure)
      break;

    if (!HTH.allFinite() || !HTz.allFinite())
    {
      markNumericFailure("normal_equation_terms", "iter=" + std::to_string(iterCount) + ", HTH=\n" +
                                                      matrixToString(HTH) + ", HTz=[" +
                                                      matrixToString(HTz.transpose()) + "]");
      break;
    }

    // -- IEKF state update --
    H_T_H.block<6, 6>(0, 0) = HTH;
    Eigen::Matrix<double, DIM, DIM> system_mat = H_T_H + cov_inv;
    if (!system_mat.allFinite())
    {
      markNumericFailure("system_matrix", "iter=" + std::to_string(iterCount) + ", HTH_6x6=\n" +
                                              matrixToString(H_T_H.block<6, 6>(0, 0)) + ", cov_inv_diag=[" +
                                              matrixToString(cov_inv.diagonal().transpose()) + "]");
      break;
    }

    Eigen::Matrix<double, DIM, DIM> K_1 = system_mat.inverse();
    if (!K_1.allFinite())
    {
      markNumericFailure("kalman_gain", "iter=" + std::to_string(iterCount) + ", system_mat_diag=[" +
                                            matrixToString(system_mat.diagonal().transpose()) + "]");
      break;
    }
    G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;

    Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
    Eigen::Matrix<double, DIM, 1> solution =
        K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);
    if (!solution.allFinite())
    {
      markNumericFailure("state_increment", "iter=" + std::to_string(iterCount) + ", HTz=[" +
                                                matrixToString(HTz.transpose()) + "], vec=[" +
                                                matrixToString(vec.transpose()) + "], solution=[" +
                                                matrixToString(solution.transpose()) + "]");
      break;
    }

    IMUST x_next = x_curr;
    x_next += solution;
    if (!stateFinite(x_next))
    {
      markNumericFailure("propagated_state", "iter=" + std::to_string(iterCount) + ", solution=[" +
                                                 matrixToString(solution.transpose()) + "], x_next.p=[" +
                                                 matrixToString(x_next.p.transpose()) + "], x_next.v=[" +
                                                 matrixToString(x_next.v.transpose()) + "], x_next.bg=[" +
                                                 matrixToString(x_next.bg.transpose()) + "], x_next.ba=[" +
                                                 matrixToString(x_next.ba.transpose()) + "]");
      break;
    }
    x_curr = x_next;

    Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    EKF_stop_flg = false;
    flg_EKF_converged = false;

    if ((rot_add.norm() * kRadToDeg < kRotConvergeThreshDeg) && (tra_add.norm() * kMeterToCm < kTraConvergeThreshCm))
      flg_EKF_converged = true;

    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == num_max_iter - 2)))
      rematch_num++;

    if (rematch_num >= 2 || (iterCount == num_max_iter - 1))
    {
      x_curr.cov = (I_STATE - G) * x_curr.cov;
      if (!x_curr.cov.allFinite())
      {
        markNumericFailure("state_covariance", "iter=" + std::to_string(iterCount) + ", G_6x6=\n" +
                                                   matrixToString(G.block<6, 6>(0, 0)) + ", cov_diag=[" +
                                                   matrixToString(x_curr.cov.diagonal().transpose()) + "]");
        break;
      }
      EKF_stop_flg = true;
    }

    if (EKF_stop_flg)
      break;
  }

  // ======================================================================
  // Cleanup temporary scan voxels (VNC only)
  // ======================================================================
  if (use_vnc)
  {
    for (auto& kv : scan_voxels)
    {
      kv.second->delete_ptr();
      delete kv.second;
    }
    scan_voxels.clear();
  }

  if (numeric_failure || !stateFinite(x_curr))
  {
    if (!numeric_failure_context.empty())
      RCLCPP_WARN(node->get_logger(), "LioStateEstimation: restoring previous state after failure");
    x_curr = x_prop;
    return false;
  }

  // ======================================================================
  // Degeneracy check
  // ======================================================================
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  Eigen::Vector3d evalue = saes.eigenvalues();
  return (evalue[0] >= kPlaneEigenvalueThresh);
}

// Backward compatibility: VNCLio calls unified method with VNC enabled
bool VINA_SLAM::VNCLio(PVecPtr pptr)
{
  return LioStateEstimation(pptr, true);
}

// VNC_lio removed - was duplicate of lio_state_estimation (dead code, saved ~125 lines)

void VINA_SLAM::LioStateEstimationKdtree(PVecPtr pptr)
{
  static pcl::KdTreeFLANN<PointType> kd_map;

  // 添加安全检查 - 防止空点云错误
  if (!pptr || pptr->empty())
  {
    RCLCPP_WARN(node->get_logger(), "Empty point cloud, skipping kdtree estimation");
    return;
  }

  if (pl_tree->size() < 100)
  {
    for (pointVar pv : *pptr)
    {
      // 将点变换到世界坐标系
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType pp;
      pp.x = pv.pnt[0];
      pp.y = pv.pnt[1];
      pp.z = pv.pnt[2];
      pl_tree->push_back(pp);
    }

    // 安全检查：确保点云不为空
    if (pl_tree->empty())
    {
      RCLCPP_WARN(node->get_logger(),
                  "[KdTree安全检查] 文件: vina_slam.cpp, 行: %d, 函数: lio_state_estimation_kdtree, 原因: "
                  "pl_tree为空",
                  __LINE__);
      return;
    }

    try
    {
      kd_map.setInputCloud(pl_tree);
      RCLCPP_DEBUG(node->get_logger(), "[KdTree成功] 文件: vina_slam.cpp, 行: %d, 点云大小: %zu", __LINE__,
                   pl_tree->size());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "[KdTree异常] 文件: vina_slam.cpp, 行: %d, 错误: %s", __LINE__, e.what());
    }

    return;
  }

  // 初始化迭代次数、变量及协方差矩阵
  const int num_max_iter = 4;
  IMUST x_prop = x_curr;  // 预测状态备份
  int psize = pptr->size();
  bool EKF_stop_flg = false;
  bool flg_EKF_converged = false;
  Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  vector<float> sqdis(NMATCH);  // 最近邻距离平方缓存
  vector<int> nearInd(NMATCH);  // 最近邻索引缓存
  PLV(3) vecs(NMATCH);          // 最近邻点坐标缓存
  int rematch_num = 0;
  Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

  // Check for NaN in covariance inverse (indicates numerical instability)
  if (!cov_inv.allFinite())
  {
    RCLCPP_WARN(node->get_logger(),
                "LioStateEstimationKdtree: covariance inverse contains NaN/Inf, resetting covariance");
    x_curr.cov = Eigen::Matrix<double, DIM, DIM>::Identity() * 1e-3;
    cov_inv = x_curr.cov.inverse();
  }

  Eigen::Matrix<double, NMATCH, 1> b;
  b.setOnes();
  b *= -1.0f;

  vector<double> ds(psize, -1);  // 点到拟合平面的距离缓存，初始化为-1无效
  PLV(3) directs(psize);         // 点对应的平面法线方向缓存
  bool refind = true;            // 是否重新计算平面法线标志

  //  迭代优化过程
  for (int iterCount = 0; iterCount < num_max_iter; iterCount++)
  {
    Eigen::Matrix<double, 6, 6> HTH;
    HTH.setZero();
    Eigen::Matrix<double, 6, 1> HTz;
    HTz.setZero();
    int valid = 0;

    // 遍历所有点，计算残差和雅可比矩阵
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
        kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);  // 搜索最近邻

        // 构造最近邻点矩阵用于拟合平面参数
        Eigen::Matrix<double, NMATCH, 3> A;
        for (int i = 0; i < NMATCH; i++)
        {
          PointType& pp = pl_tree->points[nearInd[i]];
          A.row(i) << pp.x, pp.y, pp.z;
        }
        Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);  // 最小二乘拟合平面法线

        // 检查拟合平面是否合理
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
        directs[i] = direct * d;  // 归一化平面法线
      }

      // 只有平面拟合有效的点才参与优化
      if (ds[i] >= 0)
      {
        double pd2 = directs[i].dot(wld) + ds[i];  // 点到平面距离

        Eigen::Matrix<double, 6, 1> jac_s;
        jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];  // 旋转雅可比
        jac_s.tail(3) = directs[i];                                // 平移雅可比

        HTH += jac_s * jac_s.transpose();  // 累积 Hessian
        HTz += jac_s * (-pd2);             // 累积残差向量
        valid++;
      }
    }

    // 计算卡尔曼增益和状态更新
    H_T_H.block<6, 6>(0, 0) = HTH;
    Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
    G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
    Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
    Eigen::Matrix<double, DIM, 1> solution =
        K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

    x_curr += solution;  // 更新状态
    Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    refind = false;
    if ((rot_add.norm() * kRadToDeg < kRotConvergeThreshDeg) && (tra_add.norm() * kMeterToCm < kTraConvergeThreshCm))
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

  // 优化完成，将点云转换到世界坐标系，并更新pl_tree
  for (pointVar pv : *pptr)
  {
    pv.pnt = x_curr.R * pv.pnt + x_curr.p;
    PointType ap;
    ap.x = pv.pnt[0];
    ap.y = pv.pnt[1];
    ap.z = pv.pnt[2];
    pl_tree->push_back(ap);
  }
  down_sampling_voxel(*pl_tree, kVoxelDownsampleSize);  // Voxel downsample for faster search
  kd_map.setInputCloud(pl_tree);                        // 重建Kd树
}

// loop_update removed - not needed for localization mode

// load the previous keyframe in the local voxel map

void VINA_SLAM::LoadKeyframes(double jour)
{
  if (history_kfsize <= 0)
  {
    return;
  }

  PointType ap_curr;
  ap_curr.x = x_curr.p[0];
  ap_curr.y = x_curr.p[1];
  ap_curr.z = x_curr.p[2];

  // 检查KdTree是否已初始化且有效
  if (pl_kdmap->empty())
  {
    RCLCPP_WARN(node->get_logger(),
                "[KdTree安全检查] 文件: vina_slam.cpp, 行: %d, 函数: keyframe_loading, 原因: pl_kdmap为空", __LINE__);
    return;
  }

  // 搜索关键帧Kd树中半径为10米的邻近点索引和距离
  vector<int> vec_idx;
  vector<float> vec_dis;
  try
  {
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);
    RCLCPP_DEBUG(node->get_logger(), "[KdTree成功] 文件: vina_slam.cpp, 行: %d, 搜索到 %zu 个关键帧", __LINE__,
                 vec_idx.size());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "[KdTree异常] 文件: vina_slam.cpp, 行: %d, 错误: %s", __LINE__, e.what());
    return;
  }

  // 遍历找到的关键帧索引
  for (int id : vec_idx)
  {
    // 仅处理存在的关键帧
    if (keyframes->at(id)->exist)
    {
      Keyframe& kf = *(keyframes->at(id));
      IMUST& xx = kf.x0;
      PVec pvec;
      pvec.reserve(kf.plptr->size());

      pointVar pv;
      pv.var.setZero();
      pv.intensity = 0.0f;  // Initialize to avoid -Wmaybe-uninitialized
      int plsize = kf.plptr->size();

      // 将关键帧点云所有点转换到世界坐标系
      for (int j = 0; j < plsize; j++)
      {
        PointType ap = kf.plptr->points[j];
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = xx.R * pv.pnt + xx.p;
        pvec.push_back(pv);
      }

      // 将转换后的点云插入当前体素地图
      cut_voxel(surf_map, pvec, win_size, jour);
      kf.exist = 0;  // 标记关键帧已处理，减少历史关键帧计数
      history_kfsize--;

      break;  // 只加载一个关键帧
    }
  }
}

int VINA_SLAM::Initialize(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, Eigen::MatrixXd& hess,
                          LidarFactor& voxhess, PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
{
  static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
  static vector<double> beg_times;
  static vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>> vec_imus;

  pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));

  if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
  {
    // std::cout << "[initialization] EKF process failed, waiting for more data." << std::endl;
    return 0;
  }

  if (win_count == 0)
  {
    imupre_scale_gravity = odom_ekf.scale_gravity;
  }

  PVecPtr pptr(new PVec);  // Create point cloud eigenvectors and prepare to initialize error covariance
  double downkd = down_size >= 0.5 ? down_size : 0.5;
  down_sampling_voxel(*pcl_curr, downkd);

  var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
  lio_state_estimation_kdtree(pptr);  // 基于KD树执行局部状态估计，优化当前状态

  //  更新输出点云位姿集合
  pwld.clear();
  pvec_update(pptr, x_curr, pwld);

  win_count++;  //  维护滑动窗口，缓存状态和点云
  x_buf.push_back(x_curr);
  pvec_buf.push_back(pptr);
  vina_slam::platform::ros2::ResultPublisher::instance().publishLocalTrajectory(pwld, 0, x_curr,
                                                                                sessionNames.size() - 1, pcl_path);

  if (win_count > 1)
  {
    // 构建IMU预积分缓冲
    imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
    imu_pre_buf[win_count - 2]->push_imu(imus);
  }

  pcl::PointCloud<PointType> pl_mid = *orig;
  down_sampling_close(*orig, down_size);
  if (orig->size() < 1000)
  {
    *orig = pl_mid;
    down_sampling_close(*orig, down_size / 2);
  }

  sort(orig->begin(), orig->end(), [](PointType& x, PointType& y) { return x.curvature < y.curvature; });

  // 历史点云与IMU数据缓存
  pl_origs.push_back(orig);
  beg_times.push_back(odom_ekf.pcl_beg_time);
  vec_imus.push_back(imus);

  int is_success = 0;

  if (win_count >= win_size)
  {
    is_success = vina_slam::pipeline::Initialization::instance().motionInit(
        pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr,
        imu_pre_buf, extrin_para);

    if (is_success == 0)
    {
      return -1;
    }
    else
    {
      return 1;
    }
  }

  return 0;
}

void VINA_SLAM::ResetSystem(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus)
{
  for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
  {
    iter->second->tras_ptr(octos_release);
    iter->second->clear_slwd(sws[0]);
    delete iter->second;
  }
  surf_map.clear();
  surf_map_slide.clear();

  x_curr.setZero();
  x_curr.p = Eigen::Vector3d(0, 0, 30);

  // 重置IMU相关状态并重新初始化
  odom_ekf.mean_acc.setZero();
  odom_ekf.init_num = 0;
  odom_ekf.IMU_init(imus);
  x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;

  // 释放IMU预积分缓存资源
  for (size_t i = 0; i < imu_pre_buf.size(); i++)
  {
    delete imu_pre_buf[i];
  }

  // 清空滑动窗口缓存
  x_buf.clear();
  pvec_buf.clear();
  imu_pre_buf.clear();
  pl_tree->clear();

  // 重置滑动窗口索引映射和计数
  for (int i = 0; i < win_size; i++)
  {
    mp[i] = i;
  }
  win_base = 0;
  win_count = 0;

  // 清空轨迹点云并发布空点云更新
  pcl_path.clear();
  vina_slam::platform::ros2::ResultPublisher::instance().clearCurrentPath();

  RCLCPP_WARN(node->get_logger(), "System reset triggered");
}

void VINA_SLAM::MultiMarginalize(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, double jour, int win_count,
                                 vector<IMUST>& xs, LidarFactor& voxopt, vector<SlideWindow*>& sw)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>*> octs;  // 分块存储体素节点指针，线程独占

  // 为每个线程创建体素节点存储容器
  for (int i = 0; i < thd_num; i++)
    octs.push_back(new vector<OctoTree*>());

  int g_size = feat_map.size();
  if (g_size < thd_num)  // 体素数量不足以分配给所有线程，直接返回
    return;

  double part = 1.0 * g_size / thd_num;  // 平均分配体素数量
  int cnt = 0;

  // 遍历体素地图，将体素均匀分配给各线程处理
  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    iter->second->jour = jour;
    octs[cnt]->push_back(iter->second);
    if (octs[cnt]->size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  // 定义线程执行函数，调用体素节点的边缘化方法
  auto margi_func = [](int win_cnt, vector<OctoTree*>* oct, vector<IMUST> xxs, LidarFactor& voxhess) {
    for (OctoTree* oc : *oct)
    {
      oc->margi(win_cnt, 1, xxs, voxhess);
    }
  };

  // Launch worker threads; main thread handles partition 0
  std::vector<std::thread> threads;
  threads.reserve(thd_num - 1);
  for (int i = 1; i < thd_num; i++)
    threads.emplace_back(margi_func, win_count, octs[i], xs, ref(voxopt));

  margi_func(win_count, octs[0], xs, voxopt);

  for (auto& t : threads)
    t.join();

  // 遍历体素地图，移除无效体素节点并清理滑动窗口数据
  for (auto iter = feat_map.begin(); iter != feat_map.end();)
  {
    if (iter->second->isexist)
    {
      iter++;
    }
    else
    {
      iter->second->clear_slwd(sw);
      feat_map.erase(iter++);
    }
  }

  for (int i = 0; i < thd_num; i++)
  {
    delete octs[i];
  }
}

// Common implementation for multi_recut - eliminates ~120 lines of duplicate code
void VINA_SLAM::MultiRecutImpl(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                               vector<vector<SlideWindow*>>& sws, function<void(OctoTree*)> post_process)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>> octss(thd_num);

  int g_size = feat_map.size();
  if (g_size < thd_num)
  {
    return;
  }

  double part = 1.0 * g_size / thd_num;
  int cnt = 0;

  // Distribute voxels evenly across threads
  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    octss[cnt].push_back(iter->second);
    if (octss[cnt].size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  // Thread worker function
  auto recut_func = [](int win_count, vector<OctoTree*>& oct, vector<IMUST> xxs, vector<SlideWindow*>& sw) {
    for (OctoTree* oc : oct)
    {
      oc->recut(win_count, xxs, sw);
    }
  };

  // Launch worker threads; main thread handles partition 0
  std::vector<std::thread> threads;
  threads.reserve(thd_num - 1);
  for (int i = 1; i < thd_num; i++)
    threads.emplace_back(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));

  recut_func(win_count, octss[0], xs, sws[0]);

  for (auto& t : threads)
    t.join();

  // Merge slide windows
  for (size_t i = 1; i < sws.size(); i++)
  {
    sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
    sws[i].clear();
  }

  // Apply post-processing if provided
  if (post_process)
  {
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
    {
      post_process(iter->second);
    }
  }
}

// Wrapper: with LidarFactor only
void VINA_SLAM::MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                           LidarFactor& voxopt, vector<vector<SlideWindow*>>& sws)
{
  MultiRecutImpl(feat_map, win_count, xs, sws, [&voxopt](OctoTree* oc) { oc->tras_opt(voxopt); });
}

// Wrapper: with LidarFactor and NormalFactor
void VINA_SLAM::MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                           LidarFactor& lidarFactor, NormalFactor& normalFactor, vector<vector<SlideWindow*>>& sws)
{
  MultiRecutImpl(feat_map, win_count, xs, sws, [&lidarFactor, &normalFactor](OctoTree* oc) {
    oc->tras_opt(lidarFactor);
    oc->tras_opt(normalFactor);
  });
}

// Wrapper: no factor (just recut)
void VINA_SLAM::MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                           vector<vector<SlideWindow*>>& sws)
{
  MultiRecutImpl(feat_map, win_count, xs, sws, nullptr);
}

// ============================================================================
// Helper methods for thd_odometry_localmapping
// ============================================================================

void VINA_SLAM::CleanupReleasedOctos()
{
  if (octos_release.empty())
    return;

  int msize = std::min(static_cast<int>(octos_release.size()), kMaxOctosBatchDelete);
  for (int i = 0; i < msize; i++)
  {
    delete octos_release.back();
    octos_release.pop_back();
  }
  malloc_trim(0);
}

void VINA_SLAM::ReleaseOldVoxels(double jour)
{
  vector<OctoTree*> octos;
  for (auto iter = surf_map.begin(); iter != surf_map.end();)
  {
    double dis = jour - iter->second->jour;
    if (dis < kVoxelReleaseDistThresh)
    {
      ++iter;
      continue;
    }

    octos.push_back(iter->second);
    iter->second->tras_ptr(octos);
    iter = surf_map.erase(iter);
  }

  for (auto* octo : octos)
  {
    delete octo;
  }
  malloc_trim(0);
}

void VINA_SLAM::CleanupSlideWindows()
{
  if (sws[0].size() <= kSlideWindowMaxSize)
    return;

  for (int i = 0; i < kSlideWindowBatchDelete; i++)
  {
    delete sws[0].back();
    sws[0].pop_back();
  }
  malloc_trim(0);
}

void VINA_SLAM::ShiftSlidingWindow(int mgsize)
{
  // Shift state and point cloud buffers
  for (int i = mgsize; i < win_count; i++)
  {
    x_buf[i - mgsize] = x_buf[i];
    std::swap(pvec_buf[i - mgsize], pvec_buf[i]);
  }

  // Remove old entries from the end
  for (int i = 0; i < mgsize; ++i)
  {
    x_buf.pop_back();
    pvec_buf.pop_back();

    delete imu_pre_buf.front();
    imu_pre_buf.pop_front();
  }

  win_base += mgsize;
  win_count -= mgsize;
}

// The main thread of odometry and local mapping

void VINA_SLAM::RunOdometryLocalMapping(std::shared_ptr<rclcpp::Node> node)
{
  PLV(3) pwld;
  Eigen::Vector3d last_pos(0, 0, 0);
  double jour = 0.0;

  pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
  int motion_init_flag = 1;
  pl_tree.reset(new pcl::PointCloud<PointType>());
  vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
  vector<double> beg_times;
  vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>> vec_imus;
  bool release_flag = false;
  int degrade_cnt = 0;

  LidarFactor voxhess(win_size);
  NormalFactor normalFactor(win_size);

  const int mgsize = 1;
  Eigen::MatrixXd hess;
  static IMUST x_last;

  while (rclcpp::ok())
  {
    // Loop detection removed - not needed for localization mode

    node->get_parameter("finish", is_finish);

    if (is_finish)
    {
      break;
    }

    // Synchronize IMU and point cloud data
    deque<std::shared_ptr<sensor_msgs::msg::Imu>> imus;
    bool if_sync_packages = sync_packages(pcl_curr, imus, odom_ekf);

    if (!if_sync_packages)
    {
      // Memory cleanup when no data available
      if (!octos_release.empty())
      {
        CleanupReleasedOctos();
      }
      else if (release_flag)
      {
        release_flag = false;
        ReleaseOldVoxels(jour);
      }
      else if (sws[0].size() > kSlideWindowMaxSize)
      {
        CleanupSlideWindows();
      }
      usleep(kSleepUsNoData);
      continue;
    }

    // Performance timing variables (for debugging, not currently output)
    double t0 = node->now().seconds();
    (void)t0;  // Suppress unused warning
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
    (void)t1;
    (void)t2;
    (void)t3;
    (void)t4;
    (void)t5;
    (void)t6;

    if (motion_init_flag)
    {
      if (pcl_curr->empty())
      {
        RCLCPP_WARN(node->get_logger(), "pcl_curr is empty or null");
        // continue;
      }

      int init = Initialize(imus, hess, voxhess, pwld, pcl_curr);

      if (init == 1)
      {
        RCLCPP_INFO(node->get_logger(), "Initialization succeeded");

        motion_init_flag = 0;
      }
      else
      {
        if (init == -1)
        {
          ResetSystem(imus);
        }

        continue;
      }
    }
    else
    {
      //  EKF状态传播与点云预处理[motion blur Aft motion]
      if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      {
        RCLCPP_WARN(node->get_logger(), "Motion blur failed during initialization");

        continue;
      }

      pcl::PointCloud<PointType> pl_down = *pcl_curr;
      down_sampling_voxel(pl_down, down_size);

      if (pl_down.size() < kMinPointsForDownsample)
      {
        pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size / 2);
      }

      //  误差协方差初始化
      PVecPtr pptr(new PVec);
      var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

      auto pcl_curr_temp = *pcl_curr;
      PVecPtr no_ds_pptr(new PVec);
      var_init(extrin_para, pcl_curr_temp, no_ds_pptr, dept_err, beam_err);

      // if (LioStateEstimation(pptr))
      if (VNCLio(no_ds_pptr))
      {
        if (degrade_cnt > 0)
        {
          degrade_cnt--;
        }
      }
      else
      {
        degrade_cnt++;
      }

      pwld.clear();
      pvec_update(pptr, x_curr, pwld);
      vina_slam::platform::ros2::ResultPublisher::instance().publishLocalTrajectory(pwld, jour, x_curr,
                                                                                    sessionNames.size() - 1, pcl_path);

      t1 = node->now().seconds();

      // 缓存滑动窗口数据
      win_count++;
      x_buf.push_back(x_curr);
      pvec_buf.push_back(pptr);
      if (win_count > 1)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
        imu_pre_buf[win_count - 2]->push_imu(imus);
      }

      //  加载关键帧及地图更新
      LoadKeyframes(jour);
      voxhess.clear();
      voxhess.win_size = win_size;
      normalFactor.clear();
      normalFactor.win_size = win_size;

      cut_voxel_multi(surf_map, pvec_buf[win_count - 1], win_count - 1, surf_map_slide, win_size, pwld, sws);
      t2 = node->now().seconds();

      MultiRecut(surf_map_slide, win_count, x_buf, voxhess, normalFactor, sws);
      t3 = node->now().seconds();

      // ------------------------------visualize plane voxels' sliding-window points after pose estimation
      pcl::PointCloud<PointType> pl_fixd_vis;
      pcl::PointCloud<PointType> pl_wind_vis;
      for (auto& kv : surf_map_slide)
      {
        if (kv.second)
          kv.second->tras_display(win_count, pl_fixd_vis, pl_wind_vis, x_buf);
      }
      pub_pl_func(pl_wind_vis, pub_test, node);

      visualization_msgs::msg::MarkerArray voxel_plane;
      visualization_msgs::msg::MarkerArray voxel_normal;
      std::unordered_set<int> voxel_plane_ids;
      std::unordered_set<int> voxel_normal_ids;
      for (auto& kv : surf_map_slide)
      {
        if (kv.second)
        {
          kv.second->collect_plane_markers(voxel_plane, max_layer, voxel_plane_ids);
          kv.second->collect_normal_markers(voxel_normal, max_layer, voxel_normal_ids);
        }
      }
      pub_voxel_plane->publish(voxel_plane);
      pub_voxel_normal->publish(voxel_normal);
      // ------------------------------ END ------------------------------
    }

    if (win_count >= win_size)
    {
      t4 = node->now().seconds();
      if (if_BA == 1)
      {
        if (g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);

          g_update = 0;
          x_curr.g = x_buf[win_count - 1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;

          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }
      }

      x_last = x_curr;

      x_curr.R = x_buf[win_count - 1].R;
      x_curr.p = x_buf[win_count - 1].p;
      t5 = node->now().seconds();

      //  发布局部地图点云及状态
      vina_slam::platform::ros2::ResultPublisher::instance().publishLocalMapFull(
          mgsize, sessionNames.size() - 1, pvec_buf, x_buf, pcl_path, win_base, win_count);

      //  执行局部地图边缘化
      MultiMarginalize(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
      t6 = node->now().seconds();

      //  根据位移判断是否触发地图释放机制
      if ((win_base + win_count) % 10 == 0)
      {
        double spat = (x_curr.p - last_pos).norm();
        if (spat > kVoxelDownsampleSize)
        {
          jour += spat;
          last_pos = x_curr.p;
          release_flag = true;
        }
      }

      if (is_save_map)
      {
        for (int i = 0; i < mgsize; i++)
          vina_slam::platform::ros2::FileReaderWriter::instance().savePcd(pvec_buf[i], x_buf[i], win_base + i,
                                                                          savepath + bagname);
      }

      // Update sliding window index map (mp is a global variable)
      for (int i = 0; i < win_size; i++)
      {
        mp[i] += mgsize;
        if (mp[i] >= win_size)
          mp[i] -= win_size;
      }

      // Shift sliding window buffers
      ShiftSlidingWindow(mgsize);
    }
    double mem = get_memory();
    RCLCPP_INFO(node->get_logger(), "Memory usage: %.2f GB", mem);
  }

  vector<OctoTree*> octos;
  for (auto& kv : surf_map)
  {
    kv.second->tras_ptr(octos);
    kv.second->clear_slwd(sws[0]);
    delete kv.second;
  }

  for (auto* octo : octos)
  {
    delete octo;
  }
  octos.clear();

  for (auto* sw : sws[0])
  {
    delete sw;
  }
  sws[0].clear();
  malloc_trim(0);
}
// Sensor callbacks (imu_handler, pcl_handler, sync_packages) remain here
// for now as they access global variables. They will be migrated to
// SensorSubscribers in a future phase.

void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg_in)
{
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

  mBuf.lock();
  imu_last_time = rclcpp::Time(msg->header.stamp).seconds();

  imu_buf.push_back(msg);
  mBuf.unlock();
}

template <class T>
void pcl_handler(T& msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());

  double t_start = feat.process(msg, *pl_ptr);
  pcl_time_lock.lock();
  pcl_time = t_start;
  pcl_time_lock.unlock();

  if (pl_ptr->empty())
  {
    PointType ap;
    ap.x = 0;
    ap.y = 0;
    ap.z = 0;
    ap.intensity = 0;
    ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType& x, PointType& y) { return x.curvature < y.curvature; });

  while (pl_ptr->back().curvature > 0.11)
  {
    pl_ptr->points.pop_back();
  }

  mBuf.lock();

  time_buf.push_back(t_start);
  pcl_buf.push_back(pl_ptr);

  mBuf.unlock();
}

bool sync_packages(pcl::PointCloud<PointType>::Ptr& pl_ptr, deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                   IMUEKF& p_imu)
{
  static bool pl_ready = false;

  if (!pl_ready)
  {
    if (pcl_buf.empty())
    {
      return false;
    }

    mBuf.lock();

    pl_ptr = pcl_buf.front();
    pcl_buf.pop_front();
    p_imu.pcl_beg_time = time_buf.front();
    time_buf.pop_front();

    mBuf.unlock();

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    if (point_notime)
    {
      if (last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pcl_time_lock.lock();
    pcl_end_time = p_imu.pcl_end_time;
    pcl_time_lock.unlock();

    pl_ready = true;
  }

  if (!pl_ready || imu_last_time <= p_imu.pcl_end_time)
  {
    return false;
  }

  mBuf.lock();
  double imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
  while ((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
    if (imu_time > p_imu.pcl_end_time)
      break;
    imus.push_back(imu_buf.front());
    imu_buf.pop_front();
  }
  mBuf.unlock();

  if (imu_buf.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("vina_slam"), "IMU buffer empty - data flow interrupted");
    return false;
  }

  pl_ready = false;

  return (imus.size() > 4);
}
