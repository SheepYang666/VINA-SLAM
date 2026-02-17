#include "vina_slam/VINASlam.hpp"
#include "vina_slam/voxel_map.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cstddef>
#include <cstdio>
#include <ios>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
// Initialization class moved to vina_slam/pipeline/initialization.hpp

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

  // ######################################## print log ########################################

  if (is_save_map == 0)
  {
    std::cout << YELLOW << "[is_save_map]: don't save map" << RESET << std::endl;
  }
  else if (is_save_map == 1)
  {
    std::cout << GREEN << "[is_save_map]: save map" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[is_save_map]: ERROR STATE " << RESET << std::endl;
  }

  if (if_loop_dect == 0)
  {
    std::cout << YELLOW << "[if_loop_dect]: loop don't dect" << RESET << std::endl;
  }
  else if (if_loop_dect == 1)
  {
    std::cout << GREEN << "[if_loop_dect]: loop dect" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[if_loop_dect]: ERROR STATE " << RESET << std::endl;
  }

  if (if_BA == 0)
  {
    std::cout << YELLOW << "[if_BA]: don't BA" << RESET << std::endl;
  }
  else if (if_BA == 1)
  {
    std::cout << GREEN << "[if_BA]: BA" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[if_BA]: ERROR STATE " << RESET << std::endl;
  }

  // ######################################## print log ########################################

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

  int ss = 0;
  if (access((savepath + bagname + "/").c_str(), X_OK) == -1)
  {
    string cmd = "mkdir " + savepath + bagname + "/";
    ss = system(cmd.c_str());
  }
  else
    ss = -1;

  if (ss != 0 && is_save_map == 1)
  {
    printf("The pointcloud will be saved in this run.\n");
    printf("So please clear or rename the existed folder.\n");
    exit(0);
  }

  sws.resize(thread_num);
  cout << "bagname: " << bagname << endl;
}

bool VINA_SLAM::LioStateEstimation(PVecPtr pptr)
{
  IMUST x_prop = x_curr;

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

    if ((rot_add.norm() * kRadToDeg < kRotConvergeThreshDeg) && (tra_add.norm() * kMeterToCm < kTraConvergeThreshCm))
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

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  Eigen::Vector3d evalue = saes.eigenvalues();

  if (evalue[0] < kPlaneEigenvalueThresh)
  {
    return false;
  }
  else
  {
    return true;
  }
}

// ============================================================================
// VNC (Vector Normal Consistency) LIO State Estimation
// ============================================================================
// Extends LioStateEstimation with additional rotation constraint from
// normal vector consistency between scan and map planes.
// See: docs/theory/unified_residual_framework.md for theoretical details.
// ============================================================================

bool VINA_SLAM::VNCLio(PVecPtr pptr)
{
  // Note: Constants (kRadToDeg, kMeterToCm, etc.) are globally defined in constants.hpp

  IMUST x_prop = x_curr;

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

  // ========== VNC Parameters ==========
  const double VNC_EPSILON = 1e-6;            // Small angle threshold
  const double VNC_ALPHA = 0.1;               // VNC weight coefficient
  const double VNC_VOXEL_SIZE = voxel_size;   // Voxel size for scan plane extraction
  const int VNC_MAX_LAYER = 2;                // Max subdivision depth
  const int VNC_MIN_POINTS_SUBDIVIDE = 10;    // Min points to consider subdivision
  const double VNC_EIGEN_RATIO_THRESH = 0.1;  // Eigenvalue ratio threshold for plane

  // ========== Build temporary voxel map for current scan ==========
  // This creates voxels from the current frame and fits planes to get n_scan
  unordered_map<VOXEL_LOC, OctoTree*> scan_voxels;
  PVec pvec_world = *pptr;  // Copy for transformation

  // Transform points to world frame for voxelization
  for (auto& pv : pvec_world)
  {
    pv.pnt = x_curr.R * pv.pnt + x_curr.p;
  }

  // Build voxel map and fit planes
  generate_voxel(scan_voxels, pvec_world, VNC_VOXEL_SIZE);

  // ========== Compute eigenvalues and subdivide if needed ==========
  // Recursive lambda for plane fitting with subdivision
  std::function<void(OctoTree*, int)> fit_plane_with_subdivide = [&](OctoTree* ot, int depth) {
    if (ot == nullptr || ot->point_fix.size() < 3)
    {
      return;
    }

    // Compute covariance and eigenvalues from points
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& pv : ot->point_fix)
    {
      mean += pv.pnt;
    }
    mean /= ot->point_fix.size();

    for (const auto& pv : ot->point_fix)
    {
      Eigen::Vector3d d = pv.pnt - mean;
      cov += d * d.transpose();
    }
    cov /= ot->point_fix.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(cov);
    ot->eig_value = eig_solver.eigenvalues();
    ot->eig_vector = eig_solver.eigenvectors();

    // Check if this is a valid plane
    double eig_ratio = ot->eig_value[0] / (ot->eig_value[2] + 1e-10);
    bool is_plane = (ot->eig_value[0] < min_eigen_value) && (eig_ratio < VNC_EIGEN_RATIO_THRESH);

    if (is_plane)
    {
      // Valid plane - compute plane parameters
      ot->plane.center = mean;
      ot->plane.normal = ot->eig_vector.col(0);  // Min eigenvector
      ot->plane.is_plane = true;
      ot->plane.radius = ot->eig_value[2];
      ot->isexist = true;

      // Ensure normal points towards sensor
      Eigen::Vector3d center_body = x_curr.R.transpose() * (ot->plane.center - x_curr.p);
      if (ot->plane.normal.dot(center_body) > 0)
      {
        ot->plane.normal = -ot->plane.normal;
      }
    }
    else if (depth < VNC_MAX_LAYER && ot->point_fix.size() >= VNC_MIN_POINTS_SUBDIVIDE)
    {
      // Not a plane but enough points - subdivide
      ot->octo_state = 1;  // Mark as subdivided

      // Distribute points to child voxels
      for (auto& pv : ot->point_fix)
      {
        int xyz[3] = { 0, 0, 0 };
        for (int k = 0; k < 3; k++)
        {
          if (pv.pnt[k] > ot->voxel_center[k])
          {
            xyz[k] = 1;
          }
        }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        if (ot->leaves[leafnum] == nullptr)
        {
          ot->leaves[leafnum] = new OctoTree(ot->layer + 1, 1);
          ot->leaves[leafnum]->voxel_center[0] = ot->voxel_center[0] + (2 * xyz[0] - 1) * ot->quater_length;
          ot->leaves[leafnum]->voxel_center[1] = ot->voxel_center[1] + (2 * xyz[1] - 1) * ot->quater_length;
          ot->leaves[leafnum]->voxel_center[2] = ot->voxel_center[2] + (2 * xyz[2] - 1) * ot->quater_length;
          ot->leaves[leafnum]->quater_length = ot->quater_length / 2;
        }

        ot->leaves[leafnum]->push_fix(pv);
      }

      // Recursively fit planes for children
      for (int i = 0; i < 8; i++)
      {
        if (ot->leaves[i] != nullptr)
        {
          fit_plane_with_subdivide(ot->leaves[i], depth + 1);
        }
      }
    }
  };

  // Apply plane fitting with subdivision to all voxels
  for (auto& kv : scan_voxels)
  {
    fit_plane_with_subdivide(kv.second, 0);
  }

  // VNC pairs storage
  vector<VNCPair> vnc_pairs;
  vnc_pairs.reserve(psize / 4);

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

    // Clear VNC pairs at each iteration (re-match)
    vnc_pairs.clear();

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

        // ========== Point-to-Plane Residual (Original) ==========
        double R_inv = 1.0 / (0.0005 + sigma_d);
        double resi = pp.normal.dot(wld - pp.center);

        Eigen::Matrix<double, 6, 1> jac;
        jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
        jac.tail(3) = pp.normal;
        HTH += R_inv * jac * jac.transpose();
        HTz -= R_inv * jac * resi;
        nnt += pp.normal * pp.normal.transpose();
        match_num++;

        // ========== VNC Residual Extraction ==========
        // Find the scan voxel this point belongs to (with subdivision support)
        float loc[3];
        for (int j = 0; j < 3; j++)
        {
          loc[j] = wld[j] / VNC_VOXEL_SIZE;
          if (loc[j] < 0)
            loc[j] -= 1;
        }
        VOXEL_LOC voxel_loc(loc[0], loc[1], loc[2]);

        auto it = scan_voxels.find(voxel_loc);
        if (it != scan_voxels.end() && pp.is_plane)
        {
          // Recursive search for the finest valid plane containing this point
          std::function<OctoTree*(OctoTree*, const Eigen::Vector3d&)> find_finest_plane =
              [&](OctoTree* ot, const Eigen::Vector3d& pt) -> OctoTree* {
            if (ot == nullptr)
            {
              return nullptr;
            }

            // If this voxel has a valid plane, check if we should go deeper
            if (ot->plane.is_plane)
            {
              // Check if any child has a valid plane containing the point
              for (int k = 0; k < 8; k++)
              {
                if (ot->leaves[k] != nullptr && ot->leaves[k]->plane.is_plane)
                {
                  // Check if point is in this child's voxel
                  bool inside = true;
                  for (int d = 0; d < 3; d++)
                  {
                    double half_size = ot->quater_length;
                    if (std::abs(pt[d] - ot->leaves[k]->voxel_center[d]) > half_size)
                    {
                      inside = false;
                      break;
                    }
                  }
                  if (inside)
                  {
                    // Recursively search in this child
                    OctoTree* finer = find_finest_plane(ot->leaves[k], pt);
                    if (finer != nullptr)
                    {
                      return finer;  // Return finer plane
                    }
                  }
                }
              }
              return ot;  // No finer plane found, return this one
            }

            // If this voxel is subdivided but not a plane itself, search children
            if (ot->octo_state == 1)
            {
              for (int k = 0; k < 8; k++)
              {
                if (ot->leaves[k] != nullptr)
                {
                  bool inside = true;
                  for (int d = 0; d < 3; d++)
                  {
                    double half_size = ot->quater_length;
                    if (std::abs(pt[d] - ot->leaves[k]->voxel_center[d]) > half_size)
                    {
                      inside = false;
                      break;
                    }
                  }
                  if (inside)
                  {
                    OctoTree* result = find_finest_plane(ot->leaves[k], pt);
                    if (result != nullptr)
                    {
                      return result;
                    }
                  }
                }
              }
            }

            return nullptr;
          };

          OctoTree* scan_ot = find_finest_plane(it->second, wld);
          if (scan_ot != nullptr && scan_ot->plane.is_plane)
          {
            // Get scan plane normal (in world frame)
            Eigen::Vector3d n_scan_world = scan_ot->plane.normal.normalized();

            // Transform to body frame for Jacobian computation
            Eigen::Vector3d n_scan_body = x_curr.R.transpose() * n_scan_world;

            // Plane quality based on eigenvalue ratio
            // Smaller min eigenvalue = flatter plane = better quality
            double lambda_min = scan_ot->eig_value[0];
            double lambda_mid = scan_ot->eig_value[1];
            double lambda_max = scan_ot->eig_value[2];
            double plane_quality = 1.0 - lambda_min / (lambda_min + lambda_mid + lambda_max + 1e-10);

            if (plane_quality > 0.5)  // Quality threshold
            {
              VNCPair vnc;
              vnc.n_scan = n_scan_body.normalized();
              vnc.n_map = pp.normal.normalized();
              vnc.p_scan = pv.pnt;
              vnc.sigma_n = sqrt(lambda_min / (lambda_min + lambda_mid + lambda_max + 1e-10));
              vnc.weight = plane_quality / (vnc.sigma_n * vnc.sigma_n + 0.01);
              vnc.skew_n_scan = hat(vnc.n_scan);
              vnc_pairs.push_back(vnc);
            }
          }
        }
      }
    }

    // ========== VNC Residual Accumulation ==========
    int vnc_count = 0;
    for (const auto& vnc : vnc_pairs)
    {
      // Transform scan normal to world frame
      Eigen::Vector3d n_scan_world = x_curr.R * vnc.n_scan;

      // Tangent space projection (onto map normal's tangent plane)
      Eigen::Matrix3d Pi = Eigen::Matrix3d::Identity() - vnc.n_map * vnc.n_map.transpose();
      Eigen::Vector3d r_vec = Pi * n_scan_world;

      // Scalar residual (tangent space distance)
      double r_n = r_vec.norm();

      // Skip small angles (numerical stability)
      if (r_n < VNC_EPSILON)
      {
        continue;
      }

      // Direction vector in tangent plane
      Eigen::Vector3d t = r_vec / r_n;

      // VNC Jacobian (6×1)
      // J_rot = t^T * R * [n_scan]×  → transpose to get 3×1
      // J_trs = 0 (normal is translation invariant)
      Eigen::Matrix<double, 6, 1> jac_n;
      jac_n.head(3) = (t.transpose() * x_curr.R * vnc.skew_n_scan).transpose();
      jac_n.tail(3) = Eigen::Vector3d::Zero();

      // Weight with VNC coefficient
      double w_n = VNC_ALPHA * vnc.weight;

      // Accumulate (same form as point-to-plane!)
      HTH += w_n * jac_n * jac_n.transpose();
      HTz -= w_n * jac_n * r_n;
      vnc_count++;
    }

    // Debug output for VNC
    RCLCPP_DEBUG(node->get_logger(), "VNC: %d pairs, HTH trace: %.4f", vnc_count, HTH.trace());

    // ========== IEKF Update ==========
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

    if ((rot_add.norm() * kRadToDeg < kRotConvergeThreshDeg) && (tra_add.norm() * kMeterToCm < kTraConvergeThreshCm))
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

  // ========== Cleanup temporary scan voxel map (recursive) ==========
  std::function<void(OctoTree*)> delete_octree = [&](OctoTree* ot) {
    if (ot == nullptr)
    {
      return;
    }
    // Recursively delete children
    for (int k = 0; k < 8; k++)
    {
      if (ot->leaves[k] != nullptr)
      {
        delete_octree(ot->leaves[k]);
        ot->leaves[k] = nullptr;
      }
    }
    delete ot;
  };

  for (auto& kv : scan_voxels)
  {
    delete_octree(kv.second);
  }
  scan_voxels.clear();

  // ========== Degeneracy Check ==========
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  Eigen::Vector3d evalue = saes.eigenvalues();

  if (evalue[0] < kPlaneEigenvalueThresh)
  {
    return false;
  }
  else
  {
    return true;
  }
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
  pub_pl_func(pcl_path, pub_cmap, node);

  std::cout << "\033[31mReset\033[0m" << std::endl;
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

  vector<thread*> mthreads(thd_num);
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

  // 创建多个线程并行执行边缘化任务，主线程执行第0块任务
  for (int i = 1; i < thd_num; i++)
  {
    mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
  }

  // 等待所有子线程完成，清理线程资源
  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      margi_func(win_count, octs[i], xs, voxopt);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

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

  vector<thread*> mthreads(thd_num);
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

  // Launch threads
  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));

  // Main thread does first batch, then joins others
  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      recut_func(win_count, octss[i], xs, sws[i]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

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
        std::cout << RED << "init success" << RESET << std::endl;

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
        std::cout << RED << "motion blur failed" << RESET << std::endl;

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
    std::cout << GREEN << "mem:" << mem << RESET << std::endl;
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
// Build the pose graph in loop closure

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

  // Step 1: If the point cloud is not ready yet, take a frame out of the cache
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

    // If the time stamp mode is turned on, the time information is simulated using frame time intervals.
    if (point_notime)
    {
      if (last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;

        return false;
      }

      // Manually set the start/end time of point cloud frame
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

  // Step 3: Extract IMU data in the range [pcl_beg_time, pcl_end_time]
  mBuf.lock();
  double imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
  while ((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
    if (imu_time > p_imu.pcl_end_time)
      break;
    imus.push_back(imu_buf.front());  // Press the corresponding IMU data of the current frame
    imu_buf.pop_front();
  }
  mBuf.unlock();

  // If the IMU data is used up, it means the data flow is broken and the program is exited
  if (imu_buf.empty())
  {
    exit(0);
  }

  pl_ready = false;  // The current frame processing is completed, the flag is reset

  // If the number of paired IMU data is greater than 4, the synchronization is considered successful
  if (imus.size() > 4)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void calcBodyVar(Eigen::Vector3d& pb, const float range_inc, const float degree_inc, Eigen::Matrix3d& var)
{
  // To prevent division by 0, make sure that the z coordinate is non-zero (avoid singular division)
  if (pb[2] == 0)
  {
    pb[2] = 0.0001;
  }

  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;

  // Construct the direction angle error covariance matrix (units in radians)
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

  Eigen::Vector3d direction(pb);
  direction.normalize();

  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;

  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();

  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();

  // Construct orthogonal projection substrate matrix N ∈ ℝ³×²
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);

  // Construct the angle error term direction projection matrix A ∈ ℝ³×²
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;

  // Construct the population covariance matrix: V = σ_r² *r̂ *r̂^T + A *σ_ang² *A^T
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

// Compute the variance of the each point

void var_init(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);

  for (int i = 0; i < plsize; i++)
  {
    PointType& ap = pl_cur[i];
    pointVar& pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
    pv.intensity = pl_cur[i].intensity;
  }
}

void pvec_update(PVecPtr pptr, IMUST& x_curr, PLV(3) & pwld)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for (pointVar& pv : *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

// read_lidarstate removed - not needed for localization mode

double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;
  while (getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;
    while (ss >> str)
    {
      if (str == "VmRSS:")
      {
        is_find = true;
        continue;
      }

      if (is_find)
        mem = stod(str);
      break;
    }
    if (is_find)
      break;
  }
  return mem / (1048576);
}

// icp_check removed - not needed for localization mode

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("vina_slam");
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  exec->add_node(node);

  pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cmap", 100);
  pub_pmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pmap", 100);
  pub_prev_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_true", 100);
  pub_init = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_init", 100);

  pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan", 100);
  pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_path", 100);
  pub_test = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);
  pub_voxel_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_plane", 10);
  pub_voxel_normal = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_normal", 10);
  pub_pmap_livox = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/map_pmap_livox", 10);

  vina_slam::platform::ros2::ResultPublisher::instance(node);
  vina_slam::platform::ros2::FileReaderWriter::instance(node);
  vina_slam::pipeline::Initialization::instance(node);
  VINA_SLAM vs(node);

  mp.resize(vs.win_size);
  for (size_t i = 0; i < mp.size(); i++)
  {
    mp[i] = i;
  }

  std::thread thread_odom(&VINA_SLAM::thd_odometry_localmapping, &vs, node);

  exec->spin();
  thread_odom.join();

  return 0;
}
