/**
 * @file initialization.cpp
 * @brief Implementation of initialization pipeline
 */

#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/core/math.hpp"
#include "vina_slam/core/sensor_context.hpp"
#include "vina_slam/voxel_map.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

namespace vina_slam {
namespace pipeline {

Initialization::Initialization(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {}

Initialization& Initialization::instance(const rclcpp::Node::SharedPtr& node_in) {
  static Initialization inst(node_in);
  return inst;
}

Initialization& Initialization::instance() {
  rclcpp::Node::SharedPtr node_temp;
  return instance(node_temp);
}

void Initialization::alignGravity(std::vector<core::IMUST>& xs) {
  if (xs.empty())
    return;

  Eigen::Vector3d g0 = xs[0].g;
  Eigen::Vector3d n0 = g0.normalized();

  Eigen::Vector3d n1(0, 0, 1);

  // If the current direction is facing down, set the target direction to the negative z axis.
  if (n0[2] < 0) {
    n1[2] = -1;
  }

  Eigen::Vector3d rotvec = n0.cross(n1);  // Calculate the rotation vector
  double rnorm = rotvec.norm();           // The rotation angle (sin(θ))
  rotvec = rotvec / rnorm;                // Uniform rotation axis

  // Construct rotation: the angle is asin(rnorm), the axis is rotvec
  Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
  Eigen::Matrix3d rot = angaxis.toRotationMatrix();
  g0 = rot * g0;  // Rotate the gravity vector

  // Get the first position as the reference point and apply rotation transformation to all states
  Eigen::Vector3d p0 = xs[0].p;
  for (size_t i = 0; i < xs.size(); i++) {
    xs[i].p = rot * (xs[i].p - p0) + p0;
    xs[i].R = rot * xs[i].R;
    xs[i].v = rot * xs[i].v;
    xs[i].g = g0;
  }
}

void Initialization::motionBlur(pcl::PointCloud<core::PointType>& pl, core::PVec& pvec,
                                core::IMUST xc, core::IMUST xl,
                                std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                                double pcl_beg_time, core::IMUST& extrin_para) {
  // Initialize bias + current IMU status variable
  xc.bg = xl.bg;
  xc.ba = xl.ba;
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
  Eigen::Matrix3d R_imu(xc.R);
  std::vector<core::IMUST> imu_poses;

  // IMU Integral build status sequence
  for (auto it_imu = imus.end() - 1; it_imu != imus.begin(); it_imu--) {
    sensor_msgs::msg::Imu& head = **(it_imu - 1);
    sensor_msgs::msg::Imu& tail = **it_imu;

    // Mean filtering (interpolated angular velocity and acceleration)
    angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
        0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
        0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
    acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
        0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
        0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

    angvel_avr -= xc.bg;
    acc_avr = acc_avr * estimation::imupre_scale_gravity - xc.ba;

    // Pose increment (exp map)
    double dt = rclcpp::Time(head.header.stamp).seconds() - rclcpp::Time(tail.header.stamp).seconds();
    Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

    // Linear acceleration integration
    acc_imu = R_imu * acc_avr + xc.g;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;
    R_imu = R_imu * Exp_f;

    // Save the current status to imu_poses
    double offt = rclcpp::Time(head.header.stamp).seconds() - pcl_beg_time;
    imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
  }

  core::pointVar pv;
  pv.var.setIdentity();
  pv.intensity = 0.0f;  // Initialize to avoid -Wmaybe-uninitialized

  // IMU compensation is not done without a time sequence number
  if (core::SensorContext::Instance().point_notime) {
    for (core::PointType& ap : pl.points) {
      pv.pnt << ap.x, ap.y, ap.z;
      pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
      pvec.push_back(pv);
    }
    return;
  }

  // If you have a time serial number, you can make IMU compensation
  auto it_pcl = pl.end() - 1;

  for (auto it_kp = imu_poses.begin(); it_kp != imu_poses.end(); it_kp++) {
    core::IMUST& head = *it_kp;
    R_imu = head.R;
    acc_imu = head.ba;
    vel_imu = head.v;
    pos_imu = head.p;
    angvel_avr = head.bg;

    for (; it_pcl->curvature > head.t; it_pcl--) {
      double dt = it_pcl->curvature - head.t;
      Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
      Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

      pv.pnt = P_compensate;
      pvec.push_back(pv);
      if (it_pcl == pl.begin()) {
        break;
      }
    }
  }
}

int Initialization::motionInit(std::vector<pcl::PointCloud<core::PointType>::Ptr>& pl_origs,
                               std::vector<std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>>& vec_imus,
                               std::vector<double>& beg_times, Eigen::MatrixXd* hess,
                               LidarFactor& voxhess, std::vector<core::IMUST>& x_buf,
                               std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map,
                               std::unordered_map<core::VOXEL_LOC, OctoTree*>& surf_map_slide,
                               std::vector<core::PVecPtr>& pvec_buf, int win_size,
                               std::vector<std::vector<SlideWindow*>>& sws,
                               core::IMUST& x_curr, std::deque<estimation::ImuPreintegration*>& imu_pre_buf,
                               core::IMUST& extrin_para) {
  PLV(3) pwld;  // World-frame de-skewed point cloud

  int converge_flag = 0;  // 1 means converged

  // Store original eigenvalue thresholds (global variables from voxel_map.hpp)
  double min_eigen_value_orig = min_eigen_value;
  std::vector<double> eigen_value_array_orig = plane_eigen_value_thre;

  // Relax thresholds for initialization
  min_eigen_value = 0.02;
  for (double& iter : plane_eigen_value_thre) {
    iter = 1.0 / 4;
  }

  double converge_thre = 0.05;  // Convergence threshold: relative change < 5%
  bool is_degrade = true;
  Eigen::Vector3d eigvalue;
  eigvalue.setZero();

  for (int iterCnt = 0; iterCnt < 10; iterCnt++) {
    if (converge_flag == 1) {
      min_eigen_value = min_eigen_value_orig;
      plane_eigen_value_thre = eigen_value_array_orig;
    }

    // Clear previous voxel map and sliding window references
    std::vector<OctoTree*> octos;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for (size_t i = 0; i < octos.size(); i++) {
      delete octos[i];
    }
    surf_map.clear();
    octos.clear();
    surf_map_slide.clear();

    // Motion blur correction and voxel insertion for each frame
    for (int i = 0; i < win_size; i++) {
      pwld.clear();
      pvec_buf[i]->clear();

      int l = (i == 0) ? i : i - 1;
      motionBlur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

      if (converge_flag == 1) {
        // Calculate point covariance after convergence
        for (core::pointVar& pv : *pvec_buf[i]) {
          Eigen::Vector3d pb = pv.pnt;
          Eigen::Matrix3d var;
          // calcBodyVar equivalent - simplified version
          float range = pb.norm();
          float range_var = core::SensorContext::Instance().dept_err * core::SensorContext::Instance().dept_err;
          float angle_var = core::SensorContext::Instance().beam_err * core::SensorContext::Instance().beam_err;
          var.setZero();
          if (range > 0.01f) {
            Eigen::Vector3d nx = pb / range;
            Eigen::Matrix3d N = nx * nx.transpose();
            Eigen::Matrix3d N_p = Eigen::Matrix3d::Identity() - N;
            var = N * range_var + N_p * angle_var * range * range;
          }
          pv.var = var;
        }
        // pvec_update equivalent
        for (core::pointVar& pv : *pvec_buf[i]) {
          pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }
      } else {
        // Direct transformation to world frame
        for (core::pointVar& pv : *pvec_buf[i]) {
          pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }
      }

      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    // Build LiDAR factors
    voxhess.clear();
    voxhess.win_size = win_size;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      iter->second->recut(win_size, x_buf, sws[0]);
      iter->second->tras_opt(voxhess);
    }

    if (voxhess.plvec_voxels.size() < 10) {
      break;
    }

    // Joint optimization (IMU + LiDAR)
    LI_BA_OptimizerGravity opt_lsv;
    std::vector<double> resis;

    opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);

    // Clear and reconstruct IMU preintegration factors
    for (int i = 0; i < win_size - 1; i++) {
      delete imu_pre_buf[i];
    }
    imu_pre_buf.clear();

    for (int i = 1; i < win_size; i++) {
      imu_pre_buf.push_back(new estimation::ImuPreintegration(x_buf[i - 1].bg, x_buf[i - 1].ba));
      imu_pre_buf.back()->pushImu(vec_imus[i]);
    }

    // Check convergence
    if (resis.size() >= 2 && fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2) {
      Eigen::Matrix3d nnt;
      nnt.setZero();

      for (Eigen::Matrix3d& iter : voxhess.eig_vectors) {
        Eigen::Vector3d v3 = iter.col(0);
        nnt += v3 * v3.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
      eigvalue = saes.eigenvalues();

      // Check for degradation (lack of constraint in one direction)
      is_degrade = eigvalue[0] < 15;

      // Lower convergence threshold after initial convergence
      converge_thre = 0.01;

      if (converge_flag == 0) {
        alignGravity(x_buf);
        converge_flag = 1;
        continue;  // Restart optimization with gravity alignment
      } else {
        break;  // Already gravity-aligned and converged
      }
    }
  }

  // Extract final initialization state
  x_curr = x_buf[win_size - 1];
  double gnm = x_curr.g.norm();

  // Gravity value bounds
  double g_min_value = 9.6;
  double g_max_value = 10.0;

  // Debug output
  std::cout << "\teigvalue[0] = " << eigvalue[0] << std::endl;
  std::cout << "\teigvalue[1] = " << eigvalue[1] << std::endl;
  std::cout << "\teigvalue[2] = " << eigvalue[2] << std::endl;
  std::cout << "\tlast data frame |G| = " << gnm << std::endl;

  // Check for degradation
  if (is_degrade) {
    std::cout << "\033[31m\tgravity direction degrade\033[0m" << std::endl;
    converge_flag = 0;
  }

  // Check for gravity value anomaly
  if (gnm < g_min_value || gnm > g_max_value) {
    std::cout << "\033[31m\tgravity value anomaly\033[0m" << std::endl;
    converge_flag = 0;
  }

  // Cleanup on failure
  if (converge_flag == 0) {
    std::vector<OctoTree*> octos;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    for (size_t i = 0; i < octos.size(); i++) {
      delete octos[i];
    }
    surf_map.clear();
    octos.clear();
    surf_map_slide.clear();
  }

  // Print first IMU state info
  Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y,
                       vec_imus[0][0]->angular_velocity.z);
  Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y,
                      vec_imus[0][0]->linear_acceleration.z);
  acc *= 9.8;

  // Clear input buffers
  pl_origs.clear();
  vec_imus.clear();
  beg_times.clear();

  // Build and publish initialization point cloud
  pcl::PointCloud<core::PointType> pcl_send;
  core::PointType pt;
  for (int i = 0; i < win_size; i++) {
    for (core::pointVar& pv : *pvec_buf[i]) {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0];
      pt.y = vv[1];
      pt.z = vv[2];
      pcl_send.push_back(pt);
    }
  }

  // Publish initialization point cloud (using global pub_init)
  if (pub_init && node) {
    pcl_send.height = 1;
    pcl_send.width = pcl_send.size();
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(pcl_send, output);
    output.header.frame_id = "camera_init";
    output.header.stamp = node->now();
    pub_init->publish(output);
  }

  return converge_flag;
}

} // namespace pipeline
} // namespace vina_slam
