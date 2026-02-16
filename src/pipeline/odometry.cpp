/**
 * @file odometry.cpp
 * @brief Implementation of odometry pipeline
 */

#include "vina_slam/pipeline/odometry.hpp"
#include "vina_slam/core/math.hpp"
#include "vina_slam/mapping/voxel_map.hpp"

namespace vina_slam {
namespace pipeline {

OdometryPipeline::OdometryPipeline() {
  x_curr.setZero();
}

void OdometryPipeline::initialize(int window_size) {
  win_size = window_size;
  max_win_size = window_size;
  x_buf.resize(window_size);
  for (auto& x : x_buf) {
    x.setZero();
  }
  imu_pre_buf.clear();
  is_initialized = false;
}

void OdometryPipeline::setImuNoise(const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyr,
                                   const Eigen::Vector3d& cov_bias_acc, const Eigen::Vector3d& cov_bias_gyr) {
  imu_ekf.cov_acc = cov_acc;
  imu_ekf.cov_gyr = cov_gyr;
  imu_ekf.cov_bias_acc = cov_bias_acc;
  imu_ekf.cov_bias_gyr = cov_bias_gyr;
}

void OdometryPipeline::setExtrinsics(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
  extrin_para.R = rotation;
  extrin_para.p = translation;
  imu_ekf.Lid_rot_to_IMU = rotation;
  imu_ekf.Lid_offset_to_IMU = translation;
}

int OdometryPipeline::processImu(std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                                 pcl::PointCloud<core::PointType>& pcl_in) {
  return imu_ekf.process(x_curr, pcl_in, imus);
}

int OdometryPipeline::stateEstimation(core::PVec& pvec,
                                      std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
                                      std::vector<mapping::SlideWindow*>& /*sws*/) {
  int match_num = 0;

  // Point-to-plane matching for EKF update
  for (auto& pv : pvec) {
    Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;
    core::Plane* pla = nullptr;
    mapping::OctoTree* oc = nullptr;
    Eigen::Matrix3d var_wld = x_curr.R * pv.var * x_curr.R.transpose();
    double sigma_d = 0;

    if (mapping::matchVoxelMap(surf_map, wld, pla, var_wld, sigma_d, oc) > 0) {
      if (pla && pla->is_plane) {
        match_num++;
      }
    }
  }

  return match_num;
}

void OdometryPipeline::addFrame(core::PVecPtr /*pvec*/) {
  // Add new state to buffer
  if (x_buf.size() < static_cast<size_t>(max_win_size)) {
    x_buf.push_back(x_curr);
  } else {
    // Shift buffer
    for (size_t i = 0; i < x_buf.size() - 1; i++) {
      x_buf[i] = x_buf[i + 1];
    }
    x_buf.back() = x_curr;
  }
}

} // namespace pipeline
} // namespace vina_slam
