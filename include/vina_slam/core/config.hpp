#pragma once

#include "vina_slam/core/types.hpp"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace vina_slam
{
namespace core
{

struct VoxelMapConfig
{
  double voxel_size = 1.0;
  double min_eigen_value = 0.0025;
  int max_layer = 2;
  int max_points = 100;
  int min_ba_point = 20;
  std::vector<double> plane_eigen_value_thre;
  Eigen::Vector4d min_point{20, 20, 15, 10};
};

struct ImuNoiseConfig
{
  double cov_gyr = 0.1;
  double cov_acc = 0.1;
  double rand_walk_gyr = 1e-4;
  double rand_walk_acc = 1e-4;
  double imu_coef = 1e-4;
  double scale_gravity = 1.0;
  Eigen::Matrix<double, 6, 6> noise_meas = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> noise_walk = Eigen::Matrix<double, 6, 6>::Identity();
};

struct SensorConfig
{
  double dept_err = 0.02;
  double beam_err = 0.05;
  int lidar_type = 0;
  double blind = 0.1;
  int point_filter_num = 3;
  int point_notime = 0;
};

struct ExtrinsicConfig
{
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

struct PipelineConfig
{
  int win_size = 10;
  double down_size = 0.1;
  int if_BA = 0;
  int is_save_map = 0;
  int thread_num = 5;
  int degrade_bound = 10;
  double full_map_voxel_size = 0.05;
};

struct TopicConfig
{
  std::string lid_topic = "/rslidar_points";
  std::string imu_topic = "/imu";
};

struct SystemConfig
{
  VoxelMapConfig voxel;
  ImuNoiseConfig odom_imu;
  ImuNoiseConfig local_ba_imu;
  SensorConfig sensor;
  ExtrinsicConfig extrinsic;
  PipelineConfig pipeline;
  TopicConfig topics;
  std::string bagname = "noNameBag";
  std::string savepath;
};

}  // namespace core
}  // namespace vina_slam
