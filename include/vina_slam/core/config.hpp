/**
 * @file config.hpp
 * @brief Centralized configuration structs for VINA-SLAM
 *
 * Replaces scattered global variables with structured configuration
 * that can be loaded from ROS2 parameters and injected into modules.
 */

#pragma once

#include "vina_slam/core/constants.hpp"
#include <Eigen/Core>
#include <vector>
#include <string>

namespace vina_slam {
namespace core {

/**
 * @brief Voxel map configuration
 *
 * Controls octree subdivision, plane fitting thresholds,
 * and voxel grid parameters.
 */
struct VoxelMapConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Minimum points per layer for plane fitting
  Eigen::Vector4d min_point = Eigen::Vector4d(15, 15, 15, 15);

  /// Minimum eigenvalue for plane classification
  double min_eigen_value = 0.01;

  /// Maximum octree depth
  int max_layer = 2;

  /// Maximum points to keep in fixed storage per voxel
  int max_points = 100;

  /// Voxel grid edge length (meters)
  double voxel_size = 1.0;

  /// Minimum points for BA factor
  int min_ba_point = 20;

  /// Plane eigenvalue ratio threshold per layer
  std::vector<double> plane_eigen_value_thre = {0.01, 0.01, 0.01, 0.01};
};

/**
 * @brief IMU noise and weighting configuration
 */
struct ImuNoiseConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Gyroscope noise covariance
  double cov_gyr = 0.1;

  /// Accelerometer noise covariance
  double cov_acc = 0.1;

  /// Gyroscope random walk
  double rand_walk_gyr = 0.01;

  /// Accelerometer random walk
  double rand_walk_acc = 0.01;

  /// IMU factor weight coefficient in optimization
  double imu_coef = 1e-4;

  /// Gravity pre-integration scale
  double imupre_scale_gravity = 50.0;
};

/**
 * @brief Sensor error model parameters
 */
struct SensorErrorConfig {
  /// Range (depth) error standard deviation
  double dept_err = 0.0;

  /// Beam angle error standard deviation (degrees)
  double beam_err = 0.0;
};

/**
 * @brief LiDAR sensor configuration
 */
struct LidarConfig {
  /// LiDAR type (0=LIVOX, 1=VELODYNE, 2=OUSTER, 3=HESAI, 4=ROBOSENSE)
  int lidar_type = 0;

  /// Minimum range (blind zone, meters)
  double blind = 0.1;

  /// Point decimation factor
  int point_filter_num = 3;

  /// Whether points have valid timestamps
  int point_notime = 0;
};

/**
 * @brief Extrinsic calibration (LiDAR to IMU)
 */
struct ExtrinsicConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Translation vector
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  /// Rotation matrix
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

/**
 * @brief Pipeline control parameters
 */
struct PipelineConfig {
  /// Sliding window size
  int win_size = 10;

  /// Downsampling voxel size
  double down_size = 0.3;

  /// Enable bundle adjustment
  int if_BA = 0;

  /// Enable loop detection
  int if_loop_dect = 0;

  /// Save map on exit
  int is_save_map = 0;

  /// Number of threads for parallel processing
  int thread_num = 5;

  /// Degeneration detection bound
  int degrade_bound = 10;
};

/**
 * @brief Topic names
 */
struct TopicConfig {
  std::string lid_topic = "/rslidar_points";
  std::string imu_topic = "/imu";
};

/**
 * @brief Complete system configuration
 *
 * Aggregates all sub-configurations. Loaded from ROS2 parameters
 * in VinaSlamNode::loadConfig().
 */
struct SystemConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxelMapConfig voxel;
  ImuNoiseConfig imu;
  SensorErrorConfig sensor;
  LidarConfig lidar;
  ExtrinsicConfig extrinsic;
  PipelineConfig pipeline;
  TopicConfig topics;

  std::string bagname = "noNameBag";
  std::string savepath;
};

} // namespace core
} // namespace vina_slam
