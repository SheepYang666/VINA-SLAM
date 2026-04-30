#pragma once

#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/preintegration.hpp"
#include <deque>
#include <vector>

extern double imu_coef;
#define DVEL 6

// The LM optimizer for LiDAR BA in HBA
class Lidar_BA_Optimizer
{
public:
  int win_size;     // Optimize window size
  int jac_leng;     // Optimize variable dimensions (usually win_size x 6, rotation + translation)
  int thd_num = 4;  // Number of threads used (default 4)

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess);
  bool damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd* hess, vector<double>& resis,
                    int max_iter = 3, bool is_display = false);
};

// The LiDAR-Inertial BA optimizer
class LI_BA_Optimizer
{
public:
  int win_size;
  int jac_leng;
  int imu_leng;

  // debug
  void print_breakdown(const char* tag, std::vector<IMUST>& xs, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus, double& imu_res, double& lidar_res, double& normal_res,
                       double& total_res) const;
  // debug

  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double divide_thread(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor);

  double only_residual(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus_factor);

  void damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd* hess);

  void damping_iter(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                    std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd* hess);
};

// The LiDAR-Inertial BA optimizer with gravity optimization
class LI_BA_OptimizerGravity
{
public:
  int win_size, jac_leng, imu_leng;

  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor);

  void damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor, vector<double>& resis,
                    Eigen::MatrixXd* hess, int max_iter = 2);
};
