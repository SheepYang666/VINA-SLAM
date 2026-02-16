/**
 * @file optimizers.hpp
 * @brief Bundle adjustment optimizers for LiDAR-Inertial SLAM
 *
 * Contains Levenberg-Marquardt based optimizers for:
 * - LiDAR-only bundle adjustment (Lidar_BA_Optimizer)
 * - LiDAR-Inertial bundle adjustment (LI_BA_Optimizer)
 * - LiDAR-Inertial with gravity optimization (LI_BA_OptimizerGravity)
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/core/constants.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include <deque>
#include <thread>
#include <vector>

namespace vina_slam {
namespace mapping {

// IMU coefficient for LiDAR-Inertial optimization weight
extern double imu_coef;

/// Velocity dimension in optimization (rotation + translation = 6)
#define DVEL 6

/**
 * @brief Levenberg-Marquardt optimizer for LiDAR-only bundle adjustment
 *
 * Optimizes poses using plane-to-plane constraints from LidarFactor.
 * Uses multi-threading for Hessian/Jacobian computation.
 */
class LidarBAOptimizer {
public:
  /// Window size (number of keyframes to optimize)
  int win_size;

  /// Jacobian length (win_size * 6)
  int jac_leng;

  /// Number of threads for parallel computation
  int thd_num = 4;

  /**
   * @brief Multi-threaded Hessian and Jacobian computation
   * @param x_stats Current state estimates (modified in place)
   * @param voxhess LidarFactor containing voxel constraints
   * @param Hess Output Hessian matrix
   * @param JacT Output gradient vector
   * @return Total residual cost
   */
  double divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  /**
   * @brief Compute residual only (for convergence check)
   * @param x_stats Current state estimates
   * @param voxhess LidarFactor containing voxel constraints
   * @return Total residual cost
   */
  double onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess);

  /**
   * @brief Run Levenberg-Marquardt damping iteration
   * @param x_stats State estimates (modified in place)
   * @param voxhess LidarFactor containing voxel constraints
   * @param hess Optional output Hessian for debugging
   * @param resis Vector of residuals per iteration
   * @param max_iter Maximum iterations
   * @param is_display Whether to print debug info
   * @return true if converged
   */
  bool dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                   Eigen::MatrixXd* hess, std::vector<double>& resis,
                   int max_iter = 3, bool is_display = false);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  double divide_thread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    return divideThread(x_stats, voxhess, Hess, JacT);
  }
  double only_residual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess) {
    return onlyResidual(x_stats, voxhess);
  }
  bool damping_iter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                    Eigen::MatrixXd* hess, std::vector<double>& resis,
                    int max_iter = 3, bool is_display = false) {
    return dampingIter(x_stats, voxhess, hess, resis, max_iter, is_display);
  }
};

/**
 * @brief LiDAR-Inertial bundle adjustment optimizer
 *
 * Combines LidarFactor, NormalFactor, and IMU preintegration
 * for tightly-coupled optimization.
 */
class LIBAOptimizer {
public:
  /// Window size (number of keyframes to optimize)
  int win_size;

  /// Jacobian length (win_size * 6)
  int jac_leng;

  /// IMU factor length (win_size - 1) * 15
  int imu_leng;

  /**
   * @brief Add Hessian and Jacobian contributions
   * @param Hess Main Hessian matrix
   * @param JacT Main gradient vector
   * @param hs Contribution Hessian
   * @param js Contribution gradient
   */
  void hessPlus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  /**
   * @brief Multi-threaded Hessian/Jacobian with IMU factors
   * @param x_stats Current state estimates
   * @param voxhess LidarFactor containing voxel constraints
   * @param imus_factor IMU preintegration factors
   * @param Hess Output Hessian matrix
   * @param JacT Output gradient vector
   * @return Total residual cost
   */
  double divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                      std::deque<estimation::ImuPreintegration*>& imus_factor,
                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  /**
   * @brief Multi-threaded with both LiDAR and normal factors
   */
  double divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                      NormalFactor& normal,
                      std::deque<estimation::ImuPreintegration*>& imus_factor,
                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  /**
   * @brief Compute residual with IMU factors
   */
  double onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                      std::deque<estimation::ImuPreintegration*>& imus_factor);

  /**
   * @brief Compute residual with all factors
   */
  double onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                      NormalFactor& normal,
                      std::deque<estimation::ImuPreintegration*>& imus_factor);

  /**
   * @brief Run damping iteration with IMU factors
   */
  void dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                   std::deque<estimation::ImuPreintegration*>& imus_factor,
                   Eigen::MatrixXd* hess);

  /**
   * @brief Run damping iteration with all factors
   */
  void dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                   NormalFactor& normal,
                   std::deque<estimation::ImuPreintegration*>& imus_factor,
                   Eigen::MatrixXd* hess);

  /**
   * @brief Print cost breakdown for debugging
   */
  void printBreakdown(const char* tag, std::vector<core::IMUST>& xs,
                      LidarFactor& lidar, NormalFactor& normal,
                      std::deque<estimation::ImuPreintegration*>& imus,
                      double& imu_res, double& lidar_res, double& normal_res,
                      double& total_res) const;

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                 Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
    hessPlus(Hess, JacT, hs, js);
  }
  double divide_thread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                       std::deque<estimation::ImuPreintegration*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    return divideThread(x_stats, voxhess, imus_factor, Hess, JacT);
  }
  double divide_thread(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                       NormalFactor& normal,
                       std::deque<estimation::ImuPreintegration*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    return divideThread(x_stats, lidar, normal, imus_factor, Hess, JacT);
  }
  double only_residual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                       std::deque<estimation::ImuPreintegration*>& imus_factor) {
    return onlyResidual(x_stats, voxhess, imus_factor);
  }
  double only_residual(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                       NormalFactor& normal,
                       std::deque<estimation::ImuPreintegration*>& imus_factor) {
    return onlyResidual(x_stats, lidar, normal, imus_factor);
  }
  void damping_iter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                    std::deque<estimation::ImuPreintegration*>& imus_factor,
                    Eigen::MatrixXd* hess) {
    dampingIter(x_stats, voxhess, imus_factor, hess);
  }
  void damping_iter(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                    NormalFactor& normal,
                    std::deque<estimation::ImuPreintegration*>& imus_factor,
                    Eigen::MatrixXd* hess) {
    dampingIter(x_stats, lidar, normal, imus_factor, hess);
  }
  void print_breakdown(const char* tag, std::vector<core::IMUST>& xs,
                       LidarFactor& lidar, NormalFactor& normal,
                       std::deque<estimation::ImuPreintegration*>& imus,
                       double& imu_res, double& lidar_res, double& normal_res,
                       double& total_res) const {
    printBreakdown(tag, xs, lidar, normal, imus, imu_res, lidar_res, normal_res, total_res);
  }
};

/**
 * @brief LiDAR-Inertial optimizer with gravity direction optimization
 *
 * Extends LI_BA_Optimizer to also optimize the gravity direction,
 * useful for initialization and scenarios with uncertain gravity.
 */
class LIBAOptimizerGravity {
public:
  /// Window size (number of keyframes to optimize)
  int win_size;

  /// Jacobian length
  int jac_leng;

  /// IMU factor length
  int imu_leng;

  /**
   * @brief Add Hessian and Jacobian contributions
   */
  void hessPlus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  /**
   * @brief Multi-threaded Hessian/Jacobian with IMU factors
   */
  double divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                      std::deque<estimation::ImuPreintegration*>& imus_factor,
                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  /**
   * @brief Compute residual with IMU factors
   */
  double onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                      std::deque<estimation::ImuPreintegration*>& imus_factor);

  /**
   * @brief Run damping iteration with gravity optimization
   */
  void dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                   std::deque<estimation::ImuPreintegration*>& imus_factor,
                   std::vector<double>& resis, Eigen::MatrixXd* hess,
                   int max_iter = 2);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                 Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
    hessPlus(Hess, JacT, hs, js);
  }
  double divide_thread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                       std::deque<estimation::ImuPreintegration*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    return divideThread(x_stats, voxhess, imus_factor, Hess, JacT);
  }
  double only_residual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                       std::deque<estimation::ImuPreintegration*>& imus_factor) {
    return onlyResidual(x_stats, voxhess, imus_factor);
  }
  void damping_iter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                    std::deque<estimation::ImuPreintegration*>& imus_factor,
                    std::vector<double>& resis, Eigen::MatrixXd* hess,
                    int max_iter = 2) {
    dampingIter(x_stats, voxhess, imus_factor, resis, hess, max_iter);
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility - using old class names
using Lidar_BA_Optimizer = vina_slam::mapping::LidarBAOptimizer;
using LI_BA_Optimizer = vina_slam::mapping::LIBAOptimizer;
using LI_BA_OptimizerGravity = vina_slam::mapping::LIBAOptimizerGravity;
