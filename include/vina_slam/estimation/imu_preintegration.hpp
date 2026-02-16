/**
 * @file imu_preintegration.hpp
 * @brief IMU preintegration factor for bundle adjustment
 *
 * This module implements IMU preintegration following the
 * Forster et al. (2015) formulation for use in sliding window
 * bundle adjustment.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <deque>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vina_slam {
namespace estimation {

// Global noise parameters (TODO: move to config in future refactor)
extern double imupre_scale_gravity;
extern Eigen::Matrix<double, 6, 6> noiseMeas;  // IMU measurement noise
extern Eigen::Matrix<double, 6, 6> noiseWalk;  // IMU random walk noise

/**
 * @brief IMU preintegration factor
 *
 * Accumulates IMU measurements between two keyframes and provides
 * residual and Jacobian computation for optimization.
 *
 * State: [R, p, v, bg, ba] (15 DOF)
 */
class ImuPreintegration {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Delta values (integrated quantities)
  Eigen::Matrix3d R_delta_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_delta_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_delta_ = Eigen::Vector3d::Zero();

  // Bias estimates
  Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();

  // Jacobians w.r.t. biases
  Eigen::Matrix3d R_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d p_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d p_ba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d v_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d v_ba_ = Eigen::Matrix3d::Zero();

  // Time integration
  double dtime_ = 0.0;

  // Covariance
  Eigen::Matrix<double, DIM, DIM> cov_ = Eigen::Matrix<double, DIM, DIM>::Zero();

  // Bias updates (for state update) - public for backward compatibility
  Eigen::Vector3d dbg = Eigen::Vector3d::Zero();
  Eigen::Vector3d dba = Eigen::Vector3d::Zero();
  Eigen::Vector3d dbg_buf = Eigen::Vector3d::Zero();
  Eigen::Vector3d dba_buf = Eigen::Vector3d::Zero();

  // Raw IMU data storage
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imus_;

  /**
   * @brief Constructor with initial biases
   * @param bg Initial gyroscope bias
   * @param ba Initial accelerometer bias
   */
  ImuPreintegration(const Eigen::Vector3d& bg = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& ba = Eigen::Vector3d::Zero());

  /**
   * @brief Push IMU measurements from buffer
   * @param imu_buffer Buffer of IMU measurements
   */
  void pushImu(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_buffer);

  /**
   * @brief Evaluate residual without gravity optimization
   * @param st1 Start state
   * @param st2 End state
   * @param jtj Output Hessian (if jac_enable)
   * @param gg Output gradient (if jac_enable)
   * @param jac_enable Whether to compute Jacobians
   * @return Residual cost
   */
  double evaluate(core::IMUST& st1, core::IMUST& st2,
                  Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable);

  /**
   * @brief Evaluate residual with gravity optimization
   * @param st1 Start state
   * @param st2 End state
   * @param jtj Output Hessian (if jac_enable)
   * @param gg Output gradient (if jac_enable)
   * @param jac_enable Whether to compute Jacobians
   * @return Residual cost
   */
  double evaluateWithGravity(core::IMUST& st1, core::IMUST& st2,
                             Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable);

  /**
   * @brief Update state with error state
   * @param dxi Error state vector (15x1)
   */
  void updateState(const Eigen::Matrix<double, DIM, 1>& dxi);

  /**
   * @brief Merge another preintegration factor
   * @param other Another preintegration factor to merge
   */
  void merge(ImuPreintegration& other);

  // ========================================================================
  // Backward compatibility methods (snake_case to camelCase aliases)
  // ========================================================================
  void push_imu(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_buffer) { pushImu(imu_buffer); }
  double give_evaluate(core::IMUST& st1, core::IMUST& st2,
                       Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable) {
    return evaluate(st1, st2, jtj, gg, jac_enable);
  }
  double give_evaluate_g(core::IMUST& st1, core::IMUST& st2,
                         Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable) {
    return evaluateWithGravity(st1, st2, jtj, gg, jac_enable);
  }
  void update_state(const Eigen::Matrix<double, DIM, 1>& dxi) { updateState(dxi); }

private:
  /**
   * @brief Add single IMU measurement
   * @param cur_gyr Gyroscope measurement
   * @param cur_acc Accelerometer measurement
   * @param dt Time step
   */
  void addImu(Eigen::Vector3d& cur_gyr, Eigen::Vector3d& cur_acc, double dt);
};

} // namespace estimation
} // namespace vina_slam

// Backward compatibility
using vina_slam::estimation::ImuPreintegration;
using IMU_PRE = vina_slam::estimation::ImuPreintegration;
using vina_slam::estimation::imupre_scale_gravity;
using vina_slam::estimation::noiseMeas;
using vina_slam::estimation::noiseWalk;
