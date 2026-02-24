/**
 * @file imu_preintegration.cpp
 * @brief Implementation of IMU preintegration factor
 */

#include "vina_slam/estimation/imu_preintegration.hpp"

namespace vina_slam {
namespace estimation {

// Global noise parameters
double imupre_scale_gravity = 1.0;
Eigen::Matrix<double, 6, 6> noiseMeas = Eigen::Matrix<double, 6, 6>::Zero();
Eigen::Matrix<double, 6, 6> noiseWalk = Eigen::Matrix<double, 6, 6>::Zero();

ImuPreintegration::ImuPreintegration(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  bg_ = bg;
  ba_ = ba;

  R_delta_.setIdentity();
  p_delta_.setZero();
  v_delta_.setZero();

  R_bg_.setZero();
  p_bg_.setZero();
  p_ba_.setZero();
  v_bg_.setZero();
  v_ba_.setZero();

  dtime_ = 0;

  dbg.setZero();
  dba.setZero();
  dbg_buf.setZero();
  dba_buf.setZero();

  cov_.setZero();
}

void ImuPreintegration::pushImu(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_buffer) {
  imus_.insert(imus_.end(), imu_buffer.begin(), imu_buffer.end());
  Eigen::Vector3d cur_gyr, cur_acc;
  for (auto it_imu = imu_buffer.begin() + 1; it_imu != imu_buffer.end(); it_imu++) {
    sensor_msgs::msg::Imu& imu_prev = **(it_imu - 1);
    sensor_msgs::msg::Imu& imu_curr = **it_imu;

    double dt = rclcpp::Time(imu_curr.header.stamp).seconds() - rclcpp::Time(imu_prev.header.stamp).seconds();

    cur_gyr << 0.5 * (imu_prev.angular_velocity.x + imu_curr.angular_velocity.x),
        0.5 * (imu_prev.angular_velocity.y + imu_curr.angular_velocity.y),
        0.5 * (imu_prev.angular_velocity.z + imu_curr.angular_velocity.z);
    cur_acc << 0.5 * (imu_prev.linear_acceleration.x + imu_curr.linear_acceleration.x),
        0.5 * (imu_prev.linear_acceleration.y + imu_curr.linear_acceleration.y),
        0.5 * (imu_prev.linear_acceleration.z + imu_curr.linear_acceleration.z);

    cur_gyr = cur_gyr - bg_;
    cur_acc = cur_acc * imupre_scale_gravity - ba_;

    addImu(cur_gyr, cur_acc, dt);
  }
}

void ImuPreintegration::addImu(Eigen::Vector3d& cur_gyr, Eigen::Vector3d& cur_acc, double dt) {
  dtime_ += dt;

  Eigen::Matrix3d rotation_increment = Exp(cur_gyr, dt);
  Eigen::Matrix3d right_jacobian = jr(cur_gyr * dt);

  Eigen::Matrix3d rotation_dt = dt * R_delta_;
  Eigen::Matrix3d rotation_dt2_half = 0.5 * dt * dt * R_delta_;

  Eigen::Matrix3d acc_skew;
  acc_skew << SKEW_SYM_MATRX(cur_acc);

  p_ba_ = p_ba_ + v_ba_ * dt - rotation_dt2_half;
  p_bg_ = p_bg_ + v_bg_ * dt - rotation_dt2_half * acc_skew * R_bg_;
  v_ba_ = v_ba_ - rotation_dt;
  v_bg_ = v_bg_ - rotation_dt * acc_skew * R_bg_;
  R_bg_ = rotation_increment.transpose() * R_bg_ - right_jacobian * dt;

  Eigen::Matrix<double, 9, 9> jacobian_a = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 6> jacobian_b = Eigen::Matrix<double, 9, 6>::Zero();

  jacobian_a.block<3, 3>(0, 0) = rotation_increment.transpose();
  jacobian_a.block<3, 3>(3, 0) = -rotation_dt2_half * acc_skew;
  jacobian_a.block<3, 3>(3, 6) = core::I33 * dt;
  jacobian_a.block<3, 3>(6, 0) = -rotation_dt * acc_skew;

  jacobian_b.block<3, 3>(0, 0) = right_jacobian * dt;
  jacobian_b.block<3, 3>(3, 3) = rotation_dt2_half;
  jacobian_b.block<3, 3>(6, 3) = rotation_dt;

  cov_.block<9, 9>(0, 0) = jacobian_a * cov_.block<9, 9>(0, 0) * jacobian_a.transpose() +
                          jacobian_b * noiseMeas * jacobian_b.transpose();
  cov_.block<6, 6>(9, 9) += noiseWalk * dt;

  p_delta_ += v_delta_ * dt + rotation_dt2_half * cur_acc;
  v_delta_ += rotation_dt * cur_acc;
  R_delta_ = R_delta_ * rotation_increment;
}

double ImuPreintegration::evaluate(core::IMUST& st1, core::IMUST& st2,
                                   Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable) {
  Eigen::Matrix<double, DIM, DIM> joca, jocb;
  Eigen::Matrix<double, DIM, 1> rr;
  joca.setZero();
  jocb.setZero();
  rr.setZero();

  Eigen::Matrix3d R_correct = R_delta_ * Exp(R_bg_ * dbg);
  Eigen::Vector3d t_correct = p_delta_ + p_bg_ * dbg + p_ba_ * dba;
  Eigen::Vector3d v_correct = v_delta_ + v_bg_ * dbg + v_ba_ * dba;

  Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
  Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime_ * st1.g);
  Eigen::Vector3d res_v = exp_v - v_correct;
  Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v * dtime_ - 0.5 * dtime_ * dtime_ * st1.g);
  Eigen::Vector3d res_t = exp_t - t_correct;

  Eigen::Vector3d res_bg = st2.bg - st1.bg;
  Eigen::Vector3d res_ba = st2.ba - st1.ba;

  double b_wei = 1;

  rr.block<3, 1>(0, 0) = Log(res_r);
  rr.block<3, 1>(3, 0) = res_t;
  rr.block<3, 1>(6, 0) = res_v;
  rr.block<3, 1>(9, 0) = res_bg * b_wei;
  rr.block<3, 1>(12, 0) = res_ba * b_wei;

  Eigen::Matrix<double, 15, 15> cov_inv = cov_.inverse();

  // Check for NaN in covariance inverse (indicates numerical instability)
  if (!cov_inv.allFinite()) {
    cov_ = Eigen::Matrix<double, 15, 15>::Identity() * 1e-3;
    cov_inv = cov_.inverse();
  }

  if (jac_enable) {
    Eigen::Matrix3d JR_inv = jr_inv(res_r);

    joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
    jocb.block<3, 3>(0, 0) = JR_inv;
    joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * jr(R_bg_ * dbg) * R_bg_;

    joca.block<3, 3>(3, 0) = hat(exp_t);
    joca.block<3, 3>(3, 3) = -st1.R.transpose();
    joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime_;
    joca.block<3, 3>(3, 9) = -p_bg_;
    joca.block<3, 3>(3, 12) = -p_ba_;
    jocb.block<3, 3>(3, 3) = st1.R.transpose();

    joca.block<3, 3>(6, 0) = hat(exp_v);
    joca.block<3, 3>(6, 6) = -st1.R.transpose();
    joca.block<3, 3>(6, 9) = -v_bg_;
    joca.block<3, 3>(6, 12) = -v_ba_;
    jocb.block<3, 3>(6, 6) = st1.R.transpose();

    joca.block<3, 3>(9, 9) = -core::I33 * b_wei;
    joca.block<3, 3>(12, 12) = -core::I33 * b_wei;
    jocb.block<3, 3>(9, 9) = core::I33 * b_wei;
    jocb.block<3, 3>(12, 12) = core::I33 * b_wei;

    Eigen::Matrix<double, DIM, 2 * DIM> joc;
    joc.block<DIM, DIM>(0, 0) = joca;
    joc.block<DIM, DIM>(0, DIM) = jocb;

    jtj = joc.transpose() * cov_inv * joc;
    gg = joc.transpose() * cov_inv * rr;
  }

  return rr.dot(cov_inv * rr);
}

double ImuPreintegration::evaluateWithGravity(core::IMUST& st1, core::IMUST& st2,
                                              Eigen::MatrixXd& jtj, Eigen::VectorXd& gg, bool jac_enable) {
  Eigen::Matrix<double, DIM, DIM> joca, jocb;
  Eigen::Matrix<double, DIM, 1> rr;
  joca.setZero();
  jocb.setZero();
  rr.setZero();
  Eigen::Matrix<double, DIM, 3> jocg;
  jocg.setZero();

  Eigen::Matrix3d R_correct = R_delta_ * Exp(R_bg_ * dbg);
  Eigen::Vector3d t_correct = p_delta_ + p_bg_ * dbg + p_ba_ * dba;
  Eigen::Vector3d v_correct = v_delta_ + v_bg_ * dbg + v_ba_ * dba;

  Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
  Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime_ * st1.g);
  Eigen::Vector3d res_v = exp_v - v_correct;
  Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v * dtime_ - 0.5 * dtime_ * dtime_ * st1.g);
  Eigen::Vector3d res_t = exp_t - t_correct;

  Eigen::Vector3d res_bg = st2.bg - st1.bg;
  Eigen::Vector3d res_ba = st2.ba - st1.ba;

  double b_wei = 1;

  rr.block<3, 1>(0, 0) = Log(res_r);
  rr.block<3, 1>(3, 0) = res_t;
  rr.block<3, 1>(6, 0) = res_v;
  rr.block<3, 1>(9, 0) = res_bg * b_wei;
  rr.block<3, 1>(12, 0) = res_ba * b_wei;

  Eigen::Matrix<double, 15, 15> cov_inv = cov_.inverse();

  // Check for NaN in covariance inverse (indicates numerical instability)
  if (!cov_inv.allFinite()) {
    cov_ = Eigen::Matrix<double, 15, 15>::Identity() * 1e-3;
    cov_inv = cov_.inverse();
  }

  if (jac_enable) {
    Eigen::Matrix3d JR_inv = jr_inv(res_r);

    joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
    jocb.block<3, 3>(0, 0) = JR_inv;
    joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * jr(R_bg_ * dbg) * R_bg_;

    joca.block<3, 3>(3, 0) = hat(exp_t);
    joca.block<3, 3>(3, 3) = -st1.R.transpose();
    joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime_;
    joca.block<3, 3>(3, 9) = -p_bg_;
    joca.block<3, 3>(3, 12) = -p_ba_;
    jocb.block<3, 3>(3, 3) = st1.R.transpose();

    joca.block<3, 3>(6, 0) = hat(exp_v);
    joca.block<3, 3>(6, 6) = -st1.R.transpose();
    joca.block<3, 3>(6, 9) = -v_bg_;
    joca.block<3, 3>(6, 12) = -v_ba_;
    jocb.block<3, 3>(6, 6) = st1.R.transpose();

    joca.block<3, 3>(9, 9) = -core::I33 * b_wei;
    joca.block<3, 3>(12, 12) = -core::I33 * b_wei;
    jocb.block<3, 3>(9, 9) = core::I33 * b_wei;
    jocb.block<3, 3>(12, 12) = core::I33 * b_wei;

    jocg.block<3, 3>(3, 0) = st1.R.transpose() * (-0.5 * dtime_ * dtime_);
    jocg.block<3, 3>(6, 0) = st1.R.transpose() * (-dtime_);

    Eigen::Matrix<double, DIM, 2 * DIM + 3> joc;
    joc.block<DIM, DIM>(0, 0) = joca;
    joc.block<DIM, DIM>(0, DIM) = jocb;
    joc.block<DIM, 3>(0, 2 * DIM) = jocg;

    jtj = joc.transpose() * cov_inv * joc;
    gg = joc.transpose() * cov_inv * rr;
  }

  return rr.dot(cov_inv * rr);
}

void ImuPreintegration::updateState(const Eigen::Matrix<double, DIM, 1>& dxi) {
  dbg_buf = dbg;
  dba_buf = dba;

  dbg += dxi.block<3, 1>(9, 0);
  dba += dxi.block<3, 1>(12, 0);
}

void ImuPreintegration::merge(ImuPreintegration& imu2) {
  p_bg_ += v_bg_ * imu2.dtime_ + R_delta_ * (imu2.p_bg_ - hat(imu2.p_delta_) * R_bg_);
  p_ba_ += v_ba_ * imu2.dtime_ + R_delta_ * imu2.p_ba_;
  v_bg_ += R_delta_ * (imu2.v_bg_ - hat(imu2.v_delta_) * R_bg_);
  v_ba_ += R_delta_ * imu2.v_ba_;
  R_bg_ = imu2.R_delta_.transpose() * R_bg_ + imu2.R_bg_;

  Eigen::Matrix<double, DIM, DIM> Ai, Bi;
  Ai.setIdentity();
  Bi.setIdentity();
  Ai.block<3, 3>(0, 0) = imu2.R_delta_.transpose();
  Ai.block<3, 3>(3, 0) = -R_delta_ * hat(imu2.p_delta_);
  Ai.block<3, 3>(3, 6) = core::I33 * imu2.dtime_;
  Ai.block<3, 3>(6, 0) = -R_delta_ * hat(imu2.v_delta_);

  Bi.block<3, 3>(3, 3) = R_delta_;
  Bi.block<3, 3>(6, 6) = R_delta_;
  cov_ = Ai * cov_ * Ai.transpose() + Bi * imu2.cov_ * Bi.transpose();

  p_delta_ += v_delta_ * imu2.dtime_ + R_delta_ * imu2.p_delta_;
  v_delta_ += R_delta_ * imu2.v_delta_;
  R_delta_ = R_delta_ * imu2.R_delta_;

  dtime_ += imu2.dtime_;
}

} // namespace estimation
} // namespace vina_slam
