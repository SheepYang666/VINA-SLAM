#pragma once

#include "vina_slam/tools.hpp"
#include <deque>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

extern double imupre_scale_gravity;
extern Eigen::Matrix<double, 6, 6> noiseMeas; // IMU measurement noise
extern Eigen::Matrix<double, 6, 6> noiseWalk; // IMU random walk noise

class IMU_PRE
{
  private:
    Eigen::Matrix3d R_delta;
    Eigen::Vector3d p_delta, v_delta;
    Eigen::Vector3d bg, ba;

    Eigen::Matrix3d R_bg;
    Eigen::Matrix3d p_bg, p_ba;
    Eigen::Matrix3d v_bg, v_ba;

    double dtime;

    Eigen::Matrix<double, DIM, DIM> cov;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::Vector3d dbg, dba;
    Eigen::Vector3d dbg_buf, dba_buf;

    // queue for raw IMU data
    deque<sensor_msgs::msg::Imu::SharedPtr> _imus;

  private:
    void add_imu(Eigen::Vector3d &cur_gyr, Eigen::Vector3d &cur_acc, double dt);

  public:
    IMU_PRE(const Eigen::Vector3d &bg1 = Eigen::Vector3d::Zero(), const Eigen::Vector3d &ba1 = Eigen::Vector3d::Zero());

    void push_imu(deque<sensor_msgs::msg::Imu::SharedPtr> &imu_buffer);

    double give_evaluate(IMUST &st1, IMUST &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable);

    double give_evaluate_g(IMUST &st1, IMUST &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable);

    void update_state(const Eigen::Matrix<double, DIM, 1> &dxi);

    void merge(IMU_PRE &imu2);
};
