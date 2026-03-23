#pragma once

#include "vina_slam/tools.hpp"
#include <deque>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUEKF
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool init_flag;
    double pcl_beg_time, pcl_end_time, last_pcl_end_time;
    int init_num;
    Eigen::Vector3d mean_acc, mean_gyr;
    std::shared_ptr<sensor_msgs::msg::Imu> last_imu;

    int min_init_num = 30;
    Eigen::Vector3d angvel_last, acc_s_last;

    Eigen::Vector3d cov_acc, cov_gyr;
    Eigen::Vector3d cov_bias_gyr, cov_bias_acc;

    Eigen::Matrix3d Lid_rot_to_IMU;
    Eigen::Vector3d Lid_offset_to_IMU;

    double scale_gravity = 1.0;
    vector<IMUST> imu_poses;
    string imu_topic = "";

    int point_notime = 0;

  public:
    IMUEKF();

    void motion_blur(IMUST &xc, pcl::PointCloud<PointType> &pcl_in,
                     deque<std::shared_ptr<sensor_msgs::msg::Imu>> &imus);

    void IMU_init(deque<std::shared_ptr<sensor_msgs::msg::Imu>> &imus);

    int process(IMUST &x_curr, pcl::PointCloud<PointType> &pcl_in, deque<std::shared_ptr<sensor_msgs::msg::Imu>> &imus);
};
