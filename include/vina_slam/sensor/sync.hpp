#pragma once

#include "vina_slam/ekf_imu.hpp"
#include "vina_slam/lidar_pointcloud_decoder.hpp"
#include <deque>
#include <mutex>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;

// Global sync buffers (moved from VINASlam.hpp)
extern mutex mBuf;
extern LidarPointCloudDecoder feat;
extern deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
extern deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
extern deque<double> time_buf;

extern double imu_last_time;
extern int point_notime;
extern double last_pcl_time;

extern mutex pcl_time_lock;
extern double pcl_time;

bool sync_packages(pcl::PointCloud<PointType>::Ptr& pl_ptr, deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                   IMUEKF& p_imu);
