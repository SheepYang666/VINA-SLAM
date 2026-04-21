#pragma once
#include "vina_slam/lidar_pointcloud_decoder.hpp"

template <class T>
void pcl_handler(const T& msg);

void pcl_handler_livox(const livox_ros_driver::CustomMsg::ConstPtr& msg);
void pcl_handler_standard(const sensor_msgs::PointCloud2::ConstPtr& msg);
