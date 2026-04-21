#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

extern ros::Subscriber sub_imu;
extern ros::Subscriber sub_pcl_livox;
extern ros::Subscriber sub_pcl_standard;

void imu_handler(const sensor_msgs::Imu::ConstPtr& msg_in);
