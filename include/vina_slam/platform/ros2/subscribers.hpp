#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

// Global subscriber variables (moved from VINASlam.hpp)
extern rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
extern rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
extern rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard;

void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg_in);
