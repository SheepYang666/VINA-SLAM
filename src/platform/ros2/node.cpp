/**
 * @file node.cpp
 * @brief Implementation of ROS2 node wrapper
 */

#include "vina_slam/platform/ros2/node.hpp"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

namespace vina_slam {
namespace platform {
namespace ros2 {

void VinaSlamConfig::print() const {
  std::cout << "VINA-SLAM Configuration:" << std::endl;
  std::cout << "  LiDAR topic: " << lid_topic << std::endl;
  std::cout << "  IMU topic: " << imu_topic << std::endl;
  std::cout << "  LiDAR type: " << lidar_type << std::endl;
  std::cout << "  Voxel size: " << voxel_size << std::endl;
  std::cout << "  Window size: " << win_size << std::endl;
  std::cout << "  Localization mode: " << (localization_mode ? "true" : "false") << std::endl;
}

VinaSlamNode::VinaSlamNode(const rclcpp::NodeOptions& /*options*/) {
  // Node will be set via instance()
}

VinaSlamNode& VinaSlamNode::instance(const rclcpp::Node::SharedPtr& node_in) {
  static VinaSlamNode inst;
  inst.node = node_in;
  if (node_in) {
    inst.config = inst.loadConfig();
  }
  return inst;
}

VinaSlamNode& VinaSlamNode::instance() {
  return instance(rclcpp::Node::SharedPtr());
}

VinaSlamConfig VinaSlamNode::loadConfig() {
  VinaSlamConfig cfg;

  // General parameters
  cfg.bagname = node->declare_parameter("General.bagname", "noNameBag");
  node->get_parameter("General.bagname", cfg.bagname);

  cfg.save_path = node->declare_parameter("General.save_path", "");
  node->get_parameter("General.save_path", cfg.save_path);

  cfg.lid_topic = node->declare_parameter("General.lid_topic", "/rslidar_points");
  node->get_parameter("General.lid_topic", cfg.lid_topic);

  cfg.imu_topic = node->declare_parameter("General.imu_topic", "/imu");
  node->get_parameter("General.imu_topic", cfg.imu_topic);

  cfg.lidar_type = node->declare_parameter("General.lidar_type", 0);
  node->get_parameter("General.lidar_type", cfg.lidar_type);

  cfg.blind = node->declare_parameter("General.blind", 0.1);
  node->get_parameter("General.blind", cfg.blind);

  cfg.point_filter_num = node->declare_parameter("General.point_filter_num", 3);
  node->get_parameter("General.point_filter_num", cfg.point_filter_num);

  // Extrinsics
  std::vector<double> vecT = node->declare_parameter("General.extrinsic_tran", std::vector<double>(3, 0.0));
  node->get_parameter("General.extrinsic_tran", vecT);
  cfg.extrinsic_T << vecT[0], vecT[1], vecT[2];

  std::vector<double> vecR = node->declare_parameter("General.extrinsic_rota", std::vector<double>(9, 0.0));
  node->get_parameter("General.extrinsic_rota", vecR);
  cfg.extrinsic_R << vecR[0], vecR[1], vecR[2], vecR[3], vecR[4], vecR[5], vecR[6], vecR[7], vecR[8];

  cfg.is_save_map = node->declare_parameter<int>("General.is_save_map", 0);
  node->get_parameter("General.is_save_map", cfg.is_save_map);

  cfg.if_loop_dect = node->declare_parameter<int>("General.if_loop_dect", 0);
  node->get_parameter("General.if_loop_dect", cfg.if_loop_dect);

  cfg.if_BA = node->declare_parameter<int>("General.if_BA", 0);
  node->get_parameter("General.if_BA", cfg.if_BA);

  // Odometry parameters
  cfg.cov_gyr = node->declare_parameter<double>("Odometry.cov_gyr", 0.1);
  node->get_parameter("Odometry.cov_gyr", cfg.cov_gyr);

  cfg.cov_acc = node->declare_parameter<double>("Odometry.cov_acc", 0.1);
  node->get_parameter("Odometry.cov_acc", cfg.cov_acc);

  cfg.rand_walk_gyr = node->declare_parameter<double>("Odometry.rand_walk_gyr", 0.1);
  node->get_parameter("Odometry.rand_walk_gyr", cfg.rand_walk_gyr);

  cfg.rand_walk_acc = node->declare_parameter<double>("Odometry.rand_walk_acc", 0.1);
  node->get_parameter("Odometry.rand_walk_acc", cfg.rand_walk_acc);

  // Mapping parameters
  cfg.voxel_size = node->declare_parameter<double>("Mapping.voxel_size", 1.0);
  node->get_parameter("Mapping.voxel_size", cfg.voxel_size);

  cfg.max_layer = node->declare_parameter<int>("Mapping.max_layer", 2);
  node->get_parameter("Mapping.max_layer", cfg.max_layer);

  cfg.win_size = node->declare_parameter<int>("Mapping.win_size", 20);
  node->get_parameter("Mapping.win_size", cfg.win_size);

  // Localization mode
  cfg.localization_mode = node->declare_parameter<bool>("Localization.mode", false);
  node->get_parameter("Localization.mode", cfg.localization_mode);

  cfg.map_path = node->declare_parameter("Localization.map_path", "");
  node->get_parameter("Localization.map_path", cfg.map_path);

  return cfg;
}

core::IMUST VinaSlamNode::getExtrinsics() const {
  core::IMUST extr;
  extr.R = config.extrinsic_R;
  extr.p = config.extrinsic_T;
  return extr;
}

} // namespace ros2
} // namespace platform
} // namespace vina_slam
