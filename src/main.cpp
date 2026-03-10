/**
 * @file main.cpp
 * @brief Entry point for VINA-SLAM node
 *
 * Creates the ROS2 node, initializes publishers/singletons,
 * and launches the SLAM pipeline thread.
 */

#include "vina_slam/VINASlam.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("vina_slam");
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  exec->add_node(node);

  // Create global publishers (legacy — to be migrated to ResultPublisher)
  pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cmap", 100);
  pub_pmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pmap", 100);
  pub_prev_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_true", 100);
  pub_init = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_init", 100);

  pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan", 100);
  pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_path", 100);
  pub_test = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);
  pub_voxel_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_plane", 10);
  pub_voxel_normal = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_normal", 10);
  pub_pmap_livox = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/map_pmap_livox", 10);

  // Initialize singletons
  vina_slam::platform::ros2::ResultPublisher::instance(node);
  vina_slam::platform::ros2::FileReaderWriter::instance(node);
  vina_slam::pipeline::Initialization::instance(node);

  // Create SLAM system
  VINA_SLAM vs(node);

  // Initialize sliding window index mapping
  mp.resize(vs.win_size);
  for (size_t i = 0; i < mp.size(); i++) {
    mp[i] = i;
  }

  // Launch SLAM pipeline thread
  std::thread thread_odom(&VINA_SLAM::thd_odometry_localmapping, &vs, node);

  exec->spin();
  thread_odom.join();

  return 0;
}
