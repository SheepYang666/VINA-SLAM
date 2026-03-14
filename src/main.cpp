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

namespace {

void resetLegacyRosHandles() {
  sub_imu.reset();
  sub_pcl.reset();

  pub_scan.reset();
  pub_cmap.reset();
  pub_init.reset();
  pub_pmap.reset();
  pub_test.reset();
  pub_prev_path.reset();
  pub_curr_path.reset();
  pub_pmap_livox.reset();
  pub_voxel_plane.reset();
  pub_voxel_normal.reset();
}

void resetSingletonRosHandles(vina_slam::platform::ros2::ResultPublisher& result_publisher,
                              vina_slam::platform::ros2::FileReaderWriter& file_reader_writer,
                              vina_slam::pipeline::Initialization& initialization) {
  result_publisher.tf_broadcaster.reset();
  result_publisher.pub_plane.reset();
  result_publisher.pub_path.reset();
  result_publisher.pub_pmap_livox.reset();
  result_publisher.pub_cmap.reset();
  result_publisher.pub_curr_path.reset();
  result_publisher.pub_pmap.reset();
  result_publisher.pub_scan.reset();
  result_publisher.pub_odom.reset();
  result_publisher.node.reset();

  file_reader_writer.node.reset();
  initialization.node.reset();
}

} // namespace

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
  pub_test = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);
  pub_voxel_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_plane", 10);
  pub_voxel_normal = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_normal", 10);
  pub_pmap_livox = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/map_pmap_livox", 10);

  // Initialize singletons
  auto& result_publisher = vina_slam::platform::ros2::ResultPublisher::instance(node);
  auto& file_reader_writer = vina_slam::platform::ros2::FileReaderWriter::instance(node);
  auto& initialization = vina_slam::pipeline::Initialization::instance(node);

  // Create SLAM system
  VINA_SLAM vs(node);

  // Initialize sliding window index mapping
  mp.resize(vs.win_size);
  for (size_t i = 0; i < mp.size(); i++) {
    mp[i] = i;
  }

  // Launch SLAM pipeline thread
  std::thread thread_odom(&VINA_SLAM::thd_odometry_localmapping, &vs, node);
  std::thread thread_watch([&exec, &thread_odom]() {
    if (thread_odom.joinable()) {
      thread_odom.join();
    }
    exec->cancel();
  });

  exec->spin();

  if (thread_watch.joinable()) {
    thread_watch.join();
  }

  resetLegacyRosHandles();
  resetSingletonRosHandles(result_publisher, file_reader_writer, initialization);

  if (rclcpp::ok()) {
    exec->cancel();
    exec->remove_node(node);
    rclcpp::shutdown();
  }

  return 0;
}
