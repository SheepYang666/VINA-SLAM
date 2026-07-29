#pragma once

#include <rclcpp/node.hpp>
#include <string>

namespace vina_slam
{

struct BagReaderOptions
{
  std::string bag_path;
  std::string lid_topic;
  std::string imu_topic;
  int lidar_type = 0;
  std::size_t max_imu_buf = 8000;
  std::size_t max_pcl_buf = 50;
  double drain_idle_timeout_sec = 5.0;
};

// Opens a rosbag2 directory, feeds imu_handler / pcl_handler for the configured topics,
// waits for buffers to drain (or idle timeout), then sets finish=true and shuts down ROS.
void run_bag_reader(const rclcpp::Node::SharedPtr& node, const BagReaderOptions& options);

}  // namespace vina_slam
