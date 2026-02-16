/**
 * @file subscribers.cpp
 * @brief Implementation of ROS2 subscribers
 */

#include "vina_slam/platform/ros2/subscribers.hpp"

namespace vina_slam {
namespace platform {
namespace ros2 {

SensorSubscribers::SensorSubscribers(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {}

void SensorSubscribers::setupImuSubscriber(const std::string& topic) {
  rclcpp::QoS imu_qos(8000);
  imu_qos.keep_last(8000);
  imu_qos.best_effort();

  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      topic, imu_qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuHandler(msg); });
}

void SensorSubscribers::setupLidarSubscriber(const std::string& topic, int type) {
  lidar_type = type;

  rclcpp::QoS pcl_qos(1000);
  pcl_qos.keep_last(1000);
  pcl_qos.best_effort();

  if (type == sensor::LIVOX) {
    sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        topic, rclcpp::SensorDataQoS(),
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livoxHandler(msg); });
  } else {
    sub_pcl_pc2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, pcl_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pointCloud2Handler(msg); });
  }
}

void SensorSubscribers::imuHandler(const sensor_msgs::msg::Imu::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(buf_mutex);
  imu_last_time = rclcpp::Time(msg->header.stamp).seconds();
  imu_buf.push_back(msg);
}

void SensorSubscribers::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(buf_mutex);
  pcl_buf_livox.push_back(msg);
  time_buf.push_back(rclcpp::Time(msg->header.stamp).seconds());
}

void SensorSubscribers::pointCloud2Handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(buf_mutex);
  pcl_buf_pc2.push_back(msg);
  time_buf.push_back(rclcpp::Time(msg->header.stamp).seconds());
}

bool SensorSubscribers::syncPackages(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imus,
                                     livox_ros_driver2::msg::CustomMsg::SharedPtr& pcl,
                                     double& pcl_time) {
  std::lock_guard<std::mutex> lock(buf_mutex);

  if (pcl_buf_livox.empty() || imu_buf.empty()) {
    return false;
  }

  // Get point cloud time range
  pcl = pcl_buf_livox.front();
  double pcl_beg_time = rclcpp::Time(pcl->header.stamp).seconds();
  double pcl_end_time = pcl_beg_time + pcl->points.back().offset_time / 1e9;
  pcl_time = pcl_beg_time;

  // Find IMU measurements within point cloud time range
  imus.clear();
  while (!imu_buf.empty()) {
    double imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
    if (imu_time < pcl_beg_time) {
      imus.push_back(imu_buf.front());
      imu_buf.pop_front();
    } else if (imu_time <= pcl_end_time) {
      imus.push_back(imu_buf.front());
      imu_buf.pop_front();
    } else {
      break;
    }
  }

  // Need at least one IMU measurement before and after
  if (imus.empty()) {
    return false;
  }

  // Pop the used point cloud
  pcl_buf_livox.pop_front();
  if (!time_buf.empty()) {
    time_buf.pop_front();
  }

  return true;
}

bool SensorSubscribers::hasData() const {
  return !pcl_buf_livox.empty() || !pcl_buf_pc2.empty();
}

} // namespace ros2
} // namespace platform
} // namespace vina_slam
