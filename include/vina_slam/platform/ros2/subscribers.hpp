/**
 * @file subscribers.hpp
 * @brief ROS2 subscribers for VINA-SLAM input
 *
 * Handles:
 * - IMU data subscription
 * - LiDAR point cloud subscription
 * - Data synchronization
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/sensor/imu_buffer.hpp"
#include "vina_slam/sensor/lidar_decoder.hpp"
#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vina_slam {
namespace platform {
namespace ros2 {

/**
 * @brief Sensor subscribers manager
 *
 * Manages IMU and LiDAR subscribers with thread-safe buffers.
 */
class SensorSubscribers {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// IMU subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

  /// LiDAR subscriber (Livox format)
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;

  /// LiDAR subscriber (PointCloud2 format)
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc2;

  /// IMU buffer
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buf;

  /// Point cloud buffer
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> pcl_buf_livox;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pcl_buf_pc2;

  /// Time buffer
  std::deque<double> time_buf;

  /// Buffer mutex
  std::mutex buf_mutex;

  /// Last IMU time
  double imu_last_time = 0.0;

  /// LiDAR type
  int lidar_type = 0;

  /**
   * @brief Constructor
   * @param node_in ROS node pointer
   */
  explicit SensorSubscribers(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Setup IMU subscriber
   * @param topic IMU topic name
   */
  void setupImuSubscriber(const std::string& topic);

  /**
   * @brief Setup LiDAR subscriber
   * @param topic LiDAR topic name
   * @param type LiDAR type (LIVOX, VELODYNE, etc.)
   */
  void setupLidarSubscriber(const std::string& topic, int type);

  /**
   * @brief IMU message handler
   * @param msg IMU message
   */
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr& msg);

  /**
   * @brief Livox point cloud handler
   * @param msg Livox custom message
   */
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

  /**
   * @brief PointCloud2 handler
   * @param msg PointCloud2 message
   */
  void pointCloud2Handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  /**
   * @brief Synchronize IMU and LiDAR data
   * @param imus Output IMU measurements
   * @param pcl Output point cloud
   * @param pcl_time Output point cloud time
   * @return true if synchronized data available
   */
  bool syncPackages(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imus,
                    livox_ros_driver2::msg::CustomMsg::SharedPtr& pcl,
                    double& pcl_time);

  /**
   * @brief Check if buffer has data
   * @return true if data available
   */
  bool hasData() const;

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg) { imuHandler(msg); }
  void pcl_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) { livoxHandler(msg); }
  bool sync_packages(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imus,
                     livox_ros_driver2::msg::CustomMsg::SharedPtr& pcl,
                     double& pcl_time) {
    return syncPackages(imus, pcl, pcl_time);
  }
};

} // namespace ros2
} // namespace platform
} // namespace vina_slam

// Backward compatibility
using vina_slam::platform::ros2::SensorSubscribers;
