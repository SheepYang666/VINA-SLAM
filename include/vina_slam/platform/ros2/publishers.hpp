/**
 * @file publishers.hpp
 * @brief ROS2 publishers for VINA-SLAM output
 *
 * Handles:
 * - Odometry publishing
 * - Point cloud publishing
 * - TF broadcasting
 * - Trajectory visualization
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

// Forward declarations
namespace vina_slam {
namespace mapping {
class Keyframe;
}
} // namespace vina_slam

namespace vina_slam {
namespace platform {
namespace ros2 {

/**
 * @brief Result publisher for VINA-SLAM output
 *
 * Manages all ROS2 publishers for visualization and output.
 */
class ResultPublisher {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

  /// Point cloud publisher (for local map)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;

  /// Global map publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pmap;

  /// Current path publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;

  /// Local map publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;

  /// Livox format map publisher
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_pmap_livox;

  /// Path publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_path;

  /// Plane marker publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_plane;

  /// TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  /// Point cloud end time (for timestamp)
  double pcl_end_time = 0.0;

  /// Mutex for time access
  std::mutex pcl_time_lock;

  /**
   * @brief Constructor with node
   * @param node_in ROS node pointer
   */
  explicit ResultPublisher(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance
   */
  static ResultPublisher& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton (must be initialized first)
   */
  static ResultPublisher& instance();

  /**
   * @brief Publish odometry and TF
   * @param xc Current state
   */
  void publishOdometry(const core::IMUST& xc);

  /**
   * @brief Publish point cloud
   * @param pl Point cloud to publish
   * @param publisher_type Publisher type (0=scan, 1=pmap)
   */
  void publishPointCloud(const pcl::PointCloud<core::PointType>& pl, int publisher_type);

  /**
   * @brief Publish point cloud to specific publisher
   * @param pl Point cloud to publish
   * @param pub Publisher to use
   */
  void publishPointCloudTo(const pcl::PointCloud<core::PointType>& pl,
                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);

  /**
   * @brief Publish Livox format point cloud
   * @param pl Point cloud to publish
   */
  void publishPmapLivox(const pcl::PointCloud<core::PointType>& pl);

  /**
   * @brief Publish local trajectory
   * @param pwld World points for visualization
   * @param jour Journey distance
   * @param x_curr Current state
   * @param cur_session Current session ID
   * @param pcl_path Path point cloud
   */
  void publishLocalTrajectory(PLV(3)& pwld, double jour, const core::IMUST& x_curr,
                              int cur_session, pcl::PointCloud<core::PointType>& pcl_path);

  /**
   * @brief Publish local map (voxel-based)
   * @param mgsize Marginalization size
   * @param surf_map Surface voxel map
   * @param x_buf State buffer
   */
  template<typename OctoTreePtr>
  void publishLocalMap(int mgsize,
                       std::unordered_map<core::VOXEL_LOC, OctoTreePtr>& surf_map,
                       std::vector<core::IMUST>& x_buf);

  /**
   * @brief Publish local map (point cloud buffer based)
   * @param mgsize Marginalization size
   * @param cur_session Current session ID
   * @param pvec_buf Point vector buffer
   * @param x_buf State buffer
   * @param pcl_path Path point cloud
   * @param win_base Window base index
   * @param win_count Window count
   */
  void publishLocalMapFull(int mgsize, int cur_session, std::vector<core::PVecPtr>& pvec_buf,
                           std::vector<core::IMUST>& x_buf, pcl::PointCloud<core::PointType>& pcl_path,
                           int win_base, int win_count);

  /**
   * @brief Publish global map from keyframes
   * @param relc_submaps Vector of keyframe submaps
   * @param ids IDs of submaps to publish
   * @param pub Publisher to use
   */
  void publishGlobalMap(std::vector<std::vector<mapping::Keyframe*>*>& relc_submaps,
                        std::vector<int>& ids,
                        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);

  /**
   * @brief Set point cloud end time
   * @param time End time in seconds
   */
  void setPclEndTime(double time);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void pub_odom_func(const core::IMUST& xc) { publishOdometry(xc); }
  void pub_pmap_livox_func(const pcl::PointCloud<core::PointType>& pl) { publishPmapLivox(pl); }
  void pub_pl_func(const pcl::PointCloud<core::PointType>& pl,
                   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
    publishPointCloudTo(pl, pub);
  }
  void pub_localtraj(PLV(3)& pwld, double jour, const core::IMUST& x_curr,
                     int cur_session, pcl::PointCloud<core::PointType>& pcl_path) {
    publishLocalTrajectory(pwld, jour, x_curr, cur_session, pcl_path);
  }
};

} // namespace ros2
} // namespace platform
} // namespace vina_slam
