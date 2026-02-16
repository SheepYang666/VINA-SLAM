/**
 * @file lidar_decoder.hpp
 * @brief Multi-format LiDAR point cloud decoder
 *
 * This module provides a unified interface for decoding various LiDAR
 * point cloud formats including Livox, Velodyne, Ouster, Hesai, Robosense,
 * and TartanAir simulation data.
 */

#pragma once

#include "vina_slam/core/types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <cstdio>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace vina_slam {
namespace sensor {

// ============================================================================
// LiDAR Type Enumeration (using int for ROS2 parameter compatibility)
// ============================================================================
enum LidarType {
  LIVOX = 0,
  VELODYNE = 1,
  OUSTER = 2,
  HESAI = 3,
  ROBOSENSE = 4,
  TARTANAIR = 5  // Simulation (no intensity/time)
};

// ============================================================================
// Point Type Definitions for Various LiDARs
// ============================================================================
struct LivoxPoint {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};

} // namespace sensor
} // namespace vina_slam

// ============================================================================
// PCL Point Type Registrations
// ============================================================================
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float time;
  ::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, time, time)(std::uint16_t, ring, ring));

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(std::uint32_t, t, t));

namespace xt32_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)(std::uint16_t, ring, ring));

namespace rslidar_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)(double, timestamp, timestamp));

namespace vina_slam {
namespace sensor {

// ============================================================================
// LiDAR Point Cloud Decoder
// ============================================================================
/**
 * @brief Multi-format LiDAR point cloud decoder
 *
 * Supports decoding of various LiDAR formats:
 * - Livox (CustomMsg)
 * - Velodyne (PointCloud2)
 * - Ouster (PointCloud2)
 * - Hesai XT32 (PointCloud2)
 * - Robosense (PointCloud2)
 * - TartanAir simulation (PointCloud2)
 */
class LidarPointCloudDecoder {
public:
  int lidar_type;  // Use int for ROS2 parameter compatibility
  int point_filter_num = 1;
  double blind = 1.0;
  double omega_l = 3610.0;  // Velodyne rotation rate for time estimation

  LidarPointCloudDecoder() = default;

  /**
   * @brief Process Livox CustomMsg
   * @param msg Livox custom message
   * @param pl_full Output point cloud
   * @return Timestamp of the message
   */
  double process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                 pcl::PointCloud<PointType>& pl_full);

  /**
   * @brief Process ROS2 PointCloud2
   * @param msg ROS2 PointCloud2 message
   * @param pl_full Output point cloud
   * @return Timestamp of the message
   */
  double process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                 pcl::PointCloud<PointType>& pl_full);

private:
  void livox_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                     pcl::PointCloud<PointType>& pl_full);

  void velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                        pcl::PointCloud<PointType>& pl_full);

  void ouster_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                      pcl::PointCloud<PointType>& pl_full);

  void hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                     pcl::PointCloud<PointType>& pl_full);

  double robosense_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                           pcl::PointCloud<PointType>& pl_full);

  void tartanair_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                         pcl::PointCloud<PointType>& pl_full);
};

} // namespace sensor
} // namespace vina_slam

// Backward compatibility: bring types to global namespace
using vina_slam::sensor::LidarType;
using vina_slam::sensor::LidarPointCloudDecoder;
using vina_slam::sensor::LivoxPoint;

// Backward compatibility: LIVOX, VELODYNE, etc. are now in vina_slam::sensor::LidarType enum
using vina_slam::sensor::LIVOX;
using vina_slam::sensor::VELODYNE;
using vina_slam::sensor::OUSTER;
using vina_slam::sensor::HESAI;
using vina_slam::sensor::ROBOSENSE;
using vina_slam::sensor::TARTANAIR;
