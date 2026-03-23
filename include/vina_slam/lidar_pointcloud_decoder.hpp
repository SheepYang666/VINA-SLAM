#pragma once

// #include
// </home/yang/local_lib/ws_livox_alias/install/livox_ros_driver/include/livox_ros_driver/livox_ros_driver/msg/custom_msg.hpp>

#include <Eigen/Core>
#include <cstdint>
#include <cstdio>
// #include <livox_ros_driver/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using PointType = pcl::PointXYZINormal;

enum LID_TYPE
{
  LIVOX,
  VELODYNE,
  OUSTER,
  HESAI,      ///< HesaiXT32
  ROBOSENSE,  ///< Robosense
  TARTANAIR   ///< TartanAir （No intensity / time）
};

struct LivoxPoint
{
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;  // datatype 8 -> float64
};

/// Decode helper functions (according to endian order)
template <typename T>
T decode_field(const uint8_t* data, bool is_bigendian);

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float time;
  ::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, time, time)(std::uint16_t, ring,
                                                                                             ring));

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
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
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t,
                                                                                                       t));

namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                       double, timestamp, timestamp)(std::uint16_t, ring, ring));

namespace rslidar_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp));

class LidarPointCloudDecoder
{
public:
  int lidar_type;
  int point_filter_num;
  double blind = 1.0;
  double omega_l = 3610.0;

  double process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);
  double process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);

private:
  void livox_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);
  // void livox_handler(const livox_ros_driver::msg::CustomMsg::SharedPtr &msg, pcl::PointCloud<PointType> &pl_full);

  void velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);

  void ouster_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);

  void hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);

  double robosense_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);

  void tartanair_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, pcl::PointCloud<PointType>& pl_full);
};
