/**
 * @file lidar_decoder.cpp
 * @brief Implementation of multi-format LiDAR point cloud decoder
 */

#include "vina_slam/sensor/lidar_decoder.hpp"
#include <algorithm>

namespace vina_slam {
namespace sensor {

// Helper function to decode fields considering endianness
template <typename T>
T decode_field(const uint8_t* data, bool is_bigendian) {
  T value;
  if (is_bigendian) {
    uint8_t reversed[sizeof(T)];
    std::reverse_copy(data, data + sizeof(T), reversed);
    std::memcpy(&value, reversed, sizeof(T));
  } else {
    std::memcpy(&value, data, sizeof(T));
  }
  return value;
}

double LidarPointCloudDecoder::process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                                       pcl::PointCloud<PointType>& pl_full) {
  livox_handler(msg, pl_full);
  return rclcpp::Time(msg->header.stamp).seconds();
}

double LidarPointCloudDecoder::process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                       pcl::PointCloud<PointType>& pl_full) {
  double t0 = rclcpp::Time(msg->header.stamp).seconds();

  switch (lidar_type) {
    case VELODYNE:
      velodyne_handler(msg, pl_full);
      break;
    case OUSTER:
      ouster_handler(msg, pl_full);
      break;
    case HESAI:
      hesai_handler(msg, pl_full);
      break;
    case ROBOSENSE:
      robosense_handler(msg, pl_full);
      break;
    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;
    default:
      printf("\033[31mUnsupported lidar type: %d\033[0m\n", lidar_type);
  }
  return t0;
}

void LidarPointCloudDecoder::livox_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                                           pcl::PointCloud<PointType>& pl_full) {
  size_t N = msg->point_num;
  pl_full.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    PointType pt;
    pt.x = msg->points[i].x;
    pt.y = msg->points[i].y;
    pt.z = msg->points[i].z;
    pt.intensity = msg->points[i].reflectivity;
    pt.curvature = msg->points[i].offset_time * (1e-9);

    if ((i % point_filter_num) == 0 &&
        (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind) {
      pl_full.push_back(pt);
    }
  }
}

void LidarPointCloudDecoder::velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                              pcl::PointCloud<PointType>& pl_full) {
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.size();
  if (N == 0) return;

  if (pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12) {
    // Has timestamp information
    for (size_t i = 0; i < N; ++i) {
      const auto& in = pl_orig[i];
      PointType pt;
      pt.x = in.x;
      pt.y = in.y;
      pt.z = in.z;
      pt.curvature = in.time;
      if ((i % point_filter_num) == 0 &&
          (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind) {
        pl_full.push_back(pt);
      }
    }
  } else {
    // Estimate timestamp from azimuth angle
    bool first = true;
    double yaw0 = 0, yaw_last = 0, bias = 0;
    int cool = 0;
    for (size_t i = 0; i < N; ++i) {
      const auto& in = pl_orig[i];
      PointType pt;
      pt.x = in.x;
      pt.y = in.y;
      pt.z = in.z;
      pt.curvature = in.time;

      if (std::fabs(pt.x) < 0.1) continue;
      double yaw = std::atan2(pt.y, pt.x) * 57.2957795 - bias;
      if (first) {
        yaw0 = yaw_last = yaw;
        first = false;
      }
      if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < blind) continue;
      if ((yaw - yaw_last) > 180 && cool-- <= 0) {
        bias += 360;
        yaw -= 360;
        cool = 1000;
      }
      if (std::fabs(yaw - yaw_last) > 180) yaw += 360;
      pt.curvature = (yaw0 - yaw) / omega_l;
      yaw_last = yaw;
      if (pt.curvature >= 0 && pt.curvature < 0.1 &&
          (i % point_filter_num) == 0) {
        pl_full.push_back(pt);
      }
    }
  }
}

void LidarPointCloudDecoder::ouster_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                            pcl::PointCloud<PointType>& pl_full) {
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.t / 1e9;

    if ((i % point_filter_num) == 0 &&
        (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind) {
      pl_full.push_back(pt);
    }
  }
}

void LidarPointCloudDecoder::hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                           pcl::PointCloud<PointType>& pl_full) {
  pcl::PointCloud<xt32_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  double t0 = pl_orig.points.front().timestamp;

  for (size_t i = 0; i < N; ++i) {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.normal_x = 0;
    pt.normal_y = 0;
    pt.normal_z = 0;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.timestamp - t0;

    if ((i % point_filter_num) == 0 &&
        (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind) {
      pl_full.push_back(pt);
    }
  }
}

double LidarPointCloudDecoder::robosense_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                                 pcl::PointCloud<PointType>& pl_full) {
  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  double t_base_s = rclcpp::Time(msg->header.stamp).seconds();

  for (size_t i = 0; i < N; ++i) {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.timestamp - t_base_s;

    if (((i % point_filter_num) == 0) && ((pt.x * pt.x + pt.y * pt.y) > blind)) {
      pl_full.push_back(pt);
    }
  }
  return t_base_s;
}

void LidarPointCloudDecoder::tartanair_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                               pcl::PointCloud<PointType>& pl_full) {
  pcl::PointCloud<pcl::PointXYZ> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  pl_full.reserve(pl_orig.size());
  for (const auto& in : pl_orig.points) {
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.curvature = 0.0;
    pl_full.push_back(pt);
  }
}

} // namespace sensor
} // namespace vina_slam
