#include "vina_slam/lidar_pointcloud_decoder.hpp"
#include "vina_slam/tools.hpp"

template <typename T>
T decode_field(const uint8_t* data, bool is_bigendian)
{
  T value;
  if (is_bigendian)
  {
    uint8_t reversed[sizeof(T)];
    std::reverse_copy(data, data + sizeof(T), reversed);
    std::memcpy(&value, reversed, sizeof(T));
  }
  else
  {
    std::memcpy(&value, data, sizeof(T));
  }
  return value;
}

double LidarPointCloudDecoder::process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                                       pcl::PointCloud<PointType>& pl_full)
{
  livox_handler(msg, pl_full);
  return rclcpp::Time(msg->header.stamp).seconds();
}

double LidarPointCloudDecoder::process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                       pcl::PointCloud<PointType>& pl_full)
{
  double t0 = rclcpp::Time(msg->header.stamp).seconds();

  switch (lidar_type)
  {
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
      t0 = robosense_handler(msg, pl_full);
      break;
    case ROBOSENSE_AIRY:
      t0 = robosense_airy_handler(msg, pl_full);
      break;
    case LIVOX_MID360_POINTCLOUD2:
      mid360_handler(msg, pl_full);
      break;
    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;
    default:
      std::cout << RED << "Unsupported lidar type: " << lidar_type << RESET << std::endl;
  }
  return t0;
}

void LidarPointCloudDecoder::livox_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
                                           pcl::PointCloud<PointType>& pl_full)
{
  size_t N = msg->point_num;
  pl_full.reserve(N);
  for (size_t i = 0; i < N; ++i)
  {
    PointType pt;
    pt.x = msg->points[i].x;
    pt.y = msg->points[i].y;
    pt.z = msg->points[i].z;
    pt.intensity = msg->points[i].reflectivity;
    pt.curvature = msg->points[i].offset_time * (1e-9);

    if ((i % point_filter_num) == 0 && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind)
    {
      pl_full.push_back(pt);
    }
  }
}

void LidarPointCloudDecoder::mid360_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                            pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<livox_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  if (pl_orig.empty())
  {
    return;
  }

  const size_t point_count = pl_orig.points.size();
  const size_t stride = point_filter_num > 0 ? static_cast<size_t>(point_filter_num) : 1U;
  const double timebase_ns = pl_orig.points.front().timestamp;
  pl_full.reserve(pl_full.size() + point_count / stride + 1U);

  for (size_t i = 1; i < point_count; ++i)
  {
    const auto& in = pl_orig.points[i];
    const std::uint8_t return_type = in.tag & 0x30;
    if ((return_type != 0x10 && return_type != 0x00) || (i % stride) != 0U)
    {
      continue;
    }

    const auto& prev = pl_orig.points[i - 1];
    const bool is_duplicate =
        std::abs(in.x - prev.x) <= 1e-7F && std::abs(in.y - prev.y) <= 1e-7F && std::abs(in.z - prev.z) <= 1e-7F;
    const double dist_sq =
        static_cast<double>(in.x) * in.x + static_cast<double>(in.y) * in.y + static_cast<double>(in.z) * in.z;
    if (is_duplicate || dist_sq <= blind)
    {
      continue;
    }

    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    // MID360 PointCloud2 timestamps are nanoseconds; curvature stores seconds
    // from scan start.
    pt.curvature = static_cast<float>((in.timestamp - timebase_ns) * 1e-9);
    pl_full.push_back(pt);
  }
}

void LidarPointCloudDecoder::velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                              pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.size();
  if (N == 0)
    return;

  if (pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12)
  {
    for (size_t i = 0; i < N; ++i)
    {
      const auto& in = pl_orig[i];
      PointType pt;
      pt.x = in.x;
      pt.y = in.y;
      pt.z = in.z;
      pt.curvature = in.time;
      if ((i % point_filter_num) == 0 && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind)
      {
        pl_full.push_back(pt);
      }
    }
  }
  else
  {
    bool first = true;
    double yaw0 = 0, yaw_last = 0, bias = 0;
    int cool = 0;
    for (size_t i = 0; i < N; ++i)
    {
      const auto& in = pl_orig[i];
      PointType pt;
      pt.x = in.x;
      pt.y = in.y;
      pt.z = in.z;
      pt.curvature = in.time;

      if (std::fabs(pt.x) < 0.1)
        continue;
      double yaw = std::atan2(pt.y, pt.x) * 57.2957795 - bias;
      if (first)
      {
        yaw0 = yaw_last = yaw;
        first = false;
      }
      if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < blind)
        continue;
      if ((yaw - yaw_last) > 180 && cool-- <= 0)
      {
        bias += 360;
        yaw -= 360;
        cool = 1000;
      }
      if (std::fabs(yaw - yaw_last) > 180)
        yaw += 360;
      pt.curvature = (yaw0 - yaw) / omega_l;
      yaw_last = yaw;
      if (pt.curvature >= 0 && pt.curvature < 0.1 && (i % point_filter_num) == 0)
      {
        pl_full.push_back(pt);
      }
    }
  }
}

void LidarPointCloudDecoder::ouster_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                            pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  for (size_t i = 0; i < N; ++i)
  {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.t / 1e9;

    if ((i % point_filter_num) == 0 && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind)
    {
      pl_full.push_back(pt);
    }
  }
}

void LidarPointCloudDecoder::hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                           pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<xt32_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  double t0 = pl_orig.points.front().timestamp;

  for (size_t i = 0; i < N; ++i)
  {
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

    if ((i % point_filter_num) == 0 && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > blind)
    {
      pl_full.push_back(pt);
    }
  }
}

double LidarPointCloudDecoder::robosense_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                                 pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  size_t N = pl_orig.points.size();
  pl_full.reserve(N);
  double t_base_s = rclcpp::Time(msg->header.stamp).seconds();

  for (size_t i = 0; i < N; ++i)
  {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.timestamp - t_base_s;

    if (((i % point_filter_num) == 0) && ((pt.x * pt.x + pt.y * pt.y) > blind))
    {
      pl_full.push_back(pt);
    }
  }
  return t_base_s;
}

double LidarPointCloudDecoder::robosense_airy_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                                      pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<robosense::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  if (pl_orig.empty())
  {
    return rclcpp::Time(msg->header.stamp).seconds();
  }

  size_t N = pl_orig.points.size();
  pl_full.reserve(N);

  // Airy point timestamps are in sensor-local seconds and keep a constant offset from
  // header.stamp. Use the earliest point only to recover per-point relative time while
  // keeping the frame timestamp on the ROS time base for IMU synchronization.
  double t_first = std::numeric_limits<double>::infinity();
  for (const auto& in : pl_orig.points)
  {
    t_first = std::min(t_first, in.timestamp);
  }
  if (!std::isfinite(t_first))
  {
    return rclcpp::Time(msg->header.stamp).seconds();
  }

  for (size_t i = 0; i < N; ++i)
  {
    const auto& in = pl_orig.points[i];
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.intensity = in.intensity;
    pt.curvature = in.timestamp - t_first;

    if (((i % point_filter_num) == 0) && ((pt.x * pt.x + pt.y * pt.y) > blind))
    {
      pl_full.push_back(pt);
    }
  }
  return rclcpp::Time(msg->header.stamp).seconds();
}

void LidarPointCloudDecoder::tartanair_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                               pcl::PointCloud<PointType>& pl_full)
{
  pcl::PointCloud<pcl::PointXYZ> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  pl_full.reserve(pl_orig.size());
  for (const auto& in : pl_orig.points)
  {
    PointType pt;
    pt.x = in.x;
    pt.y = in.y;
    pt.z = in.z;
    pt.curvature = 0.0;
    pl_full.push_back(pt);
  }
}
