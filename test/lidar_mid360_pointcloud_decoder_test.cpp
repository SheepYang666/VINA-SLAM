#include "vina_slam/lidar_pointcloud_decoder.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{
livox_ros::Point make_point(float x, float y, float z, float intensity, std::uint8_t tag, double timestamp)
{
  livox_ros::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.tag = tag;
  point.timestamp = timestamp;
  return point;
}
}  // namespace

TEST(LidarPointCloudDecoder, DecodesLivoxMid360PointCloud2)
{
  pcl::PointCloud<livox_ros::Point> input;
  input.push_back(make_point(1.0F, 0.0F, 0.0F, 1.0F, 0x10, 1'000'000'000.0));
  input.push_back(make_point(2.0F, 0.0F, 0.0F, 2.0F, 0x10, 1'020'000'000.0));
  input.push_back(make_point(2.0F, 0.0F, 0.0F, 3.0F, 0x10, 1'030'000'000.0));
  input.push_back(make_point(3.0F, 0.0F, 0.0F, 4.0F, 0x30, 1'040'000'000.0));
  input.push_back(make_point(4.0F, 0.0F, 0.0F, 5.0F, 0x00, 1'050'000'000.0));

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(input, msg);
  msg.header.stamp.sec = 123;
  msg.header.stamp.nanosec = 0;

  LidarPointCloudDecoder decoder;
  decoder.lidar_type = LIVOX_MID360_POINTCLOUD2;
  decoder.point_filter_num = 1;
  decoder.blind = 0.0;

  pcl::PointCloud<PointType> output;
  const auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
  const double frame_time = decoder.process(msg_ptr, output);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_DOUBLE_EQ(frame_time, 123.0);
  EXPECT_FLOAT_EQ(output[0].x, 2.0F);
  EXPECT_FLOAT_EQ(output[0].intensity, 2.0F);
  EXPECT_NEAR(output[0].curvature, 0.02F, 1e-6F);
  EXPECT_FLOAT_EQ(output[1].x, 4.0F);
  EXPECT_FLOAT_EQ(output[1].intensity, 5.0F);
  EXPECT_NEAR(output[1].curvature, 0.05F, 1e-6F);
}
