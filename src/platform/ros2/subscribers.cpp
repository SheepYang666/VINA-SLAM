#include "vina_slam/platform/ros2/subscribers.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <rclcpp/time.hpp>

// Global subscriber definitions (moved from VINASlam.hpp)
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard;

void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg_in)
{
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

  mBuf.lock();
  imu_last_time = rclcpp::Time(msg->header.stamp).seconds();

  imu_buf.push_back(msg);
  mBuf.unlock();
}
