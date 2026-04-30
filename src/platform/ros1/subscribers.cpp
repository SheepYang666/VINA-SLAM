#include "vina_slam/platform/ros1/subscribers.hpp"
#include "vina_slam/sensor/sync.hpp"

ros::Subscriber sub_imu;
ros::Subscriber sub_pcl_livox;
ros::Subscriber sub_pcl_standard;

void imu_handler(const sensor_msgs::Imu::ConstPtr& msg_in)
{
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  mBuf.lock();
  imu_last_time = msg->header.stamp.toSec();
  imu_buf.push_back(msg);
  mBuf.unlock();
}
