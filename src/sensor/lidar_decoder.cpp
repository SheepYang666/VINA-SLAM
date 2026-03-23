#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

template <class T>
void pcl_handler(T& msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());

  double t_start = feat.process(msg, *pl_ptr);
  pcl_time_lock.lock();
  pcl_time = t_start;
  pcl_time_lock.unlock();

  if (pl_ptr->empty())
  {
    PointType ap;
    ap.x = 0;
    ap.y = 0;
    ap.z = 0;
    ap.intensity = 0;
    ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType& x, PointType& y) { return x.curvature < y.curvature; });

  while (pl_ptr->back().curvature > 0.11)
  {
    pl_ptr->points.pop_back();
  }

  mBuf.lock();

  time_buf.push_back(t_start);
  pcl_buf.push_back(pl_ptr);

  mBuf.unlock();
}

// Explicit template instantiations for the two message types used
template void pcl_handler<const livox_ros_driver2::msg::CustomMsg::SharedPtr>(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
template void
pcl_handler<const sensor_msgs::msg::PointCloud2::SharedPtr>(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
