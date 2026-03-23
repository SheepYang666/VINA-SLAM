#include "vina_slam/sensor/sync.hpp"
#include <rclcpp/time.hpp>

// Global sync buffer definitions (moved from VINASlam.hpp/cpp)
mutex mBuf;
LidarPointCloudDecoder feat;
deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;

double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;

mutex pcl_time_lock;
double pcl_time = 0;

bool sync_packages(pcl::PointCloud<PointType>::Ptr& pl_ptr, deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                   IMUEKF& p_imu)
{
  static bool pl_ready = false;

  // Step 1: If the point cloud is not ready yet, take a frame out of the cache
  if (!pl_ready)
  {
    if (pcl_buf.empty())
    {
      return false;
    }

    mBuf.lock();

    pl_ptr = pcl_buf.front();
    pcl_buf.pop_front();
    p_imu.pcl_beg_time = time_buf.front();
    time_buf.pop_front();

    mBuf.unlock();

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    // If the time stamp mode is turned on, the time information is simulated using frame time intervals.
    if (point_notime)
    {
      if (last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;

        return false;
      }

      // Manually set the start/end time of point cloud frame
      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
  }

  if (!pl_ready || imu_last_time <= p_imu.pcl_end_time)
  {
    return false;
  }

  // Step 3: Extract IMU data in the range [pcl_beg_time, pcl_end_time]
  mBuf.lock();
  double imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
  while ((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = rclcpp::Time(imu_buf.front()->header.stamp).seconds();
    if (imu_time > p_imu.pcl_end_time)
      break;
    imus.push_back(imu_buf.front());  // Press the corresponding IMU data of the current frame
    imu_buf.pop_front();
  }
  mBuf.unlock();

  // If the IMU data is used up, it means the data flow is broken and the program is exited
  if (imu_buf.empty())
  {
    exit(0);
  }

  pl_ready = false;  // The current frame processing is completed, the flag is reset

  // If the number of paired IMU data is greater than 4, the synchronization is considered successful
  if (imus.size() > 4)
  {
    return true;
  }
  else
  {
    return false;
  }
}
