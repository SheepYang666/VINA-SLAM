#include "vina_slam/platform/ros2/bag_reader.hpp"

#include "vina_slam/lidar_pointcloud_decoder.hpp"
#include "vina_slam/platform/ros2/subscribers.hpp"
#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vina_slam
{
namespace
{

void wait_for_buffer_space(std::size_t max_imu_buf, std::size_t max_pcl_buf)
{
  while (rclcpp::ok())
  {
    mBuf.lock();
    const std::size_t imu_size = imu_buf.size();
    const std::size_t pcl_size = pcl_buf.size();
    mBuf.unlock();

    if (imu_size < max_imu_buf && pcl_size < max_pcl_buf)
    {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void wait_for_drain(double idle_timeout_sec)
{
  auto last_progress = std::chrono::steady_clock::now();
  std::size_t last_total = std::numeric_limits<std::size_t>::max();

  while (rclcpp::ok())
  {
    mBuf.lock();
    const std::size_t imu_size = imu_buf.size();
    const std::size_t pcl_size = pcl_buf.size();
    mBuf.unlock();

    // No lidar left to process: leftover IMUs after the last scan are expected at end-of-bag.
    if (pcl_size == 0)
    {
      // Allow the odometry thread to finish the in-flight synced package (if any).
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (imu_size > 0)
      {
        std::cout << "[BagReader] Drain complete with leftover imu_buf=" << imu_size
                  << " (expected after last lidar), finishing." << std::endl;
      }
      return;
    }

    const std::size_t total = imu_size + pcl_size;
    if (total < last_total)
    {
      last_total = total;
      last_progress = std::chrono::steady_clock::now();
    }
    else
    {
      const auto idle = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_progress).count();
      if (idle >= idle_timeout_sec)
      {
        std::cout << "[BagReader] Drain idle timeout with imu_buf=" << imu_size << " pcl_buf=" << pcl_size
                  << ", finishing." << std::endl;
        return;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <typename MsgT>
typename MsgT::SharedPtr deserialize_message(const rosbag2_storage::SerializedBagMessage& bag_msg)
{
  rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
  auto msg = std::make_shared<MsgT>();
  rclcpp::Serialization<MsgT> serialization;
  serialization.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

}  // namespace

void run_bag_reader(const rclcpp::Node::SharedPtr& node, const BagReaderOptions& options)
{
  if (options.bag_path.empty())
  {
    throw std::runtime_error("General.bag_path is empty; set a rosbag2 directory in config.");
  }

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = options.bag_path;

  rosbag2_cpp::Reader reader;
  try
  {
    reader.open(storage_options);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(std::string("Failed to open bag at '") + options.bag_path + "': " + e.what());
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics = {options.lid_topic, options.imu_topic};
  reader.set_filter(filter);

  const bool use_livox = (options.lidar_type == LIVOX);

  std::cout << "[BagReader] Reading bag: " << options.bag_path << std::endl;
  std::cout << "[BagReader] lid_topic: " << options.lid_topic << " imu_topic: " << options.imu_topic
            << " lidar_type: " << options.lidar_type << std::endl;

  std::size_t imu_count = 0;
  std::size_t lidar_count = 0;

  while (rclcpp::ok() && reader.has_next())
  {
    wait_for_buffer_space(options.max_imu_buf, options.max_pcl_buf);
    if (!rclcpp::ok())
    {
      break;
    }

    auto bag_msg = reader.read_next();
    if (!bag_msg)
    {
      continue;
    }

    if (bag_msg->topic_name == options.imu_topic)
    {
      auto msg = deserialize_message<sensor_msgs::msg::Imu>(*bag_msg);
      imu_handler(msg);
      ++imu_count;
    }
    else if (bag_msg->topic_name == options.lid_topic)
    {
      if (use_livox)
      {
        const livox_ros_driver2::msg::CustomMsg::SharedPtr msg =
            deserialize_message<livox_ros_driver2::msg::CustomMsg>(*bag_msg);
        pcl_handler(msg);
      }
      else
      {
        const sensor_msgs::msg::PointCloud2::SharedPtr msg =
            deserialize_message<sensor_msgs::msg::PointCloud2>(*bag_msg);
        pcl_handler(msg);
      }
      ++lidar_count;
    }
  }

  std::cout << "[BagReader] Finished reading. imu=" << imu_count << " lidar=" << lidar_count << std::endl;

  wait_for_drain(options.drain_idle_timeout_sec);

  if (node)
  {
    node->set_parameter(rclcpp::Parameter("finish", true));
  }
}

}  // namespace vina_slam
