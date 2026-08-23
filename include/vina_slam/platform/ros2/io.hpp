#pragma once

#include "vina_slam/core/types.hpp"
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <rclcpp/node.hpp>
#include <string>
#include <thread>

using namespace std;

class FileReaderWriter
{
private:
  explicit FileReaderWriter(const rclcpp::Node::SharedPtr& node_in);
  ~FileReaderWriter();

  rclcpp::Node::SharedPtr node;
  std::ofstream pose_ofs;

  struct PcdWriteJob
  {
    pcl::PointCloud<PointType>::ConstPtr cloud;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    std::string path;
  };

  std::thread pcd_writer_thread;
  std::mutex pcd_mutex;
  std::condition_variable pcd_queue_filled;
  std::condition_variable pcd_queue_drained;
  std::deque<PcdWriteJob> pcd_queue;
  bool pcd_writer_stopping = false;
  bool pcd_write_in_progress = false;
  std::size_t pcd_write_failures = 0;

  void pcd_writer_loop();
  bool write_frame_pcd(const PcdWriteJob& job) const;

public:
  FileReaderWriter(const FileReaderWriter&) = delete;
  FileReaderWriter& operator=(const FileReaderWriter&) = delete;

  static FileReaderWriter& instance(const rclcpp::Node::SharedPtr& node_in);

  static FileReaderWriter& instance();

  void clear_txt_file(const std::string& filePath);

  void init_pose_file(const std::string& full_path);

  void save_pose_tum(const IMUST& x);

  // Queues <frame_id>.pcd for writing on a background thread and returns as soon as the job is accepted, so the
  // mapping loop never blocks on disk I/O. Points are stored in the IMU/body frame (LiDAR extrinsic applied, frame
  // pose not applied); the matching row of the TUM trajectory maps them into the world frame.
  bool save_frame_pcd(const pcl::PointCloud<PointType>::ConstPtr& cloud, const IMUST& extrinsic, int frame_id,
                      const std::string& save_dir);

  // Blocks until every queued frame has been written. Returns false if any write failed.
  bool flush_frame_pcd();
};
