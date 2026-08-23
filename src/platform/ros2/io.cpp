#include "vina_slam/platform/ros2/io.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <sstream>

namespace
{
// Upper bound on frames waiting to be written. Beyond this the mapping loop is throttled so that a slow disk cannot
// grow the backlog without limit.
constexpr std::size_t kMaxPendingFrames = 16;

std::string frame_pcd_filename(int frame_id)
{
  std::ostringstream name;
  name << std::setfill('0') << std::setw(6) << frame_id << ".pcd";
  return name.str();
}

bool is_valid_normal(const PointType& point)
{
  return std::isfinite(point.normal_x) && std::isfinite(point.normal_y) && std::isfinite(point.normal_z) &&
         (std::abs(point.normal_x) > 0.0F || std::abs(point.normal_y) > 0.0F || std::abs(point.normal_z) > 0.0F);
}
}  // namespace

FileReaderWriter::FileReaderWriter(const rclcpp::Node::SharedPtr& node_in) : node(node_in)
{
}

FileReaderWriter::~FileReaderWriter()
{
  {
    std::lock_guard<std::mutex> lock(pcd_mutex);
    pcd_writer_stopping = true;
  }
  pcd_queue_filled.notify_all();
  pcd_queue_drained.notify_all();

  if (pcd_writer_thread.joinable())
  {
    pcd_writer_thread.join();
  }
}

FileReaderWriter& FileReaderWriter::instance(const rclcpp::Node::SharedPtr& node_in)
{
  static FileReaderWriter inst(node_in);
  return inst;
}

FileReaderWriter& FileReaderWriter::instance()
{
  rclcpp::Node::SharedPtr node_temp;
  return instance(node_temp);
}

void FileReaderWriter::clear_txt_file(const std::string& filePath)
{
  std::ofstream ofs(filePath, std::ofstream::out | std::ofstream::trunc);
  if (!ofs.is_open())
  {
    std::cerr << "Cannot open file to clear: " << filePath << std::endl;
    return;
  }
  ofs.close();
}

void FileReaderWriter::init_pose_file(const std::string& full_path)
{
  if (pose_ofs.is_open())
  {
    pose_ofs.close();
  }

  const std::filesystem::path pose_path(full_path);
  if (pose_path.has_parent_path())
  {
    std::error_code error;
    std::filesystem::create_directories(pose_path.parent_path(), error);
  }

  pose_ofs.open(full_path, std::ios::out | std::ios::trunc);
  if (!pose_ofs.is_open())
  {
    std::cerr << "[is_save_pose]: Cannot open pose file: " << full_path << std::endl;
  }
  else
  {
    std::cout << "[is_save_pose]: Saving trajectory to " << full_path << std::endl;
  }
}

void FileReaderWriter::save_pose_tum(const IMUST& x)
{
  if (!pose_ofs.is_open())
    return;

  // TUM trajectory format:
  // timestamp tx ty tz qx qy qz qw
  // timestamp is the LiDAR scan header stamp; pose is the post-BA optimized frame pose.
  Eigen::Quaterniond q(x.R);
  pose_ofs << std::fixed << std::setprecision(9)
           << x.t << " "
           << x.p.x() << " " << x.p.y() << " " << x.p.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
           << "\n";
  pose_ofs.flush();
}

bool FileReaderWriter::save_frame_pcd(const pcl::PointCloud<PointType>::ConstPtr& cloud, const IMUST& extrinsic,
                                      int frame_id, const std::string& save_dir)
{
  if (!cloud || save_dir.empty())
  {
    return false;
  }

  std::error_code error;
  std::filesystem::create_directories(save_dir, error);
  if (error)
  {
    return false;
  }

  PcdWriteJob job;
  job.cloud = cloud;
  job.rotation = extrinsic.R;
  job.translation = extrinsic.p;
  job.path = (std::filesystem::path(save_dir) / frame_pcd_filename(frame_id)).string();

  std::unique_lock<std::mutex> lock(pcd_mutex);
  if (pcd_writer_stopping)
  {
    return false;
  }

  if (!pcd_writer_thread.joinable())
  {
    pcd_writer_thread = std::thread(&FileReaderWriter::pcd_writer_loop, this);
  }

  pcd_queue_drained.wait(lock, [this] { return pcd_writer_stopping || pcd_queue.size() < kMaxPendingFrames; });
  if (pcd_writer_stopping)
  {
    return false;
  }

  pcd_queue.push_back(std::move(job));
  lock.unlock();
  pcd_queue_filled.notify_one();
  return true;
}

bool FileReaderWriter::flush_frame_pcd()
{
  std::unique_lock<std::mutex> lock(pcd_mutex);
  pcd_queue_drained.wait(lock, [this] { return pcd_queue.empty() && !pcd_write_in_progress; });

  if (pcd_write_failures != 0)
  {
    std::cerr << "[is_save_map]: " << pcd_write_failures << " frame PCD write(s) failed" << std::endl;
    return false;
  }
  return true;
}

void FileReaderWriter::pcd_writer_loop()
{
  std::unique_lock<std::mutex> lock(pcd_mutex);

  while (true)
  {
    pcd_queue_filled.wait(lock, [this] { return pcd_writer_stopping || !pcd_queue.empty(); });

    // Drain the backlog even while stopping so a graceful shutdown keeps every frame.
    if (pcd_queue.empty())
    {
      return;
    }

    const PcdWriteJob job = std::move(pcd_queue.front());
    pcd_queue.pop_front();
    pcd_write_in_progress = true;

    lock.unlock();
    pcd_queue_drained.notify_all();
    const bool written = write_frame_pcd(job);
    lock.lock();

    pcd_write_in_progress = false;
    if (!written)
    {
      pcd_write_failures++;
    }
    pcd_queue_drained.notify_all();
  }
}

bool FileReaderWriter::write_frame_pcd(const PcdWriteJob& job) const
{
  // Points arrive in the raw LiDAR frame; only the LiDAR-IMU extrinsic is applied so that the file stays in the
  // IMU/body frame of its own scan and stays valid no matter how later optimization moves the trajectory.
  pcl::PointCloud<PointType> cloud_body = *job.cloud;

  for (PointType& point : cloud_body.points)
  {
    const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
    const Eigen::Vector3d point_body = job.rotation * point_lidar + job.translation;
    point.x = static_cast<float>(point_body.x());
    point.y = static_cast<float>(point_body.y());
    point.z = static_cast<float>(point_body.z());

    if (is_valid_normal(point))
    {
      const Eigen::Vector3d normal_lidar(point.normal_x, point.normal_y, point.normal_z);
      const Eigen::Vector3d normal_body = job.rotation * normal_lidar;
      point.normal_x = static_cast<float>(normal_body.x());
      point.normal_y = static_cast<float>(normal_body.y());
      point.normal_z = static_cast<float>(normal_body.z());
    }
  }

  if (pcl::io::savePCDFileBinary(job.path, cloud_body) != 0)
  {
    std::cerr << "[is_save_map]: Cannot write frame PCD: " << job.path << std::endl;
    return false;
  }
  return true;
}
