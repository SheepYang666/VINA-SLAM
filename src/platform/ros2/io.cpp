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
  // The caller supplies the post-BA pose, so each written row is the optimized frame pose.
  Eigen::Quaterniond q(x.R);
  pose_ofs << std::fixed << std::setprecision(9)
           << x.t << " "
           << x.p.x() << " " << x.p.y() << " " << x.p.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
           << "\n";
  pose_ofs.flush();
}

bool FileReaderWriter::save_frame_pcd(const pcl::PointCloud<PointType>& cloud, const IMUST& optimized_pose,
                                      const IMUST& extrinsic, int frame_id, const std::string& save_dir)
{
  if (save_dir.empty())
  {
    return false;
  }

  std::error_code error;
  std::filesystem::create_directories(save_dir, error);
  if (error)
  {
    return false;
  }

  pcl::PointCloud<PointType> cloud_world = cloud;
  const Eigen::Matrix3d world_rotation = optimized_pose.R * extrinsic.R;
  const Eigen::Vector3d world_translation = optimized_pose.R * extrinsic.p + optimized_pose.p;

  for (PointType& point : cloud_world.points)
  {
    const Eigen::Vector3d point_body(point.x, point.y, point.z);
    const Eigen::Vector3d point_world = world_rotation * point_body + world_translation;
    point.x = static_cast<float>(point_world.x());
    point.y = static_cast<float>(point_world.y());
    point.z = static_cast<float>(point_world.z());

    if (is_valid_normal(point))
    {
      const Eigen::Vector3d normal_body(point.normal_x, point.normal_y, point.normal_z);
      const Eigen::Vector3d normal_world = world_rotation * normal_body;
      point.normal_x = static_cast<float>(normal_world.x());
      point.normal_y = static_cast<float>(normal_world.y());
      point.normal_z = static_cast<float>(normal_world.z());
    }
  }

  const std::filesystem::path pcd_path = std::filesystem::path(save_dir) / frame_pcd_filename(frame_id);
  return pcl::io::savePCDFileBinary(pcd_path.string(), cloud_world) == 0;
}
