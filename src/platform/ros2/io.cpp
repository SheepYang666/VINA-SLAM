/**
 * @file io.cpp
 * @brief Implementation of file I/O utilities
 */

#include "vina_slam/platform/ros2/io.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>

namespace vina_slam {
namespace platform {
namespace ros2 {

bool FileIO::savePcd(const std::string& path, const pcl::PointCloud<core::PointType>& cloud) {
  return pcl::io::savePCDFileBinary(path, cloud) == 0;
}

bool FileIO::loadPcd(const std::string& path, pcl::PointCloud<core::PointType>& cloud) {
  return pcl::io::loadPCDFile<core::PointType>(path, cloud) == 0;
}

bool FileIO::savePoses(const std::string& path, const std::vector<core::IMUST>& poses) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }

  for (const auto& pose : poses) {
    Eigen::Quaterniond q(pose.R);
    ofs << pose.t << " " << pose.p.x() << " " << pose.p.y() << " " << pose.p.z() << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }

  ofs.close();
  return true;
}

bool FileIO::loadPoses(const std::string& path, std::vector<core::IMUST>& poses) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return false;
  }

  poses.clear();
  double t, px, py, pz, qx, qy, qz, qw;
  while (ifs >> t >> px >> py >> pz >> qx >> qy >> qz >> qw) {
    core::IMUST pose;
    pose.t = t;
    pose.p << px, py, pz;
    pose.R = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    poses.push_back(pose);
  }

  ifs.close();
  return true;
}

bool FileIO::saveTumTrajectory(const std::string& path, const std::vector<core::IMUST>& poses) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }

  for (const auto& pose : poses) {
    Eigen::Quaterniond q(pose.R);
    ofs << pose.t << " " << pose.p.x() << " " << pose.p.y() << " " << pose.p.z() << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }

  ofs.close();
  return true;
}

bool FileIO::saveKitTiTrajectory(const std::string& path, const std::vector<core::IMUST>& poses) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }

  for (const auto& pose : poses) {
    Eigen::Matrix3d R = pose.R;
    Eigen::Vector3d t = pose.p;

    // KITTI format: R00 R01 R02 tx R10 R11 R12 ty R20 R21 R22 tz
    ofs << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t.x() << " " << R(1, 0) << " " << R(1, 1)
        << " " << R(1, 2) << " " << t.y() << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " "
        << t.z() << std::endl;
  }

  ofs.close();
  return true;
}

bool FileIO::fileExists(const std::string& path) {
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0);
}

bool FileIO::createDirectory(const std::string& path) {
  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

FileReaderWriter::FileReaderWriter(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {
}

FileReaderWriter& FileReaderWriter::instance(const rclcpp::Node::SharedPtr& node_in) {
  static FileReaderWriter inst(node_in);
  return inst;
}

FileReaderWriter& FileReaderWriter::instance() {
  return instance(rclcpp::Node::SharedPtr());
}

void FileReaderWriter::savePcd(core::PVecPtr pptr, core::IMUST& /*xx*/, int count, const std::string& savename) {
  pcl::PointCloud<pcl::PointXYZI> pl_save;

  for (core::pointVar& pw : *pptr) {
    pcl::PointXYZI ap;
    ap.x = pw.pnt[0];
    ap.y = pw.pnt[1];
    ap.z = pw.pnt[2];
    pl_save.push_back(ap);
  }

  std::string pcdname = savename + "/" + std::to_string(count) + ".pcd";
  pcl::io::savePCDFileBinary(pcdname, pl_save);
}

void FileReaderWriter::clearTxtFile(const std::string& filePath) {
  std::ofstream ofs(filePath, std::ofstream::out | std::ofstream::trunc);
  if (!ofs.is_open()) {
    std::cerr << "Unable to open file for clearing: " << filePath << std::endl;
    return;
  }
  ofs.close();
}

void FileReaderWriter::savePcdMap(const pcl::PointCloud<core::PointType>& pl) {
  if (save_path.empty() || is_save_map == 0) {
    return;
  }

  std::string pcd_path = save_path + "/map.pcd";
  FileIO::savePcd(pcd_path, pl);
}

bool FileReaderWriter::loadPcdMap(const std::string& path, pcl::PointCloud<core::PointType>& pl) {
  return FileIO::loadPcd(path, pl);
}

} // namespace ros2
} // namespace platform
} // namespace vina_slam
