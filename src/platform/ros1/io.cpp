#include "vina_slam/platform/ros1/io.hpp"

#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pcl/io/pcd_io.h>

FileReaderWriter& FileReaderWriter::instance()
{
  static FileReaderWriter inst;
  return inst;
}

void FileReaderWriter::save_pcd(PVecPtr pptr, IMUST& xx, int count, const string& savename)
{
  pcl::PointCloud<pcl::PointXYZI> pl_save;

  for (pointVar& pw : *pptr)
  {
    pcl::PointXYZI ap;
    ap.x = pw.pnt[0];
    ap.y = pw.pnt[1];
    ap.z = pw.pnt[2];
    pl_save.push_back(ap);
  }

  string pcdname = savename + "/" + to_string(count) + ".pcd";

  pcl::io::savePCDFileBinary(pcdname, pl_save);
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
  Eigen::Quaterniond q(x.R);
  pose_ofs << std::fixed << std::setprecision(9)
           << x.t << " "
           << x.p.x() << " " << x.p.y() << " " << x.p.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
           << "\n";
}
