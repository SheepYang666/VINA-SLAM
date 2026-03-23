#include "vina_slam/platform/ros2/io.hpp"

#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>

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
