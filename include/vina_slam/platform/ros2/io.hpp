#pragma once

#include "vina_slam/core/types.hpp"
#include <fstream>
#include <rclcpp/node.hpp>
#include <string>

using namespace std;

class FileReaderWriter
{
private:
  explicit FileReaderWriter(const rclcpp::Node::SharedPtr& node_in);
  rclcpp::Node::SharedPtr node;
  std::ofstream pose_ofs;

public:
  static FileReaderWriter& instance(const rclcpp::Node::SharedPtr& node_in);

  static FileReaderWriter& instance();

  void clear_txt_file(const std::string& filePath);

  void init_pose_file(const std::string& full_path);

  void save_pose_tum(const IMUST& x);

  bool save_frame_pcd(const pcl::PointCloud<PointType>& cloud, const IMUST& optimized_pose, const IMUST& extrinsic,
                      int frame_id, const std::string& save_dir);
};
