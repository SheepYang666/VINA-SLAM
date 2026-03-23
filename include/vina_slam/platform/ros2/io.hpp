#pragma once

#include "vina_slam/core/types.hpp"
#include <rclcpp/node.hpp>
#include <string>

using namespace std;

class FileReaderWriter
{
private:
  explicit FileReaderWriter(const rclcpp::Node::SharedPtr& node_in);
  rclcpp::Node::SharedPtr node;

public:
  static FileReaderWriter& instance(const rclcpp::Node::SharedPtr& node_in);

  static FileReaderWriter& instance();

  void save_pcd(PVecPtr pptr, IMUST& xx, int count, const string& savename);

  void clear_txt_file(const std::string& filePath);
};
