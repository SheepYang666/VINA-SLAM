#pragma once

#include "vina_slam/core/types.hpp"

#include <fstream>
#include <string>

using namespace std;

class FileReaderWriter
{
private:
  FileReaderWriter() = default;
  std::ofstream pose_ofs;

public:
  static FileReaderWriter& instance();

  void save_pcd(PVecPtr pptr, IMUST& xx, int count, const string& savename);

  void clear_txt_file(const std::string& filePath);

  void init_pose_file(const std::string& full_path);

  void save_pose_tum(const IMUST& x);
};
