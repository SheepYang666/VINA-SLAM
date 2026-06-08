#pragma once

#include <filesystem>
#include <string>

namespace vina_slam
{
namespace core
{

struct ZDriftLogPaths
{
  std::filesystem::path root_dir;
  std::filesystem::path run_dir;
  std::filesystem::path frontend_csv;
  std::filesystem::path ba_csv;
};

std::string sanitize_run_label(const std::string& raw_label);

ZDriftLogPaths make_z_drift_log_paths(const std::string& log_root, const std::string& run_label);

}  // namespace core
}  // namespace vina_slam
