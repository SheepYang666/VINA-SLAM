#include "vina_slam/core/debug_logging.hpp"

#include <algorithm>
#include <cctype>

namespace vina_slam
{
namespace core
{
namespace
{
constexpr const char* kDefaultZDriftLogRoot = "/tmp/vina_slam_z_drift";
constexpr const char* kDefaultRunLabel = "run";
constexpr std::size_t kMaxRunLabelLength = 96;

std::string trim_underscores(const std::string& value)
{
  const auto begin = std::find_if(value.begin(), value.end(), [](char ch) { return ch != '_'; });
  const auto end = std::find_if(value.rbegin(), value.rend(), [](char ch) { return ch != '_'; }).base();
  if (begin >= end)
  {
    return "";
  }
  return std::string(begin, end);
}
}  // namespace

std::string sanitize_run_label(const std::string& raw_label)
{
  std::string sanitized;
  sanitized.reserve(std::min(raw_label.size(), kMaxRunLabelLength));

  bool last_was_separator = false;
  for (unsigned char ch : raw_label)
  {
    const bool is_allowed = std::isalnum(ch) || ch == '-' || ch == '_';
    if (is_allowed)
    {
      sanitized.push_back(static_cast<char>(ch));
      last_was_separator = false;
    }
    else if (!last_was_separator)
    {
      sanitized.push_back('_');
      last_was_separator = true;
    }

    if (sanitized.size() >= kMaxRunLabelLength)
    {
      break;
    }
  }

  sanitized = trim_underscores(sanitized);
  if (sanitized.empty())
  {
    return kDefaultRunLabel;
  }
  return sanitized;
}

ZDriftLogPaths make_z_drift_log_paths(const std::string& log_root, const std::string& run_label)
{
  std::filesystem::path root_dir = log_root.empty() ? std::filesystem::path(kDefaultZDriftLogRoot)
                                                    : std::filesystem::path(log_root);
  root_dir = root_dir.lexically_normal();

  const std::filesystem::path run_dir = root_dir / sanitize_run_label(run_label);
  return ZDriftLogPaths{
    root_dir,
    run_dir,
    run_dir / "z_drift_frontend.csv",
    run_dir / "z_drift_ba.csv",
  };
}

}  // namespace core
}  // namespace vina_slam
