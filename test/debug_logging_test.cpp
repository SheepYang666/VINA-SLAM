#include "vina_slam/core/debug_logging.hpp"

#include <gtest/gtest.h>

namespace vina_slam::core
{
namespace
{

TEST(DebugLogging, SanitizesEmptyRunLabel)
{
  EXPECT_EQ(sanitize_run_label(""), "run");
  EXPECT_EQ(sanitize_run_label("   "), "run");
}

TEST(DebugLogging, ReplacesPathSeparatorsAndUnsafeCharacters)
{
  EXPECT_EQ(sanitize_run_label("../outdoor mapping:2026/06/04"), "outdoor_mapping_2026_06_04");
}

TEST(DebugLogging, BuildsStableDefaultPaths)
{
  const ZDriftLogPaths paths = make_z_drift_log_paths("", "../bad run");

  EXPECT_EQ(paths.root_dir, std::filesystem::path("/tmp/vina_slam_z_drift"));
  EXPECT_EQ(paths.run_dir, std::filesystem::path("/tmp/vina_slam_z_drift/bad_run"));
  EXPECT_EQ(paths.frontend_csv, std::filesystem::path("/tmp/vina_slam_z_drift/bad_run/z_drift_frontend.csv"));
  EXPECT_EQ(paths.ba_csv, std::filesystem::path("/tmp/vina_slam_z_drift/bad_run/z_drift_ba.csv"));
}

}  // namespace
}  // namespace vina_slam::core
