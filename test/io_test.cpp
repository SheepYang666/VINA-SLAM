#include "vina_slam/platform/ros2/io.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <unistd.h>

namespace
{
std::filesystem::path make_temp_dir(const std::string& name)
{
  const auto dir = std::filesystem::temp_directory_path() / (name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

bool has_field(const pcl::PCLPointCloud2& cloud, const std::string& name)
{
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&](const pcl::PCLPointField& field) { return field.name == name; });
}

pcl::PointCloud<PointType>::Ptr make_single_point_cloud()
{
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  PointType point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 42.0F;
  point.normal_x = 0.0F;
  point.normal_y = 1.0F;
  point.normal_z = 0.0F;
  point.curvature = 0.125F;
  cloud->push_back(point);
  return cloud;
}

IMUST make_extrinsic()
{
  IMUST extrinsic;
  extrinsic.R = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  extrinsic.p = Eigen::Vector3d(1.0, 0.0, 0.0);
  return extrinsic;
}
}  // namespace

TEST(FileReaderWriter, SaveFramePcdPreservesFieldsAndAppliesExtrinsicOnly)
{
  const auto dir = make_temp_dir("vina_slam_frame_pcd");

  ASSERT_TRUE(FileReaderWriter::instance().save_frame_pcd(make_single_point_cloud(), make_extrinsic(), 7,
                                                          dir.string()));
  ASSERT_TRUE(FileReaderWriter::instance().flush_frame_pcd());

  pcl::PCLPointCloud2 pcd_blob;
  ASSERT_EQ(pcl::io::loadPCDFile((dir / "000007.pcd").string(), pcd_blob), 0);
  EXPECT_TRUE(has_field(pcd_blob, "x"));
  EXPECT_TRUE(has_field(pcd_blob, "y"));
  EXPECT_TRUE(has_field(pcd_blob, "z"));
  EXPECT_TRUE(has_field(pcd_blob, "intensity"));
  EXPECT_TRUE(has_field(pcd_blob, "normal_x"));
  EXPECT_TRUE(has_field(pcd_blob, "normal_y"));
  EXPECT_TRUE(has_field(pcd_blob, "normal_z"));
  EXPECT_TRUE(has_field(pcd_blob, "curvature"));

  pcl::PointCloud<PointType> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile<PointType>((dir / "000007.pcd").string(), loaded), 0);
  ASSERT_EQ(loaded.size(), 1U);

  // Body frame only: Rz(90) * (1, 2, 3) + (1, 0, 0)
  EXPECT_NEAR(loaded[0].x, -1.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].y, 1.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].z, 3.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].intensity, 42.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].normal_x, -1.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].normal_y, 0.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].normal_z, 0.0F, 1e-5F);
  EXPECT_NEAR(loaded[0].curvature, 0.125F, 1e-5F);

  std::filesystem::remove_all(dir);
}

TEST(FileReaderWriter, SaveFramePcdWritesEveryQueuedFrameAsynchronously)
{
  const auto dir = make_temp_dir("vina_slam_frame_pcd_async");
  constexpr int kFrameCount = 40;

  for (int frame_id = 0; frame_id < kFrameCount; ++frame_id)
  {
    ASSERT_TRUE(
        FileReaderWriter::instance().save_frame_pcd(make_single_point_cloud(), make_extrinsic(), frame_id,
                                                    dir.string()));
  }

  ASSERT_TRUE(FileReaderWriter::instance().flush_frame_pcd());

  for (int frame_id = 0; frame_id < kFrameCount; ++frame_id)
  {
    std::ostringstream name;
    name << std::setfill('0') << std::setw(6) << frame_id << ".pcd";
    EXPECT_TRUE(std::filesystem::exists(dir / name.str())) << "missing " << name.str();
  }

  std::filesystem::remove_all(dir);
}

TEST(FileReaderWriter, SaveFramePcdRejectsEmptyDirectoryAndNullCloud)
{
  const auto dir = make_temp_dir("vina_slam_frame_pcd_invalid");

  EXPECT_FALSE(FileReaderWriter::instance().save_frame_pcd(make_single_point_cloud(), make_extrinsic(), 0, ""));
  EXPECT_FALSE(FileReaderWriter::instance().save_frame_pcd(nullptr, make_extrinsic(), 0, dir.string()));

  std::filesystem::remove_all(dir);
}

TEST(FileReaderWriter, SavePoseTumWritesTimestampPositionAndQuaternion)
{
  const auto dir = make_temp_dir("vina_slam_pose_tum");
  const auto path = dir / "nested" / "trajectory.txt";

  IMUST optimized_pose;
  optimized_pose.t = 12.5;
  optimized_pose.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  optimized_pose.R = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  FileReaderWriter::instance().init_pose_file(path.string());
  FileReaderWriter::instance().save_pose_tum(optimized_pose);

  ASSERT_TRUE(std::filesystem::exists(path.parent_path()));
  std::ifstream input(path);
  ASSERT_TRUE(input.is_open());
  std::string line;
  std::getline(input, line);

  std::istringstream values(line);
  double timestamp = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0;
  values >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

  EXPECT_NEAR(timestamp, 12.5, 1e-9);
  EXPECT_NEAR(tx, 1.0, 1e-9);
  EXPECT_NEAR(ty, 2.0, 1e-9);
  EXPECT_NEAR(tz, 3.0, 1e-9);
  EXPECT_NEAR(qx, 0.0, 1e-9);
  EXPECT_NEAR(qy, 0.0, 1e-9);
  EXPECT_NEAR(qz, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(qw, std::sqrt(0.5), 1e-9);

  std::filesystem::remove_all(dir);
}
