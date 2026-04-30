#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/ekf_imu.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include "vina_slam/preintegration.hpp"

#include <deque>
#include <fstream>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

extern double dept_err, beam_err;

inline double get_memory();

class VINA_SLAM
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;
  double down_size;
  double full_map_voxel_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE*> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow*>> sws;

  vector<OctoTree*> octos_release;
  int thread_num = 5;
  int degrade_bound = 10;

  bool is_finish = false;

  string bagname, savepath, lid_topic, imu_topic;
  int is_save_map;
  int if_BA;
  int enable_visualization = 0;
  int is_save_pose = 0;
  std::string pose_save_path;
  std::string pose_filename;

  explicit VINA_SLAM(const ros::NodeHandle& nh_in, const ros::NodeHandle& pnh_in);

  bool lio_state_estimation(PVecPtr pptr);
  bool VNC_lio(PVecPtr pptr);
  bool LioStateEstimation(PVecPtr pptr, bool use_vnc);

  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr);

  int initialization(deque<sensor_msgs::Imu::Ptr>& imus, Eigen::MatrixXd& hess, LidarFactor& voxhess,
                     PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr);

  void system_reset(deque<sensor_msgs::Imu::Ptr>& imus);

  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, double jour, int win_count, vector<IMUST>& xs,
                   LidarFactor& voxopt, vector<SlideWindow*>& sw);

  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs, LidarFactor& voxopt,
                   vector<vector<SlideWindow*>>& sws);

  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                   LidarFactor& lidarFactor, NormalFactor& normalFactor, vector<vector<SlideWindow*>>& sws);

  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                   vector<vector<SlideWindow*>>& sws);

  void thd_odometry_localmapping();
};

inline double get_memory()
{
  std::ifstream infile("/proc/self/status");
  double mem = -1;
  std::string lineStr, str;
  while (std::getline(infile, lineStr))
  {
    std::stringstream ss(lineStr);
    bool is_find = false;
    while (ss >> str)
    {
      if (str == "VmRSS:")
      {
        is_find = true;
        continue;
      }

      if (is_find)
        mem = std::stod(str);
      break;
    }
    if (is_find)
      break;
  }
  return mem / (1048576);
}
