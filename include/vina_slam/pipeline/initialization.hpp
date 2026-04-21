#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include "vina_slam/preintegration.hpp"

#include <deque>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>
#include <vector>

using namespace std;

class Initialization
{
private:
  Initialization() = default;

public:
  static Initialization& instance();

  void align_gravity(vector<IMUST>& xs);

  void motion_blur(pcl::PointCloud<PointType>& pl, PVec& pvec, IMUST xc, IMUST xl,
                   deque<sensor_msgs::Imu::Ptr>& imus, double pcl_beg_time, IMUST& extrin_para);

  int motion_init(vector<pcl::PointCloud<PointType>::Ptr>& pl_origs,
                  vector<deque<sensor_msgs::Imu::Ptr>>& vec_imus, vector<double>& beg_times,
                  Eigen::MatrixXd* hess, LidarFactor& voxhess, vector<IMUST>& x_buf,
                  unordered_map<VOXEL_LOC, OctoTree*>& surf_map, unordered_map<VOXEL_LOC, OctoTree*>& surf_map_slide,
                  vector<PVecPtr>& pvec_buf, int win_size, vector<vector<SlideWindow*>>& sws, IMUST& x_curr,
                  deque<IMU_PRE*>& imu_pre_buf, IMUST& extrin_para);
};
