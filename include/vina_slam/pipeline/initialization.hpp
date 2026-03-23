#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include "vina_slam/preintegration.hpp"

#include <deque>
#include <pcl/point_cloud.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unordered_map>
#include <vector>

using namespace std;

class Initialization
{
private:
  explicit Initialization(const rclcpp::Node::SharedPtr& node_in);
  rclcpp::Node::SharedPtr node;

public:
  static Initialization& instance(const rclcpp::Node::SharedPtr& node_in);

  static Initialization& instance();

  void align_gravity(vector<IMUST>& xs);

  void motion_blur(pcl::PointCloud<PointType>& pl, PVec& pvec, IMUST xc, IMUST xl,
                   deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, double pcl_beg_time, IMUST& extrin_para);

  int motion_init(vector<pcl::PointCloud<PointType>::Ptr>& pl_origs,
                  vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>>& vec_imus, vector<double>& beg_times,
                  Eigen::MatrixXd* hess, LidarFactor& voxhess, vector<IMUST>& x_buf,
                  unordered_map<VOXEL_LOC, OctoTree*>& surf_map, unordered_map<VOXEL_LOC, OctoTree*>& surf_map_slide,
                  vector<PVecPtr>& pvec_buf, int win_size, vector<vector<SlideWindow*>>& sws, IMUST& x_curr,
                  deque<IMU_PRE*>& imu_pre_buf, IMUST& extrin_para);
};
