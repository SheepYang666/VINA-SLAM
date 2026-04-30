#pragma once

#include "vina_slam/core/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

extern ros::Publisher pub_scan;
extern ros::Publisher pub_cmap;
extern ros::Publisher pub_curr_path;
extern ros::Publisher pub_voxel_plane;
extern ros::Publisher pub_voxel_normal;

template <typename CloudT>
void pub_pl_func(CloudT& pl, ros::Publisher& pub)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);

  output.header.frame_id = "camera_init";
  output.header.stamp = ros::Time::now();

  pub.publish(output);
}

class ResultOutput
{
private:
  ResultOutput() = default;

public:
  static ResultOutput& instance();

  void pub_odom_func(IMUST& xc);

  void pub_localtraj(PLV(3) & pwld, double jour, IMUST& x_curr, int cur_session, pcl::PointCloud<PointType>& pcl_path);

  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr>& pvec_buf, vector<IMUST>& x_buf,
                    pcl::PointCloud<PointType>& pcl_path, int win_base, int win_count);
};
