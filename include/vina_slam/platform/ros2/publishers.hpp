#pragma once

#include "vina_slam/core/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;

// Global publisher variables (moved from VINASlam.hpp)
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_plane;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_normal;

template <typename CloudT>
void pub_pl_func(CloudT& pl, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                 const rclcpp::Node::SharedPtr& node)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);

  output.header.frame_id = "camera_init";
  output.header.stamp = node->now();

  pub->publish(output);
}

class ResultOutput
{
public:
  static std::unique_ptr<ResultOutput> inst_;
  static bool initialized_;

private:
  explicit ResultOutput(const rclcpp::Node::SharedPtr& node_in);
  rclcpp::Node::SharedPtr node;

public:
  static ResultOutput& instance(const rclcpp::Node::SharedPtr& node_in);

  static ResultOutput& instance();

  void pub_odom_func(IMUST& xc);

  void pub_localtraj(PLV(3) & pwld, double jour, IMUST& x_curr, int cur_session, pcl::PointCloud<PointType>& pcl_path);

  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr>& pvec_buf, vector<IMUST>& x_buf,
                    pcl::PointCloud<PointType>& pcl_path, int win_base, int win_count);
};
