#pragma once

#include "vina_slam/core/types.hpp"
#include <pcl/point_cloud.h>

// 10 scans merge into a keyframe
struct Keyframe
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  IMUST x0;
  pcl::PointCloud<PointType>::Ptr plptr;
  int exist;
  int id, mp;
  float jour;

  Keyframe(IMUST& _x0);

  void generate(pcl::PointCloud<PointType>& pl_send, Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0));
};
