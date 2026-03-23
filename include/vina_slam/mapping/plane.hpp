#pragma once

#include "vina_slam/core/types.hpp"
#include <Eigen/Core>

struct Plane
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> plane_var;
  float radius = 0;
  bool is_plane = false;
  Eigen::Vector3d normalized_normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_prev = Eigen::Vector3d::Zero();
  double d = 0;

  bool is_update = false;
  bool is_published = false;
  bool is_normal_update = false;
  bool is_normal_published = false;

  Plane();
};

void Bf_var(const pointVar& pv, Eigen::Matrix<double, 9, 9>& bcov, const Eigen::Vector3d& vec);
