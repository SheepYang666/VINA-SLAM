#pragma once
#include <Eigen/Core>

struct ImuData
{
  double timestamp;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
};
