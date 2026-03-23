#pragma once

#include "vina_slam/core/constants.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

using namespace std;

inline Eigen::Matrix3d I33(Eigen::Matrix3d::Identity());

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& ang)
{
  double ang_norm = ang.norm();
  if (ang_norm >= 1e-9)
  {
    Eigen::Vector3d r_axis = ang / ang_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }

  return I33;
}

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& ang_vel, const double& dt)
{
  double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 1e-7)
  {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << SKEW_SYM_MATRX(r_axis);
    double r_ang = ang_vel_norm * dt;

    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }

  return I33;
}

inline Eigen::Vector3d Log(const Eigen::Matrix3d& R)
{
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3d K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Omega;
}

inline Eigen::Matrix3d jr(Eigen::Vector3d vec)
{
  double ang = vec.norm();

  if (ang < 1e-9)
  {
    return I33;
  }
  else
  {
    vec /= ang;
    double ra = sin(ang) / ang;
    return ra * I33 + (1 - ra) * vec * vec.transpose() - (1 - cos(ang)) / ang * hat(vec);
  }
}

inline Eigen::Matrix3d jr_inv(const Eigen::Matrix3d& rotR)
{
  Eigen::AngleAxisd rot_vec(rotR);
  Eigen::Vector3d axi = rot_vec.axis();
  double ang = rot_vec.angle();

  if (ang < 1e-9)
  {
    return I33;
  }
  else
  {
    double ctt = ang / 2 / tan(ang / 2);
    return ctt * I33 + (1 - ctt) * axi * axi.transpose() + ang / 2 * hat(axi);
  }
}
