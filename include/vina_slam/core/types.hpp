#pragma once

#include "vina_slam/core/constants.hpp"
#include "vina_slam/core/math.hpp"
#include <Eigen/Core>
#include <memory>
#include <pcl/point_cloud.h>
#include <unordered_map>
#include <vector>

using namespace std;

class VOXEL_LOC
{
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz)
  {
  }

  bool operator==(const VOXEL_LOC& other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }
};

namespace std
{
template <>
struct hash<VOXEL_LOC>
{
  size_t operator()(const VOXEL_LOC& s) const
  {
    using std::hash;
    using std::size_t;

    return (((hash<int64_t>()(s.z) * HASH_P) % MAX_N + hash<int64_t>()(s.y)) * HASH_P) % MAX_N + hash<int64_t>()(s.x);
  }
};
}  // namespace std

struct IMUST
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  double t;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d g;
  Eigen::Matrix<double, DIM, DIM> cov;

  IMUST()
  {
    setZero();
  }

  IMUST(double _t, const Eigen::Matrix3d& _R, const Eigen::Vector3d& _p, const Eigen::Vector3d& _v,
        const Eigen::Vector3d& _bg, const Eigen::Vector3d& _ba,
        const Eigen::Vector3d& _g = Eigen::Vector3d(0, 0, -G_m_s2))
    : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g)
  {
  }

  IMUST& operator+=(const Eigen::Matrix<double, DIM, 1>& ist)
  {
    this->R = this->R * Exp(ist.block<3, 1>(0, 0));
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    return *this;
  }

  Eigen::Matrix<double, DIM, 1> operator-(const IMUST& b)
  {
    Eigen::Matrix<double, DIM, 1> a;
    a.block<3, 1>(0, 0) = Log(b.R.transpose() * this->R);
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    return a;
  }

  IMUST& operator=(const IMUST& b)
  {
    this->R = b.R;
    this->p = b.p;
    this->v = b.v;
    this->bg = b.bg;
    this->ba = b.ba;
    this->g = b.g;
    this->t = b.t;
    this->cov = b.cov;
    return *this;
  }

  void setZero()
  {
    t = 0;
    R.setIdentity();
    p.setZero();
    v.setZero();
    bg.setZero();
    ba.setZero();
    cov.setIdentity();
    cov *= 0.0001;
    cov.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 0.00001;
  }
};

class PointCluster
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d& vec)
  {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Matrix3d cov()
  {
    Eigen::Vector3d center = v / N;
    return P / N - center * center.transpose();
  }

  PointCluster& operator+=(const PointCluster& sigv)
  {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  PointCluster& operator-=(const PointCluster& sigv)
  {
    this->P -= sigv.P;
    this->v -= sigv.v;
    this->N -= sigv.N;

    return *this;
  }

  void transform(const PointCluster& sigv, const IMUST& stat)
  {
    N = sigv.N;
    v = stat.R * sigv.v + N * stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R * sigv.P * stat.R.transpose() + rp + rp.transpose() + N * stat.p * stat.p.transpose();
  }
};

struct pointVar
{
  Eigen::Vector3d pnt;
  Eigen::Matrix3d var;
  float intensity = 0;
};

using PVec = std::vector<pointVar>;
using PVecPtr = std::shared_ptr<std::vector<pointVar>>;
