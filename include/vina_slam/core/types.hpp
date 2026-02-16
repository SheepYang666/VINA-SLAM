/**
 * @file types.hpp
 * @brief Core data types for VINA-SLAM
 *
 * This file contains all fundamental data structures used throughout
 * the VINA-SLAM system, including state representations, point types,
 * and sensor data structures.
 */

#pragma once

#include "vina_slam/core/constants.hpp"
#include "vina_slam/core/math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <vector>

namespace vina_slam {
namespace core {

// ============================================================================
// Point Type Definition
// ============================================================================
using PointType = pcl::PointXYZINormal;

// ============================================================================
// IMU State (15-DOF)
// ============================================================================
/**
 * @brief Full IMU state representation
 *
 * State vector layout:
 * - R (3): Rotation (SO3, parameterized by rotation vector)
 * - p (3): Position
 * - v (3): Velocity
 * - bg (3): Gyroscope bias
 * - ba (3): Accelerometer bias
 * - g (3): Gravity vector (usually [0, 0, -9.8])
 *
 * Total: 15 DOF + time + covariance
 */
struct IMUST {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t;                                  // Timestamp
  Eigen::Matrix3d R;                         // Rotation matrix
  Eigen::Vector3d p;                         // Position
  Eigen::Vector3d v;                         // Velocity
  Eigen::Vector3d bg;                        // Gyroscope bias
  Eigen::Vector3d ba;                        // Accelerometer bias
  Eigen::Vector3d g;                         // Gravity vector
  Eigen::Matrix<double, DIM, DIM> cov;       // State covariance

  /**
   * @brief Default constructor - initialize to zero/identity
   */
  IMUST() {
    setZero();
  }

  /**
   * @brief Copy constructor
   */
  IMUST(const IMUST& b)
      : t(b.t), R(b.R), p(b.p), v(b.v), bg(b.bg), ba(b.ba), g(b.g), cov(b.cov) {}

  /**
   * @brief Full constructor
   */
  IMUST(double _t, const Eigen::Matrix3d& _R, const Eigen::Vector3d& _p,
        const Eigen::Vector3d& _v, const Eigen::Vector3d& _bg,
        const Eigen::Vector3d& _ba,
        const Eigen::Vector3d& _g = Eigen::Vector3d(0, 0, -G_m_s2))
      : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {
  }

  /**
   * @brief State update using error state
   * @param ist Error state vector (15x1)
   */
  IMUST& operator+=(const Eigen::Matrix<double, DIM, 1>& ist) {
    this->R = this->R * Exp(ist.block<3, 1>(0, 0));
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    return *this;
  }

  /**
   * @brief State difference (error state)
   * @param b Other state
   * @return Error state vector (15x1)
   */
  Eigen::Matrix<double, DIM, 1> operator-(const IMUST& b) {
    Eigen::Matrix<double, DIM, 1> a;
    a.block<3, 1>(0, 0) = Log(b.R.transpose() * this->R);
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    return a;
  }

  /**
   * @brief Assignment operator
   */
  IMUST& operator=(const IMUST& b) {
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

  /**
   * @brief Reset state to zero/identity
   */
  void setZero() {
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

// ============================================================================
// Point with Variance
// ============================================================================
/**
 * @brief Point with position, variance, and intensity
 */
struct pointVar {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pnt;       // Point position
  Eigen::Matrix3d var;       // Position variance/covariance
  float intensity;           // Point intensity
};

using PVec = std::vector<pointVar>;
using PVecPtr = std::shared_ptr<std::vector<pointVar>>;

// ============================================================================
// Point Cluster (for plane fitting)
// ============================================================================
/**
 * @brief Accumulated point statistics for plane fitting
 *
 * Stores:
 * - P: Second moment matrix (sum of p*p^T)
 * - v: Sum of point positions
 * - N: Number of points
 */
class PointCluster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d P;   // Second moment
  Eigen::Vector3d v;   // First moment (sum)
  int N;               // Point count

  PointCluster() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  /**
   * @brief Add a point to the cluster
   */
  void push(const Eigen::Vector3d& vec) {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  /**
   * @brief Compute covariance matrix
   */
  Eigen::Matrix3d cov() {
    Eigen::Vector3d center = v / N;
    return P / N - center * center.transpose();
  }

  PointCluster& operator+=(const PointCluster& sigv) {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;
    return *this;
  }

  PointCluster& operator-=(const PointCluster& sigv) {
    this->P -= sigv.P;
    this->v -= sigv.v;
    this->N -= sigv.N;
    return *this;
  }

  /**
   * @brief Transform cluster by a state
   */
  void transform(const PointCluster& sigv, const IMUST& stat) {
    N = sigv.N;
    v = stat.R * sigv.v + N * stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R * sigv.P * stat.R.transpose() + rp + rp.transpose() +
        N * stat.p * stat.p.transpose();
  }
};

// ============================================================================
// Plane Model
// ============================================================================
/**
 * @brief Plane representation for mapping
 */
struct Plane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> plane_var;
  float radius = 0;
  bool is_plane = false;
  Eigen::Vector3d normalized_normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_prev = Eigen::Vector3d::Zero();
  double d = 0;

  // Eigenvalue statistics
  double min_eig_value = 0;
  double mid_eig_value = 0;
  double max_eig_value = 0;
  double center_norm = 0;

  // Visualization flags
  bool is_update = false;
  bool is_published = false;
  bool is_normal_update = false;
  bool is_normal_published = false;

  Plane();
};

// ============================================================================
// VNC (Vector Normal Consistency) Pair
// ============================================================================
/**
 * @brief VNC matching pair for rotation constraint
 *
 * Stores the relationship between a scan plane normal and a map plane normal
 * for VNC residual computation in IEKF.
 */
struct VNCPair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d n_scan;      ///< Scan plane normal (body frame)
  Eigen::Vector3d n_map;       ///< Map plane normal (world frame)
  Eigen::Vector3d p_scan;      ///< Scan point position (body frame)
  double sigma_n;              ///< Normal estimation uncertainty
  double weight;               ///< Precomputed weight

  // Cached values for efficiency
  Eigen::Matrix3d skew_n_scan; ///< Skew-symmetric matrix [n_scan]×

  /**
   * @brief Default constructor
   */
  VNCPair()
    : n_scan(Eigen::Vector3d::Zero())
    , n_map(Eigen::Vector3d::Zero())
    , p_scan(Eigen::Vector3d::Zero())
    , sigma_n(0.1)
    , weight(1.0)
    , skew_n_scan(Eigen::Matrix3d::Zero()) {}
};

// ============================================================================
// Voxel Location Hash
// ============================================================================
/**
 * @brief Voxel location for hash map key
 */
class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {
  }

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

} // namespace core
} // namespace vina_slam

// ============================================================================
// Hash function for VOXEL_LOC
// ============================================================================
namespace std {
template <>
struct hash<vina_slam::core::VOXEL_LOC> {
  size_t operator()(const vina_slam::core::VOXEL_LOC& s) const {
    using std::hash;
    using std::size_t;
    return (((hash<int64_t>()(s.z) * HASH_P) % MAX_N +
             hash<int64_t>()(s.y)) * HASH_P) % MAX_N +
           hash<int64_t>()(s.x);
  }
};
} // namespace std

// ============================================================================
// Backward compatibility aliases
// ============================================================================
using vina_slam::core::PointType;
using vina_slam::core::IMUST;
using vina_slam::core::pointVar;
using vina_slam::core::PVec;
using vina_slam::core::PVecPtr;
using vina_slam::core::PointCluster;
using vina_slam::core::Plane;
using vina_slam::core::VOXEL_LOC;
using vina_slam::core::VNCPair;
