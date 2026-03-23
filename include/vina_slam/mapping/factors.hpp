#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/mapping/plane.hpp"
#include <Eigen/Eigenvalues>
#include <vector>

// The LiDAR BA factor in optimization

class LidarFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  vector<PointCluster> sig_vecs;
  vector<vector<PointCluster>> plvec_voxels;
  vector<double> coeffs;
  PLV(3) eig_values;
  PLM(3) eig_vectors;
  vector<PointCluster> pcr_adds;
  int win_size;

  explicit LidarFactor(int _w);

  void push_voxel(vector<PointCluster>& vec_orig, PointCluster& fix, double coe, Eigen::Vector3d& eig_value,
                  Eigen::Matrix3d& eig_vector, PointCluster& pcr_add);

  void acc_evaluate2(const vector<IMUST>& xs, int head, int end, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                     double& residual);

  void evaluate_only_residual(const vector<IMUST>& xs, int head, int end, double& residual);

  void clear();

  ~LidarFactor() = default;
};

// The NormalFactor BA factor in optimization
class NormalFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  std::vector<PointCluster> sig_vecs;
  std::vector<std::vector<PointCluster>> plvec_voxels;
  std::vector<double> coeffs;
  PLV(3) n_refs;

  std::vector<PointCluster> pcr_adds;

  int win_size;

  explicit NormalFactor(int _w);

  void push_voxel(std::vector<PointCluster>& vec_orig, PointCluster& fix, double coe, Eigen::Vector3d& n_ref,
                  PointCluster& pcr_add);

  void acc_evaluate2(const std::vector<IMUST>& xs, int head, int end, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                     double& residual);

  void evaluate_only_residual(const std::vector<IMUST>& xs, int head, int end, double& residual);

  void clear();
  ~NormalFactor() = default;
};
