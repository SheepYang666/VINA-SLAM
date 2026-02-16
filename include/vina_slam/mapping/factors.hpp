/**
 * @file factors.hpp
 * @brief Optimization factors for LiDAR bundle adjustment
 *
 * Contains LidarFactor and NormalFactor for plane-based
 * and normal-based optimization constraints.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/core/constants.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include <Eigen/Eigenvalues>
#include <vector>

namespace vina_slam {
namespace mapping {

/**
 * @brief LiDAR factor for plane-based bundle adjustment
 *
 * Accumulates point-to-plane constraints across the sliding window
 * and provides efficient Hessian and Jacobian computation using
 * eigenvalue decomposition of point clusters.
 */
class LidarFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Aggregated point cluster for each voxel
  std::vector<core::PointCluster> sig_vecs;

  /// Point clusters per frame for each voxel
  std::vector<std::vector<core::PointCluster>> plvec_voxels;

  /// Weight coefficients for each voxel
  std::vector<double> coeffs;

  /// Eigenvalues for each voxel (smallest eigenvalue = plane fit quality)
  PLV(3) eig_values;

  /// Eigenvectors for each voxel
  PLM(3) eig_vectors;

  /// Accumulated point clusters after transformation
  std::vector<core::PointCluster> pcr_adds;

  /// Sliding window size
  int win_size;

  /**
   * @brief Constructor with window size
   * @param _w Window size (number of frames)
   */
  explicit LidarFactor(int _w);

  /**
   * @brief Add a voxel constraint
   * @param vec_orig Point clusters per frame in this voxel
   * @param fix Aggregated fixed point cluster
   * @param coe Weight coefficient
   * @param eig_value Eigenvalues of the voxel
   * @param eig_vector Eigenvectors of the voxel
   * @param pcr_add Accumulated point cluster
   */
  void pushVoxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                 double coe, Eigen::Vector3d& eig_value, Eigen::Matrix3d& eig_vector,
                 core::PointCluster& pcr_add);

  /**
   * @brief Evaluate Hessian and Jacobian for optimization
   * @param xs Current state estimates
   * @param head Start index in voxel array
   * @param end End index in voxel array
   * @param Hess Output Hessian matrix
   * @param JacT Output Jacobian transpose (gradient)
   * @param residual Output residual cost
   */
  void accEvaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                    Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual);

  /**
   * @brief Evaluate only the residual (for convergence check)
   * @param xs Current state estimates
   * @param head Start index in voxel array
   * @param end End index in voxel array
   * @param residual Output residual cost
   */
  void evaluateOnlyResidual(const std::vector<core::IMUST>& xs, int head, int end, double& residual);

  /**
   * @brief Clear all voxel constraints
   */
  void clear();

  ~LidarFactor() = default;

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void push_voxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                  double coe, Eigen::Vector3d& eig_value, Eigen::Matrix3d& eig_vector,
                  core::PointCluster& pcr_add) {
    pushVoxel(vec_orig, fix, coe, eig_value, eig_vector, pcr_add);
  }
  void acc_evaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                     Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual) {
    accEvaluate2(xs, head, end, Hess, JacT, residual);
  }
  void evaluate_only_residual(const std::vector<core::IMUST>& xs, int head, int end, double& residual) {
    evaluateOnlyResidual(xs, head, end, residual);
  }
};

/**
 * @brief Normal-based factor for bundle adjustment
 *
 * Uses reference normals for optimization, useful when
 * plane eigenvalues are unreliable but normal direction is known.
 */
class NormalFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Aggregated point cluster for each voxel
  std::vector<core::PointCluster> sig_vecs;

  /// Point clusters per frame for each voxel
  std::vector<std::vector<core::PointCluster>> plvec_voxels;

  /// Weight coefficients for each voxel
  std::vector<double> coeffs;

  /// Reference normals for each voxel
  PLV(3) n_refs;

  /// Accumulated point clusters after transformation
  std::vector<core::PointCluster> pcr_adds;

  /// Sliding window size
  int win_size;

  /**
   * @brief Constructor with window size
   * @param _w Window size (number of frames)
   */
  explicit NormalFactor(int _w);

  /**
   * @brief Add a voxel constraint with reference normal
   * @param vec_orig Point clusters per frame in this voxel
   * @param fix Aggregated fixed point cluster
   * @param coe Weight coefficient
   * @param n_ref Reference normal direction
   * @param pcr_add Accumulated point cluster
   */
  void pushVoxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                 double coe, Eigen::Vector3d& n_ref, core::PointCluster& pcr_add);

  /**
   * @brief Evaluate Hessian and Jacobian for optimization
   * @param xs Current state estimates
   * @param head Start index in voxel array
   * @param end End index in voxel array
   * @param Hess Output Hessian matrix
   * @param JacT Output Jacobian transpose (gradient)
   * @param residual Output residual cost
   */
  void accEvaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                    Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual);

  /**
   * @brief Evaluate only the residual (for convergence check)
   * @param xs Current state estimates
   * @param head Start index in voxel array
   * @param end End index in voxel array
   * @param residual Output residual cost
   */
  void evaluateOnlyResidual(const std::vector<core::IMUST>& xs, int head, int end, double& residual);

  /**
   * @brief Clear all voxel constraints
   */
  void clear();

  ~NormalFactor() = default;

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void push_voxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                  double coe, Eigen::Vector3d& n_ref, core::PointCluster& pcr_add) {
    pushVoxel(vec_orig, fix, coe, n_ref, pcr_add);
  }
  void acc_evaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                     Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual) {
    accEvaluate2(xs, head, end, Hess, JacT, residual);
  }
  void evaluate_only_residual(const std::vector<core::IMUST>& xs, int head, int end, double& residual) {
    evaluateOnlyResidual(xs, head, end, residual);
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility aliases removed - use voxel_map.hpp for global class names
