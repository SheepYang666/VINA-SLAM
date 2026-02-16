/**
 * @file octree.hpp
 * @brief OctoTree data structure for voxel-based mapping
 *
 * Implements an octree for hierarchical spatial organization of
 * point clouds, with plane fitting and sliding window management.
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/core/constants.hpp"
#include "vina_slam/mapping/plane.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include "vina_slam/mapping/factors.hpp"
#include <Eigen/Eigenvalues>
#include <mutex>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vina_slam {
namespace mapping {

// Global configuration (defined in voxel_map.cpp)
extern int max_layer;
extern int max_points;
extern double voxel_size;
extern int min_ba_point;
extern std::vector<int> mp;
extern Eigen::Vector4d min_point;
extern double min_eigen_value;
extern std::vector<double> plane_eigen_value_thre;

/**
 * @brief Compute covariance contribution from point variance
 * @param pv Point with variance
 * @param bcov Output 9x9 covariance contribution
 * @param vec Point position in world frame
 */
void bfVar(const core::pointVar& pv, Eigen::Matrix<double, 9, 9>& bcov, const Eigen::Vector3d& vec);

/**
 * @brief OctoTree node for hierarchical voxel organization
 *
 * Each node can either store points directly (leaf) or have
 * up to 8 child nodes. Used for efficient plane extraction
 * and point-to-plane matching.
 */
class OctoTree {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Sliding window for temporal point storage
  SlideWindow* sw = nullptr;

  /// Accumulated point cluster statistics
  core::PointCluster pcr_add;

  /// Second-order moment covariance [xx,xy,xz,yy,yz,zz,x,y,z]
  Eigen::Matrix<double, 9, 9> cov_add;

  /// Fixed point cluster (from prior map)
  core::PointCluster pcr_fix;

  /// Fixed points (from prior map)
  core::PVec point_fix;

  /// Tree depth level (0 = root)
  int layer;

  /// 0 = leaf (not subdivided), 1 = internal (has children), 2 = plane node
  int octo_state;

  /// Window size for sliding window
  int wdsize;

  /// Child nodes (8 possible children for octree)
  OctoTree* leaves[8];

  /// Center of this voxel
  double voxel_center[3];

  /// Journey distance (for tracking)
  double jour = 0;

  /// Quarter of voxel edge length
  float quater_length;

  /// Fitted plane
  core::Plane plane;

  /// Whether this node has any points
  bool isexist = false;

  /// Eigenvalues of point distribution
  Eigen::Vector3d eig_value;

  /// Eigenvectors of point distribution
  Eigen::Matrix3d eig_vector;

  /// Last point count (for change detection)
  int last_num = 0;

  /// Optimization state
  int opt_state = -1;

  /// Mutex for thread-safe access
  std::mutex mVox;

  /**
   * @brief Constructor
   * @param _l Layer/depth level
   * @param _w Window size
   */
  OctoTree(int _l, int _w);

  /**
   * @brief Push a point into the sliding window
   * @param ord Frame order in window
   * @param pv Point with variance
   * @param pw Point in world frame
   * @param sws Sliding window pool for reuse
   */
  void push(int ord, const core::pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws);

  /**
   * @brief Push a fixed point (from prior map)
   * @param pv Point with variance
   */
  void pushFix(core::pointVar& pv);

  /**
   * @brief Push a fixed point without variance tracking
   * @param pv Point with variance
   */
  void pushFixNovar(core::pointVar& pv);

  /**
   * @brief Check if eigenvalues indicate a plane
   * @param eig_values Eigenvalues to check
   * @return true if points form a valid plane
   */
  bool planeJudge(Eigen::Vector3d& eig_values);

  /**
   * @brief Allocate point to appropriate location (leaf or child)
   * @param ord Frame order in window
   * @param pv Point with variance
   * @param pw Point in world frame
   * @param sws Sliding window pool
   */
  void allocate(int ord, const core::pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws);

  /**
   * @brief Allocate fixed point to appropriate location
   * @param pv Point with variance
   */
  void allocateFix(core::pointVar& pv);

  /**
   * @brief Distribute fixed points to child nodes
   * @param sws Sliding window pool
   */
  void fixDivide(std::vector<SlideWindow*>& sws);

  /**
   * @brief Subdivide points from one frame to children
   * @param si Source frame index
   * @param xx Pose for transformation
   * @param sws Sliding window pool
   */
  void subdivide(int si, core::IMUST& xx, std::vector<SlideWindow*>& sws);

  /**
   * @brief Update plane from accumulated points
   */
  void planeUpdate();

  /**
   * @brief Recursively recut/refine the tree structure
   * @param win_count Current window count
   * @param x_buf State buffer
   * @param sws Sliding window pool
   */
  void recut(int win_count, std::vector<core::IMUST>& x_buf, std::vector<SlideWindow*>& sws);

  /**
   * @brief Marginalize old frames
   * @param win_count Current window count
   * @param mgsize Number of frames to marginalize
   * @param x_buf State buffer
   * @param vox_opt LidarFactor for optimization transfer
   */
  void margi(int win_count, int mgsize, std::vector<core::IMUST>& x_buf, const LidarFactor& vox_opt);

  /**
   * @brief Transfer optimization data to factor
   * @param vox_opt Output LidarFactor
   */
  void trasOpt(LidarFactor& vox_opt);

  /**
   * @brief Transfer optimization data to NormalFactor
   * @param vox_opt Output NormalFactor
   */
  void trasOpt(NormalFactor& vox_opt);

  /**
   * @brief Match point against planes in this tree
   * @param wld World point to match
   * @param pla Output matched plane
   * @param max_prob Output match probability
   * @param var_wld Point variance in world frame
   * @param sigma_d Output distance sigma
   * @param oc Output matching OctoTree node
   * @return Match quality (0 = no match)
   */
  int match(Eigen::Vector3d& wld, core::Plane*& pla, double& max_prob,
            Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc);

  /**
   * @brief Transfer child pointers for release
   * @param octos_release Vector to collect nodes for release
   */
  void trasPtr(std::vector<OctoTree*>& octos_release);

  /**
   * @brief Delete all child nodes recursively
   */
  void deletePtr();

  /**
   * @brief Collect points for visualization
   * @param win_count Current window count
   * @param pl_fixd Output fixed points
   * @param pl_wind Output sliding window points
   * @param x_buf State buffer
   */
  void trasDisplay(int win_count, pcl::PointCloud<core::PointType>& pl_fixd,
                   pcl::PointCloud<core::PointType>& pl_wind, std::vector<core::IMUST>& x_buf);

  /**
   * @brief Check if point is inside this voxel
   * @param wld World point
   * @return true if inside
   */
  bool inside(Eigen::Vector3d& wld);

  /**
   * @brief Clear sliding window data
   * @param sws Sliding window pool for return
   */
  void clearSlwd(std::vector<SlideWindow*>& sws);

  /**
   * @brief Collect plane markers for visualization
   * @param out Output marker array
   * @param max_layer Maximum layer to visualize
   * @param used_ids Set of used marker IDs
   * @param alpha Transparency
   * @param max_trace Maximum trace for filtering
   * @param pow_num Power for size scaling
   */
  void collectPlaneMarkers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                           std::unordered_set<int>& used_ids, float alpha = 0.8f,
                           double max_trace = 0.25, double pow_num = 0.2);

  /**
   * @brief Collect normal markers for visualization
   */
  void collectNormalMarkers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                            std::unordered_set<int>& used_ids, float alpha = 0.8f,
                            double max_trace = 0.25, double pow_num = 0.2);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void push_fix(core::pointVar& pv) { pushFix(pv); }
  void push_fix_novar(core::pointVar& pv) { pushFixNovar(pv); }
  bool plane_judge(Eigen::Vector3d& eig_values) { return planeJudge(eig_values); }
  void allocate_fix(core::pointVar& pv) { allocateFix(pv); }
  void fix_divide(std::vector<SlideWindow*>& sws) { fixDivide(sws); }
  void plane_update() { planeUpdate(); }
  void tras_opt(LidarFactor& vox_opt) { trasOpt(vox_opt); }
  void tras_opt(NormalFactor& vox_opt) { trasOpt(vox_opt); }
  void tras_ptr(std::vector<OctoTree*>& octos_release) { trasPtr(octos_release); }
  void delete_ptr() { deletePtr(); }
  void tras_display(int win_count, pcl::PointCloud<core::PointType>& pl_fixd,
                    pcl::PointCloud<core::PointType>& pl_wind, std::vector<core::IMUST>& x_buf) {
    trasDisplay(win_count, pl_fixd, pl_wind, x_buf);
  }
  void clear_slwd(std::vector<SlideWindow*>& sws) { clearSlwd(sws); }
  void collect_plane_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                             std::unordered_set<int>& used_ids, float alpha = 0.8f,
                             double max_trace = 0.25, double pow_num = 0.2) {
    collectPlaneMarkers(out, max_layer, used_ids, alpha, max_trace, pow_num);
  }
  void collect_normal_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                              std::unordered_set<int>& used_ids, float alpha = 0.8f,
                              double max_trace = 0.25, double pow_num = 0.2) {
    collectNormalMarkers(out, max_layer, used_ids, alpha, max_trace, pow_num);
  }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility aliases removed - use voxel_map.hpp for global class names
