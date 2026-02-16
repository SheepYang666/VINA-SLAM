#pragma once

#include "vina_slam/estimation/imu_preintegration.hpp"
#include "vina_slam/core/common.hpp"  // Replaces tools.hpp - provides types, math, constants
#include <Eigen/Eigenvalues>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

// Bring frequently used STL types to global scope (legacy code compatibility)
using std::deque;
using std::mutex;
using std::pair;
using std::make_pair;
using std::ref;
using std::thread;
using std::unordered_map;
using std::unordered_set;
using std::vector;

// Backward compatibility aliases for core types
using pointVar = vina_slam::core::pointVar;
using PVec = vina_slam::core::PVec;
using PVecPtr = vina_slam::core::PVecPtr;
using Plane = vina_slam::core::Plane;
using IMUST = vina_slam::core::IMUST;
using PointType = vina_slam::core::PointType;
using PointCluster = vina_slam::core::PointCluster;
using VOXEL_LOC = vina_slam::core::VOXEL_LOC;

extern Eigen::Vector4d min_point;
extern double min_eigen_value;
extern int max_layer;
extern int max_points;
extern double voxel_size;
extern int min_ba_point;
extern std::vector<double> plane_eigen_value_thre;

void Bf_var(const pointVar& pv, Eigen::Matrix<double, 9, 9>& bcov, const Eigen::Vector3d& vec);

// The LiDAR BA factor in optimization

class LidarFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

// The LM optimizer for LiDAR BA in HBA
class Lidar_BA_Optimizer
{
public:
  int win_size;     // Optimize window size
  int jac_leng;     // Optimize variable dimensions (usually win_size × 6, rotation + translation)
  int thd_num = 4;  // Number of threads used (default 4)

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess);
  bool damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd* hess, vector<double>& resis,
                    int max_iter = 3, bool is_display = false);
};

extern double imu_coef;
#define DVEL 6

// The LiDAR-Inertial BA optimizer
class LI_BA_Optimizer
{
public:
  int win_size;
  int jac_leng;
  int imu_leng;

  // debug
  void print_breakdown(const char* tag, std::vector<IMUST>& xs, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus, double& imu_res, double& lidar_res, double& normal_res,
                       double& total_res) const;
  // debug

  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double divide_thread(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor);

  double only_residual(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                       std::deque<IMU_PRE*>& imus_factor);

  void damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd* hess);

  void damping_iter(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                    std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd* hess);
};

// The LiDAR-Inertial BA optimizer with gravity optimization
class LI_BA_OptimizerGravity
{
public:
  int win_size, jac_leng, imu_leng;

  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs, Eigen::VectorXd& js);

  double divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT);

  double only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor);

  void damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor, vector<double>& resis,
                    Eigen::MatrixXd* hess, int max_iter = 2);
};

// 10 scans merge into a keyframe
struct Keyframe
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUST x0;
  pcl::PointCloud<PointType>::Ptr plptr;
  int exist;
  int id, mp;
  float jour;

  Keyframe(IMUST& _x0);

  void generate(pcl::PointCloud<PointType>& pl_send, Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0));
};

// The sldingwindow in each voxel nodes
class SlideWindow
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PVec> points;  /// The set of points for each frame in the sliding window, containing covariance
                             /// information (PVec = vector<pointVar>)
  std::vector<PointCluster> pcrs_local;  /// Statistics of point clouds for each frame in the sliding window (center of
                                         /// mass, covariance, etc.)

  SlideWindow(int wdsize);
  void resize(int wdsize);

  void clear();
};

// The octotree map for odometry and local mapping

// int *mp;
extern std::vector<int> mp;

class OctoTree
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlideWindow* sw = nullptr;
  PointCluster pcr_add;
  Eigen::Matrix<double, 9, 9> cov_add;  // 二阶矩(6) + 均值(3) [xx, xy, xz, yy, yz, zz, x, y, z]

  PointCluster pcr_fix;
  PVec point_fix;

  int layer, octo_state, wdsize;
  OctoTree* leaves[8];
  double voxel_center[3];
  double jour = 0;
  float quater_length;

  Plane plane;
  bool isexist = false;

  Eigen::Vector3d eig_value;
  Eigen::Matrix3d eig_vector;

  int last_num = 0, opt_state = -1;
  mutex mVox;

  OctoTree(int _l, int _w);

  void push(int ord, const pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws);

  void push_fix(pointVar& pv);

  void push_fix_novar(pointVar& pv);

  bool plane_judge(Eigen::Vector3d& eig_values);

  void allocate(int ord, const pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws);

  void allocate_fix(pointVar& pv);

  void fix_divide(std::vector<SlideWindow*>& sws);

  void subdivide(int si, IMUST& xx, std::vector<SlideWindow*>& sws);

  void plane_update();

  void recut(int win_count, std::vector<IMUST>& x_buf, std::vector<SlideWindow*>& sws);

  void margi(int win_count, int mgsize, std::vector<IMUST>& x_buf, const LidarFactor& vox_opt);

  void tras_opt(LidarFactor& vox_opt);
  void tras_opt(NormalFactor& vox_opt);

  int match(Eigen::Vector3d& wld, Plane*& pla, double& max_prob, Eigen::Matrix3d& var_wld, double& sigma_d,
            OctoTree*& oc);

  void tras_ptr(vector<OctoTree*>& octos_release);

  void delete_ptr();

  void tras_display(int win_count, pcl::PointCloud<PointType>& pl_fixd, pcl::PointCloud<PointType>& pl_wind,
                    std::vector<IMUST>& x_buf);

  bool inside(Eigen::Vector3d& wld);

  void clear_slwd(std::vector<SlideWindow*>& sws);

  void collect_plane_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                             std::unordered_set<int>& used_ids, float alpha = 0.8f, double max_trace = 0.25,
                             double pow_num = 0.2);

  void collect_normal_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                              std::unordered_set<int>& used_ids, float alpha = 0.8f, double max_trace = 0.25,
                              double pow_num = 0.2);
};

void cut_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVecPtr pvec, int win_count,
               std::unordered_map<VOXEL_LOC, OctoTree*>& feat_tem_map, int wdsize, PLV(3) & pwld,
               std::vector<SlideWindow*>& sws);

void cut_voxel_multi(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVecPtr pvec, int win_count,
                     std::unordered_map<VOXEL_LOC, OctoTree*>& feat_tem_map, int wdsize, PLV(3) & pwld,
                     std::vector<std::vector<SlideWindow*>>& sws);

void cut_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVec& pvec, int wdsize, double jour);

void generate_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVec& pvec, double voxel_size);

void generate_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, IMUST& x_curr, PVec& pvec, double voxel_size);

// Match the point with the plane in the voxel map
int match(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, Eigen::Vector3d& wld, Plane*& pla, Eigen::Matrix3d& var_wld,
          double& sigma_d, OctoTree*& oc);

void down_sampling_pvec(PVec& pvec, double voxel_size, pcl::PointCloud<PointType>& pl_keep);
