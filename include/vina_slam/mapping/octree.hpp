#pragma once

#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/plane.hpp"
#include "vina_slam/mapping/slide_window.hpp"
#include <mutex>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

extern Eigen::Vector4d min_point;
extern double min_eigen_value;
extern int max_layer;
extern int max_points;
extern double voxel_size;
extern int min_ba_point;
extern std::vector<double> plane_eigen_value_thre;

extern std::vector<int> mp;

// The octotree map for odometry and local mapping
class OctoTree
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SlideWindow* sw = nullptr;
  PointCluster pcr_add;
  Eigen::Matrix<double, 9, 9> cov_add;

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

  bool fitScanPlane(const Eigen::Vector3d& sensor_pos);

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
