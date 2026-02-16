#pragma once

// #include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

// Note: BTC.hpp and loop_detection.hpp removed - not needed for localization mode
#include "vina_slam/estimation/imu_ekf.hpp"
#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/core/common.hpp"  // Replaces tools.hpp
#include "vina_slam/voxel_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Bring frequently used STL types to global scope (legacy code compatibility)
using std::deque;
using std::function;
using std::string;
using std::unordered_map;
using std::vector;

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <filesystem>
#include <geometry_msgs/msg/pose_array.hpp>

#include <malloc.h>
#include <mutex>
#include <functional>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;

// ============================================================================
// Global Variables (extern declarations - definitions in sensor_context.cpp)
// ============================================================================
// These global variables are deprecated. Use SensorContext instead.
// See vina_slam/core/sensor_context.hpp for the new API.

extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_init;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pmap;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_test;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prev_path;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;
extern rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_pmap_livox;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_plane;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_normal;
extern rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
extern rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl;

// Utility function for publishing point clouds
template <typename CloudT>
void pub_pl_func(CloudT& pl, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                 const rclcpp::Node::SharedPtr& node)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);

  output.header.frame_id = "camera_init";
  output.header.stamp = node->now();

  pub->publish(output);
}

// Data buffers and synchronization
extern mutex mBuf;
extern LidarPointCloudDecoder feat;
extern deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
extern deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
extern deque<double> time_buf;

// Time tracking
extern double imu_last_time;
extern int point_notime;
extern double last_pcl_time;

// PCL time synchronization
extern mutex pcl_time_lock;
extern double pcl_time;
extern double pcl_end_time;

// Point cloud handler template
template <class T>
void pcl_handler(T& msg);

// Error parameters
extern double dept_err, beam_err;

// ResultOutput and FileReaderWriter classes moved to platform/ros2 module
// Use vina_slam::platform::ros2::ResultPublisher and vina_slam::platform::ros2::FileReaderWriter

// Initialization class moved to vina_slam/pipeline/initialization.hpp
// Backward compatibility alias provided at bottom of file

class VINA_SLAM
{
private:
  rclcpp::Node::SharedPtr node;

public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;
  double down_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE*> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow*>> sws;

  // Loop closure related members removed - not needed for localization mode
  // vector<ScanPose*>* scanPoses;
  // mutex mtx_loop;
  // deque<ScanPose*> buf_lba2loop, buf_lba2loop_tem;
  // int loop_detect = 0;
  // unordered_map<VOXEL_LOC, OctoTree*> map_loop;

  vector<Keyframe*>* keyframes;
  IMUST dx;
  pcl::PointCloud<PointType>::Ptr pl_kdmap;
  pcl::KdTreeFLANN<PointType> kd_keyframes;
  int history_kfsize = 0;
  vector<OctoTree*> octos_release;
  int reset_flag = 0;
  int g_update = 0;
  int thread_num = 5;
  int degrade_bound = 10;

  // Loop closure related members removed
  // vector<vector<ScanPose*>*> multimap_scanPoses;
  vector<vector<Keyframe*>*> multimap_keyframes;
  volatile int gba_flag = 0;
  int gba_size = 0;
  vector<int> cnct_map;
  mutex mtx_keyframe;
  bool is_finish = false;

  vector<string> sessionNames;
  string bagname, savepath, lid_topic, imu_topic;
  int is_save_map;
  int if_BA;
  int if_loop_dect;

  static VINA_SLAM& Instance(const rclcpp::Node::SharedPtr& node_in);
  static VINA_SLAM& instance(const rclcpp::Node::SharedPtr& node_in) { return Instance(node_in); }  // deprecated

  static VINA_SLAM& Instance();
  static VINA_SLAM& instance() { return Instance(); }  // deprecated

  explicit VINA_SLAM(const rclcpp::Node::SharedPtr& node_in);

  bool LioStateEstimation(PVecPtr pptr);
  bool lio_state_estimation(PVecPtr pptr) { return LioStateEstimation(pptr); }  // deprecated

  /**
   * @brief LIO state estimation with VNC (Vector Normal Consistency) residual
   *
   * Extends LioStateEstimation with additional rotation constraint from
   * normal vector consistency between scan and map planes.
   *
   * @param pptr Point cloud with variance information
   * @return true if estimation is valid (non-degenerate), false otherwise
   */
  bool VNCLio(PVecPtr pptr);

  pcl::PointCloud<PointType>::Ptr pl_tree;
  void LioStateEstimationKdtree(PVecPtr pptr);
  void lio_state_estimation_kdtree(PVecPtr pptr) { LioStateEstimationKdtree(pptr); }  // deprecated

  // loop_update removed - not needed for localization mode

  // load the previous keyframe in the local voxel map
  void LoadKeyframes(double jour);
  void keyframe_loading(double jour) { LoadKeyframes(jour); }  // deprecated

  int Initialize(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, Eigen::MatrixXd& hess, LidarFactor& voxhess,
                 PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr);
  int initialization(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, Eigen::MatrixXd& hess, LidarFactor& voxhess,
                     PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr) {  // deprecated
    return Initialize(imus, hess, voxhess, pwld, pcl_curr);
  }

  void ResetSystem(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus);
  void system_reset(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus) { ResetSystem(imus); }  // deprecated

  void MultiMarginalize(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, double jour, int win_count, vector<IMUST>& xs,
                        LidarFactor& voxopt, vector<SlideWindow*>& sw);
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, double jour, int win_count, vector<IMUST>& xs,
                   LidarFactor& voxopt, vector<SlideWindow*>& sw) {  // deprecated
    MultiMarginalize(feat_map, jour, win_count, xs, voxopt, sw);
  }

  // Determine the plane and recut the voxel map in octo-tree
  // Common implementation with optional post-processing
  void MultiRecutImpl(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                      vector<vector<SlideWindow*>>& sws, function<void(OctoTree*)> post_process);
  void multi_recut_impl(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                        vector<vector<SlideWindow*>>& sws, function<void(OctoTree*)> post_process) {  // deprecated
    MultiRecutImpl(feat_map, win_count, xs, sws, post_process);
  }

  void MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs, LidarFactor& voxopt,
                  vector<vector<SlideWindow*>>& sws);
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs, LidarFactor& voxopt,
                   vector<vector<SlideWindow*>>& sws) {  // deprecated
    MultiRecut(feat_map, win_count, xs, voxopt, sws);
  }

  void MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                  LidarFactor& lidarFactor, NormalFactor& normalFactor, vector<vector<SlideWindow*>>& sws);
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                   LidarFactor& lidarFactor, NormalFactor& normalFactor, vector<vector<SlideWindow*>>& sws) {  // deprecated
    MultiRecut(feat_map, win_count, xs, lidarFactor, normalFactor, sws);
  }

  void MultiRecut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                  vector<vector<SlideWindow*>>& sws);
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                   vector<vector<SlideWindow*>>& sws) {  // deprecated
    MultiRecut(feat_map, win_count, xs, sws);
  }

  // The main thread of odometry and local mapping
  void RunOdometryLocalMapping(std::shared_ptr<rclcpp::Node> node);
  void thd_odometry_localmapping(std::shared_ptr<rclcpp::Node> node) { RunOdometryLocalMapping(node); }  // deprecated

private:
  // Helper methods for thd_odometry_localmapping

  // Cleanup released octo tree nodes from memory
  void CleanupReleasedOctos();

  // Release old voxels from surf_map based on journey distance
  void ReleaseOldVoxels(double jour);

  // Cleanup oversized slide windows
  void CleanupSlideWindows();

  // Shift sliding window buffers by mgsize positions
  void ShiftSlidingWindow(int mgsize);

  // Backward compatibility aliases (deprecated)
  void cleanup_released_octos() { CleanupReleasedOctos(); }
  void release_old_voxels(double jour) { ReleaseOldVoxels(jour); }
  void cleanup_slide_windows() { CleanupSlideWindows(); }
  void shift_sliding_window(int mgsize) { ShiftSlidingWindow(mgsize); }
};

inline void calcBodyVar(Eigen::Vector3d& pb, const float range_inc, const float degree_inc, Eigen::Matrix3d& var);

// Compute the variance of the each point

inline void var_init(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr, double dept_err, double beam_err);

inline void pvec_update(PVecPtr pptr, IMUST& x_curr, PLV(3) & pwld);

// read_lidarstate removed - not needed for localization mode

inline double get_memory();

// icp_check removed - not needed for localization mode

inline void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg_in);

inline bool sync_packages(pcl::PointCloud<PointType>::Ptr& pl_ptr, deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus,
                          IMUEKF& p_imu);

// Initialization class is now in vina_slam/pipeline/initialization.hpp
// Include that header directly and use vina_slam::pipeline::Initialization
