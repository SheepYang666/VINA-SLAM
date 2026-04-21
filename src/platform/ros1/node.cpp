// VINA_SLAM class implementation and main() function
// Moved from VINASlam.cpp

#include "vina_slam/platform/ros1/node.hpp"
#include "vina_slam/platform/ros1/publishers.hpp"
#include "vina_slam/platform/ros1/subscribers.hpp"
#include "vina_slam/platform/ros1/io.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/sensor/sync.hpp"
#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/mapping/voxel_map.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace
{
template <typename T>
T get_param_or(ros::NodeHandle& node, const std::string& name, const T& default_value)
{
  T value;
  if (!node.getParam(name, value))
  {
    value = default_value;
    node.setParam(name, value);
  }
  return value;
}
}  // namespace

double dept_err, beam_err;

VINA_SLAM::VINA_SLAM(const ros::NodeHandle& nh_in, const ros::NodeHandle& pnh_in) : nh(nh_in), pnh(pnh_in)
{
  double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
  vector<double> vecR(9), vecT(3);

  bagname = get_param_or(pnh, "General/bagname", std::string("noNameBag"));
  savepath = get_param_or(pnh, "General/save_path", std::string(""));
  lid_topic = get_param_or(pnh, "General/lid_topic", std::string("/rslidar_points"));
  imu_topic = get_param_or(pnh, "General/imu_topic", std::string("/imu"));
  feat.lidar_type = get_param_or(pnh, "General/lidar_type", 0);
  feat.blind = get_param_or(pnh, "General/blind", 0.1);
  feat.point_filter_num = get_param_or(pnh, "General/point_filter_num", 3);
  vecT = get_param_or(pnh, "General/extrinsic_tran", std::vector<double>(3, 0.0));
  vecR = get_param_or(pnh, "General/extrinsic_rota", std::vector<double>(9, 0.0));
  is_save_map = get_param_or(pnh, "General/is_save_map", 0);
  is_save_pose = get_param_or(pnh, "General/is_save_pose", 0);
  pose_save_path = get_param_or(pnh, "General/pose_save_path", std::string(""));
  pose_filename = get_param_or(pnh, "General/pose_filename", std::string("trajectory.txt"));
  if_BA = get_param_or(pnh, "General/if_BA", 0);
  enable_visualization = get_param_or(pnh, "General/enable_visualization", 0);

  if (is_save_map == 0)
  {
    std::cout << YELLOW << "[is_save_map]: don't save map" << RESET << std::endl;
  }
  else if (is_save_map == 1)
  {
    std::cout << GREEN << "[is_save_map]: save map" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[is_save_map]: ERROR STATE " << RESET << std::endl;
  }

  if (if_BA == 0)
  {
    std::cout << YELLOW << "[if_BA]: don't BA" << RESET << std::endl;
  }
  else if (if_BA == 1)
  {
    std::cout << GREEN << "[if_BA]: BA" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[if_BA]: ERROR STATE " << RESET << std::endl;
  }

  if (is_save_pose == 0)
  {
    std::cout << YELLOW << "[is_save_pose]: don't save pose" << RESET << std::endl;
  }
  else if (is_save_pose == 1)
  {
    std::cout << GREEN << "[is_save_pose]: save pose trajectory" << RESET << std::endl;
  }
  else
  {
    std::cout << RED << "[is_save_pose]: ERROR STATE " << RESET << std::endl;
  }

  sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 8000, imu_handler);

  if (feat.lidar_type == LIVOX)
  {
    sub_pcl_livox = nh.subscribe<livox_ros_driver::CustomMsg>(
        lid_topic, 1000, [](const livox_ros_driver::CustomMsg::ConstPtr& msg) { pcl_handler(msg); });
  }
  else
  {
    sub_pcl_standard = nh.subscribe<sensor_msgs::PointCloud2>(
        lid_topic, 1000, [](const sensor_msgs::PointCloud2::ConstPtr& msg) { pcl_handler(msg); });
  }
  odom_ekf.imu_topic = imu_topic;

  cov_gyr = get_param_or(pnh, "Odometry/cov_gyr", 0.1);
  cov_acc = get_param_or(pnh, "Odometry/cov_acc", 0.1);
  rand_walk_gyr = get_param_or(pnh, "Odometry/rdw_gyr", 1e-4);
  rand_walk_acc = get_param_or(pnh, "Odometry/rdw_acc", 1e-4);
  down_size = get_param_or(pnh, "Odometry/down_size", 0.1);
  dept_err = get_param_or(pnh, "Odometry/dept_err", 0.02);
  beam_err = get_param_or(pnh, "Odometry/beam_err", 0.05);
  voxel_size = get_param_or(pnh, "Odometry/voxel_size", 1.0);
  full_map_voxel_size = get_param_or(pnh, "General/full_map_voxel_size", 0.05);
  min_eigen_value = get_param_or(pnh, "Odometry/min_eigen_value", 0.0025);
  degrade_bound = get_param_or(pnh, "Odometry/degrade_bound", 100);
  point_notime = get_param_or(pnh, "Odometry/point_notime", 0);

  odom_ekf.point_notime = point_notime;

  feat.blind = feat.blind * feat.blind;
  odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
  odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
  odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
  odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
  odom_ekf.Lid_offset_to_IMU << vecT[0], vecT[1], vecT[2];
  odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2], vecR[3], vecR[4], vecR[5], vecR[6], vecR[7], vecR[8];
  extrin_para.R = odom_ekf.Lid_rot_to_IMU;
  extrin_para.p = odom_ekf.Lid_offset_to_IMU;
  min_point << 20, 20, 15, 10;

  win_size = get_param_or(pnh, "LocalBA/win_size", 10);
  max_layer = get_param_or(pnh, "LocalBA/max_layer", 2);
  cov_gyr = get_param_or(pnh, "LocalBA/cov_gyr", 0.1);
  cov_acc = get_param_or(pnh, "LocalBA/cov_acc", 0.1);
  rand_walk_gyr = get_param_or(pnh, "LocalBA/rdw_gyr", 1e-4);
  rand_walk_acc = get_param_or(pnh, "LocalBA/rdw_acc", 1e-4);
  min_ba_point = get_param_or(pnh, "LocalBA/min_ba_point", 20);
  plane_eigen_value_thre = get_param_or(pnh, "LocalBA/plane_eigen_value_thre", std::vector<double>({ 1, 1, 1, 1 }));
  imu_coef = get_param_or(pnh, "LocalBA/imu_coef", 1e-4);
  thread_num = get_param_or(pnh, "LocalBA/thread_num", 5);
  is_finish = get_param_or(pnh, "finish", false);

  for (double& iter : plane_eigen_value_thre)
  {
    iter = 1.0 / iter;
  }

  noiseMeas.setZero();
  noiseWalk.setZero();
  noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, cov_acc, cov_acc, cov_acc;
  noiseWalk.diagonal() << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, rand_walk_acc, rand_walk_acc, rand_walk_acc;

  std::string session_dir = savepath + bagname + "/";

  if (is_save_map == 1)
  {
    if (std::filesystem::exists(session_dir))
    {
      std::cout << BOLDRED << "[ERROR] Session directory already exists: " << session_dir << RESET << std::endl;
      std::cout << BOLDRED << "[ERROR] is_save_map=1, saving would overwrite existing data." << RESET << std::endl;
      std::cout << BOLDRED << "[ERROR] Please delete or rename the existing directory, or change 'bagname' in config."
                << RESET << std::endl;
      exit(1);
    }
    std::filesystem::create_directories(session_dir);
  }
  else if (!std::filesystem::exists(session_dir))
  {
    std::filesystem::create_directories(session_dir);
  }

  sws.resize(thread_num);
  cout << "bagname: " << bagname << endl;
}

int VINA_SLAM::initialization(deque<sensor_msgs::Imu::Ptr>& imus, Eigen::MatrixXd& hess,
                              LidarFactor& voxhess, PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
{
  static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
  static vector<double> beg_times;
  static vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;

  pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));

  if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
  {
    return 0;
  }

  if (win_count == 0)
  {
    imupre_scale_gravity = odom_ekf.scale_gravity;
  }

  PVecPtr pptr(new PVec);
  double downkd = down_size >= 0.5 ? down_size : 0.5;
  down_sampling_voxel(*pcl_curr, downkd);

  var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
  lio_state_estimation_kdtree(pptr);

  pwld.clear();
  pvec_update(pptr, x_curr, pwld);

  win_count++;
  x_buf.push_back(x_curr);
  pvec_buf.push_back(pptr);
  ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, 0, pcl_path);

  if (win_count > 1)
  {
    imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
    imu_pre_buf[win_count - 2]->push_imu(imus);
  }

  pcl::PointCloud<PointType> pl_mid = *orig;
  down_sampling_close(*orig, down_size);
  if (orig->size() < 1000)
  {
    *orig = pl_mid;
    down_sampling_close(*orig, down_size / 2);
  }

  sort(orig->begin(), orig->end(), [](PointType& x, PointType& y) { return x.curvature < y.curvature; });

  pl_origs.push_back(orig);
  beg_times.push_back(odom_ekf.pcl_beg_time);
  vec_imus.push_back(imus);

  int is_success = 0;

  if (win_count >= win_size)
  {
    is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map,
                                                        surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf,
                                                        extrin_para);

    if (is_success == 0)
    {
      return -1;
    }
    return 1;
  }

  return 0;
}

void VINA_SLAM::system_reset(deque<sensor_msgs::Imu::Ptr>& imus)
{
  for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
  {
    iter->second->tras_ptr(octos_release);
    iter->second->clear_slwd(sws[0]);
    delete iter->second;
  }
  surf_map.clear();
  surf_map_slide.clear();

  x_curr.setZero();
  x_curr.p = Eigen::Vector3d(0, 0, 30);

  odom_ekf.mean_acc.setZero();
  odom_ekf.init_num = 0;
  odom_ekf.IMU_init(imus);
  x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;

  for (int i = 0; i < imu_pre_buf.size(); i++)
  {
    delete imu_pre_buf[i];
  }

  x_buf.clear();
  pvec_buf.clear();
  imu_pre_buf.clear();
  pl_tree->clear();

  for (int i = 0; i < win_size; i++)
  {
    mp[i] = i;
  }
  win_base = 0;
  win_count = 0;

  pcl_path.clear();
  pub_pl_func(pcl_path, pub_cmap);

  std::cout << "\033[31mReset\033[0m" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vina_slam");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pub_cmap = nh.advertise<sensor_msgs::PointCloud2>("/map_cmap", 100);
  pub_scan = nh.advertise<sensor_msgs::PointCloud2>("/map_scan", 100);
  pub_curr_path = nh.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_voxel_plane = nh.advertise<visualization_msgs::MarkerArray>("/voxel_plane", 10);
  pub_voxel_normal = nh.advertise<visualization_msgs::MarkerArray>("/voxel_normal", 10);

  ResultOutput::instance();
  FileReaderWriter::instance();
  Initialization::instance();
  VINA_SLAM vs(nh, pnh);

  mp.resize(vs.win_size);
  for (int i = 0; i < mp.size(); i++)
  {
    mp[i] = i;
  }

  std::thread thread_odom(&VINA_SLAM::thd_odometry_localmapping, &vs);

  ros::spin();

  thread_odom.join();

  return 0;
}
