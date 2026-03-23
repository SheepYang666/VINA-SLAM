// VINA_SLAM class implementation and main() function
// Moved from VINASlam.cpp

#include "vina_slam/platform/ros2/node.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/subscribers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace
{
template <typename T>
void declare_if_not(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  if (node && !node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }
}
}  // namespace

// Global variables
double dept_err, beam_err;

VINA_SLAM& VINA_SLAM::instance(const rclcpp::Node::SharedPtr& node_in)
{
  static VINA_SLAM inst(node_in);
  return inst;
}

VINA_SLAM& VINA_SLAM::instance()
{
  rclcpp::Node::SharedPtr node_temp;
  return instance(node_temp);
}

VINA_SLAM::VINA_SLAM(const rclcpp::Node::SharedPtr& node_in) : node(node_in)
{
  double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
  vector<double> vecR(9), vecT(3);

  bagname = node->declare_parameter("General.bagname", "noNameBag");
  node->get_parameter("General.bagname", bagname);

  savepath = node->declare_parameter("General.save_path", "");
  node->get_parameter("General.save_path", savepath);

  lid_topic = node->declare_parameter("General.lid_topic", "/rslidar_points");
  node->get_parameter("General.lid_topic", lid_topic);

  imu_topic = node->declare_parameter("General.imu_topic", "/imu");
  node->get_parameter("General.imu_topic", imu_topic);

  feat.lidar_type = node->declare_parameter("General.lidar_type", 0);
  node->get_parameter("General.lidar_type", feat.lidar_type);

  feat.blind = node->declare_parameter("General.blind", 0.1);
  node->get_parameter("General.blind", feat.blind);

  feat.point_filter_num = node->declare_parameter("General.point_filter_num", 3);
  node->get_parameter("General.point_filter_num", feat.point_filter_num);

  vecT = node->declare_parameter("General.extrinsic_tran", std::vector<double>(3, 0.0));
  node->get_parameter("General.extrinsic_tran", vecT);

  vecR = node->declare_parameter("General.extrinsic_rota", std::vector<double>(9, 0.0));
  node->get_parameter("General.extrinsic_rota", vecR);

  is_save_map = node->declare_parameter<int>("General.is_save_map", false);
  node->get_parameter("General.is_save_map", is_save_map);

  if_BA = node->declare_parameter<int>("General.if_BA", false);
  node->get_parameter("General.if_BA", if_BA);

  enable_visualization = node->declare_parameter<int>("General.enable_visualization", 0);
  node->get_parameter("General.enable_visualization", enable_visualization);

  // ######################################## print log ########################################

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

  // ######################################## print log ########################################

  rclcpp::QoS imu_qos(8000);
  imu_qos.keep_last(8000);
  imu_qos.best_effort();

  rclcpp::QoS pcl_qos(1000);
  pcl_qos.keep_last(1000);
  pcl_qos.best_effort();

  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_handler(msg); });

  if (feat.lidar_type == LIVOX)
  {
    sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { pcl_handler(msg); });
  }
  else
  {
    sub_pcl_standard = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, pcl_qos,
        [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcl_handler(msg); });
  }
  odom_ekf.imu_topic = imu_topic;

  // Odometry param
  cov_gyr = node->declare_parameter<double>("Odometry.cov_gyr", 0.1);
  node->get_parameter("Odometry.cov_gyr", cov_gyr);

  cov_acc = node->declare_parameter<double>("Odometry.cov_acc", 0.1);
  node->get_parameter("Odometry.cov_acc", cov_acc);

  rand_walk_gyr = node->declare_parameter<double>("Odometry.rdw_gyr", 1e-4);
  node->get_parameter("Odometry.rdw_gyr", rand_walk_gyr);

  rand_walk_acc = node->declare_parameter<double>("Odometry.rdw_acc", 1e-4);
  node->get_parameter("Odometry.rdw_acc", rand_walk_acc);

  down_size = node->declare_parameter<double>("Odometry.down_size", 0.1);
  node->get_parameter("Odometry.down_size", down_size);

  dept_err = node->declare_parameter<double>("Odometry.dept_err", 0.02);
  node->get_parameter("Odometry.dept_err", dept_err);

  beam_err = node->declare_parameter<double>("Odometry.beam_err", 0.05);
  node->get_parameter("Odometry.beam_err", beam_err);

  voxel_size = node->declare_parameter<double>("Odometry.voxel_size", 1.0);
  node->get_parameter("Odometry.voxel_size", voxel_size);

  full_map_voxel_size = node->declare_parameter<double>("General.full_map_voxel_size", 0.05);
  node->get_parameter("General.full_map_voxel_size", full_map_voxel_size);

  min_eigen_value = node->declare_parameter<double>("Odometry.min_eigen_value", 0.0025);
  node->get_parameter("Odometry.min_eigen_value", min_eigen_value);

  degrade_bound = node->declare_parameter<int>("Odometry.degrade_bound", 100);
  node->get_parameter("Odometry.degrade_bound", degrade_bound);

  point_notime = node->declare_parameter<int>("Odometry.point_notime", 0);
  node->get_parameter("Odometry.point_notime", point_notime);

  odom_ekf.point_notime = point_notime;

  // Other parameters and initialization
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

  // LocalBA param
  win_size = node->declare_parameter<int>("LocalBA.win_size", 10);
  node->get_parameter("LocalBA.win_size", win_size);

  max_layer = node->declare_parameter<int>("LocalBA.max_layer", 2);
  node->get_parameter("LocalBA.max_layer", max_layer);

  cov_gyr = node->declare_parameter<double>("LocalBA.cov_gyr", 0.1);
  node->get_parameter("LocalBA.cov_gyr", cov_gyr);

  cov_acc = node->declare_parameter<double>("LocalBA.cov_acc", 0.1);
  node->get_parameter("LocalBA.cov_acc", cov_acc);

  rand_walk_gyr = node->declare_parameter<double>("LocalBA.rdw_gyr", 1e-4);
  node->get_parameter("LocalBA.rdw_gyr", rand_walk_gyr);

  rand_walk_acc = node->declare_parameter<double>("LocalBA.rdw_acc", 1e-4);
  node->get_parameter("LocalBA.rdw_acc", rand_walk_acc);

  min_ba_point = node->declare_parameter<int>("LocalBA.min_ba_point", 20);
  node->get_parameter("LocalBA.min_ba_point", min_ba_point);

  plane_eigen_value_thre = node->declare_parameter<std::vector<double>>("LocalBA.plane_eigen_value_thre",
                                                                        std::vector<double>({ 1, 1, 1, 1 }));
  node->get_parameter("LocalBA.plane_eigen_value_thre", plane_eigen_value_thre);

  imu_coef = node->declare_parameter<double>("LocalBA.imu_coef", 1e-4);
  node->get_parameter("LocalBA.imu_coef", imu_coef);

  thread_num = node->declare_parameter<int>("LocalBA.thread_num", 5);
  node->get_parameter("LocalBA.thread_num", thread_num);

  is_finish = node->declare_parameter<bool>("finish", false);
  node->get_parameter("finish", is_finish);

  for (double& iter : plane_eigen_value_thre)
  {
    iter = 1.0 / iter;
  }

  // Noise matrix initialization
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
  else
  {
    if (!std::filesystem::exists(session_dir))
    {
      std::filesystem::create_directories(session_dir);
    }
  }

  sws.resize(thread_num);
  cout << "bagname: " << bagname << endl;
}

int VINA_SLAM::initialization(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, Eigen::MatrixXd& hess,
                              LidarFactor& voxhess, PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
{
  static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
  static vector<double> beg_times;
  static vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>> vec_imus;

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
    else
    {
      return 1;
    }
  }

  return 0;
}

void VINA_SLAM::system_reset(deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus)
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
  pub_pl_func(pcl_path, pub_cmap, node);

  std::cout << "\033[31mReset\033[0m" << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("vina_slam");
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  exec->add_node(node);

  pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cmap", 100);

  pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan", 100);
  pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_path", 100);
  pub_voxel_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_plane", 10);
  pub_voxel_normal = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_normal", 10);

  ResultOutput::instance(node);
  FileReaderWriter::instance(node);
  Initialization::instance(node);
  VINA_SLAM vs(node);

  mp.resize(vs.win_size);
  for (int i = 0; i < mp.size(); i++)
  {
    mp[i] = i;
  }

  std::thread thread_odom(&VINA_SLAM::thd_odometry_localmapping, &vs, node);

  exec->spin();

  thread_odom.join();

  return 0;
}
