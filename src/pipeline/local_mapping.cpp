// Local mapping methods of VINA_SLAM class
// Moved from VINASlam.cpp: multi_margi(), multi_recut() (3 overloads), thd_odometry_localmapping()

#include "vina_slam/core/constants.hpp"
#include "vina_slam/core/debug_logging.hpp"
#include "vina_slam/platform/ros2/node.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/platform/ros2/io.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/mapping/voxel_map.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <malloc.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace
{
constexpr std::size_t kDefaultVoxelMarkerReserve = 1024;

void write_frontend_z_drift_header(std::ofstream& stream)
{
  stream << "frame_id,pcl_beg_time,imu_count,raw_point_count,downsampled_point_count,"
            "z_after_imu,z_after_lio,delta_z_lio,delta_norm_lio,lio_success,last_match_num,"
            "nnt_eig0,nnt_eig1,nnt_eig2,nnt_min_dir_x,nnt_min_dir_y,nnt_min_dir_z,"
            "iterations,frontend_path,x_curr_modified_no_rollback\n";
}

void write_ba_z_drift_header(std::ofstream& stream)
{
  stream << "frame_id,timestamp,ba_enabled,window_size,lidar_factor_count,normal_factor_count,imu_factor_count,"
            "z_before_ba,z_after_ba,delta_z_ba,delta_norm_ba,"
            "imu_res_before,lidar_res_before,normal_res_before,total_res_before,"
            "imu_res_after,lidar_res_after,normal_res_after,total_res_after\n";
}
}

void VINA_SLAM::multi_margi(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, double jour, int win_count,
                            vector<IMUST>& xs, LidarFactor& voxopt, vector<SlideWindow*>& sw)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>*> octs;

  for (int i = 0; i < thd_num; i++)
    octs.push_back(new vector<OctoTree*>());

  int g_size = feat_map.size();
  if (g_size < thd_num)
    return;

  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;
  int cnt = 0;

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    iter->second->jour = jour;
    octs[cnt]->push_back(iter->second);
    if (octs[cnt]->size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  auto margi_func = [](int win_cnt, vector<OctoTree*>* oct, vector<IMUST> xxs, LidarFactor& voxhess) {
    for (OctoTree* oc : *oct)
    {
      oc->margi(win_cnt, 1, xxs, voxhess);
    }
  };

  for (int i = 1; i < thd_num; i++)
  {
    mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
  }

  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      margi_func(win_count, octs[i], xs, voxopt);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

  for (auto iter = feat_map.begin(); iter != feat_map.end();)
  {
    if (iter->second->isexist)
    {
      iter++;
    }
    else
    {
      iter->second->clear_slwd(sw);
      feat_map.erase(iter++);
    }
  }

  for (int i = 0; i < thd_num; i++)
  {
    delete octs[i];
  }
}

void VINA_SLAM::multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                            LidarFactor& voxopt, vector<vector<SlideWindow*>>& sws)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>> octss(thd_num);

  int g_size = feat_map.size();
  if (g_size < thd_num)
  {
    return;
  }

  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;
  int cnt = 0;

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    octss[cnt].push_back(iter->second);
    if (octss[cnt].size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  auto recut_func = [](int win_count, vector<OctoTree*>& oct, vector<IMUST> xxs, vector<SlideWindow*>& sw) {
    for (OctoTree* oc : oct)
    {
      oc->recut(win_count, xxs, sw);
    }
  };

  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));

  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      recut_func(win_count, octss[i], xs, sws[i]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

  for (int i = 1; i < sws.size(); i++)
  {
    sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
    sws[i].clear();
  }

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    iter->second->tras_opt(voxopt);
  }
}

void VINA_SLAM::multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                            LidarFactor& lidarFactor, NormalFactor& normalFactor, vector<vector<SlideWindow*>>& sws)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>> octss(thd_num);

  int g_size = feat_map.size();
  if (g_size < thd_num)
  {
    return;
  }

  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;
  int cnt = 0;

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    octss[cnt].push_back(iter->second);
    if (octss[cnt].size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  auto recut_func = [](int win_count, vector<OctoTree*>& oct, vector<IMUST> xxs, vector<SlideWindow*>& sw) {
    for (OctoTree* oc : oct)
    {
      oc->recut(win_count, xxs, sw);
    }
  };

  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));

  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      recut_func(win_count, octss[i], xs, sws[i]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

  for (int i = 1; i < sws.size(); i++)
  {
    sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
    sws[i].clear();
  }

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    iter->second->tras_opt(lidarFactor);
    iter->second->tras_opt(normalFactor);
  }
}

void VINA_SLAM::multi_recut(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, int win_count, vector<IMUST>& xs,
                            vector<vector<SlideWindow*>>& sws)
{
  int thd_num = thread_num;
  vector<vector<OctoTree*>> octss(thd_num);

  int g_size = feat_map.size();
  if (g_size < thd_num)
  {
    return;
  }

  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;
  int cnt = 0;

  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
  {
    octss[cnt].push_back(iter->second);
    if (octss[cnt].size() >= part && cnt < thd_num - 1)
      cnt++;
  }

  auto recut_func = [](int win_count, vector<OctoTree*>& oct, vector<IMUST> xxs, vector<SlideWindow*>& sw) {
    for (OctoTree* oc : oct)
    {
      oc->recut(win_count, xxs, sw);
    }
  };

  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));

  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      recut_func(win_count, octss[i], xs, sws[i]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }

  for (int i = 1; i < sws.size(); i++)
  {
    sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
    sws[i].clear();
  }
}

// The main thread of odometry and local mapping

void VINA_SLAM::thd_odometry_localmapping(std::shared_ptr<rclcpp::Node> node)
{
  PLV(3) pwld;
  double down_sizes[3] = { 0.1, 0.1, 0.1 };
  Eigen::Vector3d last_pos(0, 0, 0);
  double jour = 0.0;
  int counter = 0;

  pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
  int motion_init_flag = 1;
  pl_tree.reset(new pcl::PointCloud<PointType>());
  vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
  vector<double> beg_times;
  vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>> vec_imus;
  bool release_flag = false;
  int degrade_cnt = 0;

  LidarFactor voxhess(win_size);
  NormalFactor normalFactor(win_size);

  const int mgsize = 1;
  Eigen::MatrixXd hess;
  static IMUST x_last;
  bool has_visualization_publish_time = false;
  double last_visualization_publish_time = 0.0;
  std::size_t last_voxel_plane_marker_count = kDefaultVoxelMarkerReserve;
  std::size_t last_voxel_normal_marker_count = kDefaultVoxelMarkerReserve;

  if (is_save_pose == 1)
  {
    std::string dir = pose_save_path.empty() ? (savepath + bagname + "/") : pose_save_path;
    if (!dir.empty() && dir.back() != '/')
      dir += '/';
    FileReaderWriter::instance().init_pose_file(dir + pose_filename);
  }

  std::ofstream z_drift_frontend_log;
  std::ofstream z_drift_ba_log;
  if (debug_enable_z_drift_log)
  {
    const std::string run_label = debug_run_label.empty() ? bagname : debug_run_label;
    const auto log_paths = vina_slam::core::make_z_drift_log_paths(debug_log_root, run_label);
    try
    {
      std::filesystem::create_directories(log_paths.run_dir);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "[Debug.z_drift_log] Cannot create log directory '%s': %s",
                   log_paths.run_dir.string().c_str(), e.what());
    }

    z_drift_frontend_log.open(log_paths.frontend_csv, std::ios::out | std::ios::trunc);
    if (z_drift_frontend_log.is_open())
    {
      z_drift_frontend_log << std::fixed << std::setprecision(9);
      write_frontend_z_drift_header(z_drift_frontend_log);
      RCLCPP_INFO(node->get_logger(), "[Debug.z_drift_log] Frontend CSV: %s", log_paths.frontend_csv.string().c_str());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "[Debug.z_drift_log] Cannot open frontend CSV: %s",
                   log_paths.frontend_csv.string().c_str());
    }

    z_drift_ba_log.open(log_paths.ba_csv, std::ios::out | std::ios::trunc);
    if (z_drift_ba_log.is_open())
    {
      z_drift_ba_log << std::fixed << std::setprecision(9);
      write_ba_z_drift_header(z_drift_ba_log);
      RCLCPP_INFO(node->get_logger(), "[Debug.z_drift_log] BA CSV: %s", log_paths.ba_csv.string().c_str());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "[Debug.z_drift_log] Cannot open BA CSV: %s",
                   log_paths.ba_csv.string().c_str());
    }
  }

  while (rclcpp::ok())
  {
    node->get_parameter("finish", is_finish);

    if (is_finish)
    {
      break;
    }

    // Synchronize IMU and point cloud data
    deque<std::shared_ptr<sensor_msgs::msg::Imu>> imus;
    bool if_sync_packages = sync_packages(pcl_curr, imus, odom_ekf);

    if (!if_sync_packages)
    {
      if (octos_release.size() != 0)
      {
        int msize = octos_release.size();
        msize = (msize > 1000) ? 1000 : msize;
        for (int i = 0; i < msize; i++)
        {
          delete octos_release.back();
          octos_release.pop_back();
        }

        malloc_trim(0);
      }
      else if (release_flag)
      {
        release_flag = false;
        vector<OctoTree*> octos;
        for (auto iter = surf_map.begin(); iter != surf_map.end();)
        {
          int dis = jour - iter->second->jour;
          if (dis < 700)
          {
            iter++;
          }
          else
          {
            octos.push_back(iter->second);
            iter->second->tras_ptr(octos);
            surf_map.erase(iter++);
          }
        }

        int ocsize = octos.size();
        for (int i = 0; i < ocsize; i++)
        {
          delete octos[i];
        }

        octos.clear();
        malloc_trim(0);
      }
      else if (sws[0].size() > 10000)
      {
        for (int i = 0; i < 500; i++)
        {
          delete sws[0].back();
          sws[0].pop_back();
        }

        malloc_trim(0);
      }
      usleep(1000);
      continue;
    }

    double t0 = node->now().seconds();
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;

    if (motion_init_flag)
    {
      if (pcl_curr->empty())
      {
        RCLCPP_WARN(node->get_logger(), "pcl_curr is empty or null");
      }

      int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

      if (init == 1)
      {
        std::cout << RED << "init success" << RESET << std::endl;

        motion_init_flag = 0;
      }
      else
      {
        if (init == -1)
        {
          system_reset(imus);
        }

        continue;
      }
    }
    else
    {
      if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      {
        std::cout << RED << "motion blur failed" << RESET << std::endl;

        continue;
      }

      const IMUST x_after_imu = x_curr;
      const std::size_t raw_point_count = pcl_curr->size();

      pcl::PointCloud<PointType> pl_down = *pcl_curr;
      down_sampling_voxel(pl_down, down_size);

      if (pl_down.size() < 2000)
      {
        pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size / 2);
      }

      PVecPtr pptr(new PVec);
      var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

      const int frame_id = win_base + win_count;
      const bool lio_success = lio_state_estimation(pptr);
      const IMUST x_after_lio = x_curr;
      const Eigen::Vector3d delta_lio = x_after_lio.p - x_after_imu.p;

      if (z_drift_frontend_log.is_open())
      {
        z_drift_frontend_log << frame_id << "," << odom_ekf.pcl_beg_time << "," << imus.size() << ","
                             << raw_point_count << "," << pl_down.size() << "," << x_after_imu.p.z() << ","
                             << x_after_lio.p.z() << "," << delta_lio.z() << "," << delta_lio.norm() << ","
                             << static_cast<int>(lio_success) << "," << last_lio_debug_stats.last_match_num << ","
                             << last_lio_debug_stats.nnt_eigenvalues[0] << ","
                             << last_lio_debug_stats.nnt_eigenvalues[1] << ","
                             << last_lio_debug_stats.nnt_eigenvalues[2] << ","
                             << last_lio_debug_stats.nnt_min_direction.x() << ","
                             << last_lio_debug_stats.nnt_min_direction.y() << ","
                             << last_lio_debug_stats.nnt_min_direction.z() << ","
                             << last_lio_debug_stats.iterations << ",lio_state_estimation,1\n";
        z_drift_frontend_log.flush();
      }

      if (lio_success)
      {
        if (degrade_cnt > 0)
        {
          degrade_cnt--;
          std::cout << RED << "degrade\n";
        }
      }
      else
      {
        // degrade_cnt++;
        if (debug_fail_on_frontend_degenerate)
        {
          RCLCPP_ERROR(node->get_logger(),
                       "[Debug.z_drift_log] Frontend degenerate at frame %d; x_curr was already updated and no "
                       "rollback is applied",
                       frame_id);
          throw std::runtime_error("Debug.fail_on_frontend_degenerate: lio_state_estimation returned false");
        }
      }

      pwld.clear();
      pvec_update(pptr, x_curr, pwld);
      ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, 0, pcl_path);

      if (is_save_pose == 1)
        FileReaderWriter::instance().save_pose_tum(x_curr);

      t1 = node->now().seconds();

      win_count++;
      x_buf.push_back(x_curr);
      pvec_buf.push_back(pptr);
      if (win_count > 1)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
        imu_pre_buf[win_count - 2]->push_imu(imus);
      }

      voxhess.clear();
      voxhess.win_size = win_size;
      normalFactor.clear();
      normalFactor.win_size = win_size;

      cut_voxel_multi(surf_map, pvec_buf[win_count - 1], win_count - 1, surf_map_slide, win_size, pwld, sws);
      t2 = node->now().seconds();

      multi_recut(surf_map_slide, win_count, x_buf, voxhess, normalFactor, sws);
      t3 = node->now().seconds();

      // Publish voxel plane and normal markers if visualization is enabled
      if (enable_visualization && (pub_voxel_plane || pub_voxel_normal))
      {
        const bool publish_plane = pub_voxel_plane && pub_voxel_plane->get_subscription_count() > 0;
        const bool publish_normal = pub_voxel_normal && pub_voxel_normal->get_subscription_count() > 0;

        if (publish_plane || publish_normal)
        {
          const double now = node->now().seconds();
          const bool should_publish = !has_visualization_publish_time || visualization_publish_hz <= 0.0 ||
                                      now - last_visualization_publish_time >= 1.0 / visualization_publish_hz;

          if (should_publish)
          {
            visualization_msgs::msg::MarkerArray voxel_plane;
            visualization_msgs::msg::MarkerArray voxel_normal;
            std::unordered_set<int> voxel_plane_ids;
            std::unordered_set<int> voxel_normal_ids;

            if (publish_plane)
            {
              const std::size_t reserve_count = std::max(kDefaultVoxelMarkerReserve, last_voxel_plane_marker_count);
              voxel_plane.markers.reserve(reserve_count);
              voxel_plane_ids.reserve(reserve_count);
            }

            if (publish_normal)
            {
              const std::size_t reserve_count = std::max(kDefaultVoxelMarkerReserve, last_voxel_normal_marker_count);
              voxel_normal.markers.reserve(reserve_count);
              voxel_normal_ids.reserve(reserve_count);
            }

            const int marker_max_layer = visualization_max_layer >= 0 ? visualization_max_layer : max_layer;
            for (auto& kv : surf_map_slide)
            {
              if (kv.second)
              {
                if (publish_plane)
                  kv.second->collect_plane_markers(voxel_plane, marker_max_layer, voxel_plane_ids);
                if (publish_normal)
                  kv.second->collect_normal_markers(voxel_normal, marker_max_layer, voxel_normal_ids);
              }
            }

            if (publish_plane)
            {
              last_voxel_plane_marker_count = std::max(kDefaultVoxelMarkerReserve, voxel_plane.markers.size());
              pub_voxel_plane->publish(voxel_plane);
            }

            if (publish_normal)
            {
              last_voxel_normal_marker_count = std::max(kDefaultVoxelMarkerReserve, voxel_normal.markers.size());
              pub_voxel_normal->publish(voxel_normal);
            }

            last_visualization_publish_time = now;
            has_visualization_publish_time = true;
          }
        }
      }

      auto x_temp = x_curr;
      if (0)
      {
        std::cout << "Degrade " << std::endl;

        degrade_cnt = 0;
        system_reset(imus);

        last_pos = x_curr.p;
        jour = 0;

        motion_init_flag = 1;
        continue;
      }
    }

    if (win_count >= win_size)
    {
      t4 = node->now().seconds();
      LI_BA_Optimizer opt_lsv;
      const int ba_frame_id = win_base + win_count - 1;
      const double ba_timestamp = x_buf[win_count - 1].t;
      const Eigen::Vector3d p_before_ba = x_buf[win_count - 1].p;
      double imu_res_before = std::numeric_limits<double>::quiet_NaN();
      double lidar_res_before = std::numeric_limits<double>::quiet_NaN();
      double normal_res_before = std::numeric_limits<double>::quiet_NaN();
      double total_res_before = std::numeric_limits<double>::quiet_NaN();
      double imu_res_after = std::numeric_limits<double>::quiet_NaN();
      double lidar_res_after = std::numeric_limits<double>::quiet_NaN();
      double normal_res_after = std::numeric_limits<double>::quiet_NaN();
      double total_res_after = std::numeric_limits<double>::quiet_NaN();

      if (z_drift_ba_log.is_open())
      {
        opt_lsv.evaluate_breakdown(x_buf, voxhess, normalFactor, imu_pre_buf, imu_res_before, lidar_res_before,
                                   normal_res_before, total_res_before);
      }

      if (if_BA == 1)
      {
        // opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        opt_lsv.damping_iter(x_buf, voxhess, normalFactor, imu_pre_buf, &hess);
      }

      const Eigen::Vector3d p_after_ba = x_buf[win_count - 1].p;
      if (z_drift_ba_log.is_open())
      {
        opt_lsv.evaluate_breakdown(x_buf, voxhess, normalFactor, imu_pre_buf, imu_res_after, lidar_res_after,
                                   normal_res_after, total_res_after);
        const Eigen::Vector3d delta_ba = p_after_ba - p_before_ba;
        z_drift_ba_log << ba_frame_id << "," << ba_timestamp << "," << static_cast<int>(if_BA == 1) << ","
                       << win_count << "," << voxhess.plvec_voxels.size() << ","
                       << normalFactor.plvec_voxels.size() << "," << imu_pre_buf.size() << ","
                       << p_before_ba.z() << "," << p_after_ba.z() << "," << delta_ba.z() << ","
                       << delta_ba.norm() << "," << imu_res_before << "," << lidar_res_before << ","
                       << normal_res_before << "," << total_res_before << "," << imu_res_after << ","
                       << lidar_res_after << "," << normal_res_after << "," << total_res_after << "\n";
        z_drift_ba_log.flush();
      }

      x_last = x_curr;

      x_curr.R = x_buf[win_count - 1].R;
      x_curr.p = x_buf[win_count - 1].p;
      t5 = node->now().seconds();

      ResultOutput::instance().pub_localmap(mgsize, 0, pvec_buf, x_buf, pcl_path, win_base, win_count);

      multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
      t6 = node->now().seconds();

      if ((win_base + win_count) % 10 == 0)
      {
        double spat = (x_curr.p - last_pos).norm();
        if (spat > 0.5)
        {
          jour += spat;
          last_pos = x_curr.p;
          release_flag = true;
        }
      }

      for (int i = 0; i < win_size; i++)
      {
        mp[i] += mgsize;
        if (mp[i] >= win_size)
          mp[i] -= win_size;
      }

      for (int i = mgsize; i < win_count; i++)
      {
        x_buf[i - mgsize] = x_buf[i];
        PVecPtr pvec_tem = pvec_buf[i - mgsize];
        pvec_buf[i - mgsize] = pvec_buf[i];
        pvec_buf[i] = pvec_tem;
      }

      for (int i = win_count - mgsize; i < win_count; i++)
      {
        x_buf.pop_back();
        pvec_buf.pop_back();

        delete imu_pre_buf.front();
        imu_pre_buf.pop_front();
      }

      win_base += mgsize;
      win_count -= mgsize;
    }
    double t_end = node->now().seconds();
    double mem = get_memory();
  }

  vector<OctoTree*> octos;
  for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
  {
    iter->second->tras_ptr(octos);
    iter->second->clear_slwd(sws[0]);
    delete iter->second;
  }

  for (int i = 0; i < octos.size(); i++)
  {
    delete octos[i];
  }
  octos.clear();

  for (int i = 0; i < sws[0].size(); i++)
  {
    delete sws[0][i];
  }
  sws[0].clear();
  malloc_trim(0);
}
