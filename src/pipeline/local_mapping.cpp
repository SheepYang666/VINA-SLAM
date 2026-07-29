// Local mapping methods of VINA_SLAM class
// Moved from VINASlam.cpp: multi_margi(), multi_recut() (3 overloads), run_odometry_local_mapping_loop()

#include "vina_slam/core/constants.hpp"
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
#include <malloc.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unistd.h>

namespace
{
constexpr std::size_t kDefaultVoxelMarkerReserve = 1024;
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

void VINA_SLAM::run_odometry_local_mapping_loop(std::shared_ptr<rclcpp::Node> node)
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
  const std::string map_frame_save_dir = savepath + bagname + "/pcd";

  auto save_optimized_frames = [&](int frame_count) {
    const int count = std::min(frame_count, static_cast<int>(std::min(x_buf.size(), frame_cloud_buf.size())));
    for (int i = 0; i < count; ++i)
    {
      const int frame_id = win_base + i;
      const IMUST& optimized_pose = x_buf[i];

      if (is_save_map == 1)
      {
        FileReaderWriter::instance().save_frame_pcd(*frame_cloud_buf[i], optimized_pose, extrin_para, frame_id,
                                                    map_frame_save_dir);
      }

      if (is_save_pose == 1)
      {
        FileReaderWriter::instance().save_pose_tum(optimized_pose);
      }
    }
  };

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

    if (motion_init_flag)
    {
      int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

      if (init == 1)
      {
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
        continue;
      }
      pcl::PointCloud<PointType>::Ptr frame_cloud(new pcl::PointCloud<PointType>(*pcl_curr));

      pcl::PointCloud<PointType> pl_down = *pcl_curr;
      down_sampling_voxel(pl_down, down_size);

      if (pl_down.size() < 2000)
      {
        pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size / 2);
      }

      PVecPtr pptr(new PVec);
      var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

      lio_state_estimation(pptr);

      pwld.clear();
      pvec_update(pptr, x_curr, pwld);
      ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, 0, pcl_path);

      win_count++;
      x_buf.push_back(x_curr);
      pvec_buf.push_back(pptr);
      frame_cloud_buf.push_back(frame_cloud);
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

      multi_recut(surf_map_slide, win_count, x_buf, voxhess, normalFactor, sws);

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

    }

    if (win_count >= win_size)
    {
      LI_BA_Optimizer opt_lsv;

      if (if_BA == 1)
      {
        // opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        opt_lsv.damping_iter(x_buf, voxhess, normalFactor, imu_pre_buf, &hess);
      }

      // Persist the frame that is leaving the sliding window after the optional local BA update.
      // This makes each saved PCD and TUM row use the final optimized pose available for that frame.
      save_optimized_frames(mgsize);

      x_last = x_curr;

      x_curr.R = x_buf[win_count - 1].R;
      x_curr.p = x_buf[win_count - 1].p;

      ResultOutput::instance().pub_localmap(mgsize, 0, pvec_buf, x_buf, pcl_path, win_base, win_count);

      multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);

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
        pcl::PointCloud<PointType>::Ptr frame_cloud_tem = frame_cloud_buf[i - mgsize];
        frame_cloud_buf[i - mgsize] = frame_cloud_buf[i];
        frame_cloud_buf[i] = frame_cloud_tem;
      }

      for (int i = win_count - mgsize; i < win_count; i++)
      {
        x_buf.pop_back();
        pvec_buf.pop_back();
        frame_cloud_buf.pop_back();

        delete imu_pre_buf.front();
        imu_pre_buf.pop_front();
      }

      win_base += mgsize;
      win_count -= mgsize;
    }
  }

  // Save any frames still held by the final sliding window. They have already passed through
  // the same local BA stage as the last published window state.
  save_optimized_frames(win_count);
  frame_cloud_buf.clear();

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

  rclcpp::shutdown();
}
