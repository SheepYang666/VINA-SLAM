// Local mapping methods of VINA_SLAM class
// Moved from VINASlam.cpp: multi_margi(), multi_recut() (3 overloads), thd_odometry_localmapping()

#include "vina_slam/platform/ros2/node.hpp"
#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/mapping/voxel_map.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <malloc.h>
#include <thread>
#include <unistd.h>

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
  double down_sizes[3] = { 0.1, 0.2, 0.4 };
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
          if (dis < 20)
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

      pcl::PointCloud<PointType> pl_down = *pcl_curr;
      down_sampling_voxel(pl_down, down_size);

      if (pl_down.size() < 2000)
      {
        pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size / 2);
      }

      PVecPtr pptr(new PVec);
      var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

      auto pcl_curr_temp = *pcl_curr;
      PVecPtr no_ds_pptr(new PVec);
      var_init(extrin_para, pcl_curr_temp, no_ds_pptr, dept_err, beam_err);

      if (lio_state_estimation(pptr))
      {
        if (degrade_cnt > 0)
        {
          degrade_cnt--;
        }
      }
      else
      {
        degrade_cnt++;
      }

      pwld.clear();
      pvec_update(pptr, x_curr, pwld);
      ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, 0, pcl_path);

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
      if (enable_visualization)
      {
        visualization_msgs::msg::MarkerArray voxel_plane;
        visualization_msgs::msg::MarkerArray voxel_normal;
        std::unordered_set<int> voxel_plane_ids;
        std::unordered_set<int> voxel_normal_ids;
        for (auto& kv : surf_map_slide)
        {
          if (kv.second)
          {
            kv.second->collect_plane_markers(voxel_plane, max_layer, voxel_plane_ids);
            kv.second->collect_normal_markers(voxel_normal, max_layer, voxel_normal_ids);
          }
        }
        pub_voxel_plane->publish(voxel_plane);
        pub_voxel_normal->publish(voxel_normal);
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
      if (if_BA == 1)
      {
        LI_BA_Optimizer opt_lsv;

        opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
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
