#include "vina_slam/mapping/voxel_map.hpp"
#include <thread>

void cut_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVecPtr pvec, int win_count,
               std::unordered_map<VOXEL_LOC, OctoTree*>& feat_tem_map, int wdsize, PLV(3) & pwld,
               std::vector<SlideWindow*>& sws)
{
  int plsize = pvec->size();
  for (int i = 0; i < plsize; i++)
  {
    pointVar& pv = (*pvec)[i];
    Eigen::Vector3d& pw = pwld[i];
    float loc[3];
    for (int j = 0; j < 3; j++)
    {
      loc[j] = pw[j] / voxel_size;
      if (loc[j] < 0)
        loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter_feat_map = feat_map.find(position);
    auto iter_feat_tem_map = feat_map.find(position);
    if (iter_feat_map != feat_map.end())
    {
      iter_feat_map->second->allocate(win_count, pv, pw, sws);
      iter_feat_map->second->isexist = true;
      if (feat_tem_map.find(position) == feat_tem_map.end())
      {
        feat_tem_map[position] = iter_feat_map->second;
      }
    }
    else
    {
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->allocate(win_count, pv, pw, sws);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }
  }
}

void cut_voxel_multi(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVecPtr pvec, int win_count,
                     std::unordered_map<VOXEL_LOC, OctoTree*>& feat_tem_map, int wdsize, PLV(3) & pwld,
                     std::vector<std::vector<SlideWindow*>>& sws)
{
  unordered_map<OctoTree*, vector<int>> map_pvec;
  int plsize = pvec->size();
  for (int i = 0; i < plsize; i++)
  {
    pointVar& pv = (*pvec)[i];
    Eigen::Vector3d& pw = pwld[i];
    float loc[3];
    for (int j = 0; j < 3; j++)
    {
      loc[j] = pw[j] / voxel_size;
      if (loc[j] < 0)
        loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    OctoTree* ot = nullptr;
    if (iter != feat_map.end())
    {
      iter->second->isexist = true;
      if (feat_tem_map.find(position) == feat_map.end())
        feat_tem_map[position] = iter->second;
      ot = iter->second;
    }
    else
    {
      ot = new OctoTree(0, wdsize);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }

    map_pvec[ot].push_back(i);
  }

  vector<pair<OctoTree* const, vector<int>>*> octs;
  octs.reserve(map_pvec.size());
  for (auto iter = map_pvec.begin(); iter != map_pvec.end(); iter++)
    octs.push_back(&(*iter));

  int thd_num = sws.size();
  int g_size = octs.size();
  if (g_size < thd_num)
    return;
  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;

  int swsize = sws[0].size() / thd_num;
  for (int i = 1; i < thd_num; i++)
  {
    sws[i].insert(sws[i].end(), sws[0].end() - swsize, sws[0].end());
    sws[0].erase(sws[0].end() - swsize, sws[0].end());
  }

  for (int i = 1; i < thd_num; i++)
  {
    mthreads[i] = new thread(
        [&](int head, int tail, vector<SlideWindow*>& sw) {
          for (int j = head; j < tail; j++)
          {
            for (int k : octs[j]->second)
              octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sw);
          }
        },
        part * i, part * (i + 1), ref(sws[i]));
  }

  for (int i = 0; i < thd_num; i++)
  {
    if (i == 0)
    {
      for (int j = 0; j < int(part); j++)
        for (int k : octs[j]->second)
          octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sws[0]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }
}

void cut_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVec& pvec, int wdsize, double jour)
{
  for (pointVar& pv : pvec)
  {
    float loc[3];
    for (int j = 0; j < 3; j++)
    {
      loc[j] = pv.pnt[j] / voxel_size;
      if (loc[j] < 0)
        loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end())
    {
      iter->second->allocate_fix(pv);
    }
    else
    {
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->push_fix_novar(pv);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->jour = jour;
      feat_map[position] = ot;
    }
  }
}

void generate_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, PVec& pvec, double voxel_size)
{
  for (pointVar& pv : pvec)
  {
    float loc[3];
    for (int j = 0; j < 3; j++)
    {
      loc[j] = pv.pnt[j] / voxel_size;
      if (loc[j] < 0)
        loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end())
    {
      iter->second->allocate_fix(pv);
    }
    else
    {
      OctoTree* ot = new OctoTree(0, 1);
      ot->push_fix_novar(pv);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->jour = 0;
      ot->isexist = true;
      feat_map[position] = ot;
    }
  }
}

void generate_voxel(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map, IMUST& x_curr, PVec& pvec, double voxel_size)
{
  for (pointVar& pv : pvec)
  {
    pv.pnt = x_curr.R * pv.pnt + x_curr.p;
  }

  for (pointVar& pv : pvec)
  {
    float loc[3];
    for (int j = 0; j < 3; j++)
    {
      loc[j] = pv.pnt[j] / voxel_size;
      if (loc[j] < 0)
        loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end())
    {
      iter->second->allocate_fix(pv);
    }
    else
    {
      OctoTree* ot = new OctoTree(0, 1);
      ot->push_fix_novar(pv);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->jour = 0;
      ot->isexist = true;
      feat_map[position] = ot;
    }
  }
}

// Match the point with the plane in the voxel map
int match(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, Eigen::Vector3d& wld, Plane*& pla, Eigen::Matrix3d& var_wld,
          double& sigma_d, OctoTree*& oc)
{
  int flag = 0;

  float loc[3];
  for (int j = 0; j < 3; j++)
  {
    loc[j] = wld[j] / voxel_size;
    if (loc[j] < 0)
      loc[j] -= 1;
  }
  VOXEL_LOC position(loc[0], loc[1], loc[2]);
  auto iter = feat_map.find(position);
  if (iter != feat_map.end())
  {
    double max_prob = 0;
    flag = iter->second->match(wld, pla, max_prob, var_wld, sigma_d, oc);
    if (flag && pla == nullptr)
    {
      printf("pla null max_prob: %lf %ld %ld %ld\n", max_prob, iter->first.x, iter->first.y, iter->first.z);
    }
  }

  return flag;
}

int matchVoxelMap(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, Eigen::Vector3d& wld, Plane*& pla,
                  Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc)
{
  double loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = wld[j] / voxel_size;
    if (loc_xyz[j] < 0)
      loc_xyz[j] -= 1.0;
  }

  VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

  double max_prob = 0;
  for (int ix = -1; ix <= 1; ix++)
  {
    for (int iy = -1; iy <= 1; iy++)
    {
      for (int iz = -1; iz <= 1; iz++)
      {
        VOXEL_LOC loc(position.x + ix, position.y + iy, position.z + iz);
        auto iter = feat_map.find(loc);
        if (iter != feat_map.end())
        {
          OctoTree* octo = iter->second;
          Plane* plane_temp = nullptr;
          double prob_temp = 0;
          double sigma_temp = 0;
          OctoTree* oc_temp = nullptr;

          if (octo->match(wld, plane_temp, prob_temp, var_wld, sigma_temp, oc_temp) > 0)
          {
            if (prob_temp > max_prob)
            {
              max_prob = prob_temp;
              pla = plane_temp;
              sigma_d = sigma_temp;
              oc = oc_temp;
            }
          }
        }
      }
    }
  }
  return (max_prob > 0) ? 1 : 0;
}

void down_sampling_pvec(PVec& pvec, double voxel_size, pcl::PointCloud<PointType>& pl_keep)
{
  unordered_map<VOXEL_LOC, pair<pointVar, int>> feat_map;
  float loc_xyz[3];

  for (pointVar& pv : pvec)
  {
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.pnt[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end())
    {
      feat_map[position] = make_pair(pv, 1);
    }
    else
    {
      pair<pointVar, int>& pp = iter->second;
      pp.first.pnt = (pp.first.pnt * pp.second + pv.pnt) / (pp.second + 1);
      pp.first.var = (pp.first.var * pp.second + pv.var) / (pp.second + 1);
      pp.second += 1;
    }
  }

  pcl::PointCloud<PointType>().swap(pl_keep);
  pl_keep.reserve(feat_map.size());
  PointType ap;

  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
  {
    pointVar& pv = iter->second.first;
    ap.x = pv.pnt[0];
    ap.y = pv.pnt[1];
    ap.z = pv.pnt[2];
    ap.normal_x = pv.var(0, 0);
    ap.normal_y = pv.var(1, 1);
    ap.normal_z = pv.var(2, 2);
    pl_keep.push_back(ap);
  }
}
