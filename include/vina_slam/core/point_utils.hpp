#pragma once

#include "vina_slam/core/math.hpp"
#include "vina_slam/core/types.hpp"

// Point cloud downsampling: keep mean position per voxel
inline void down_sampling_voxel(pcl::PointCloud<PointType>& pl_feat, double voxel_size)
{
  if (voxel_size < 0.001)
    return;

  unordered_map<VOXEL_LOC, PointType> feat_map;
  float loc_xyz[3];
  for (PointType& p_c : pl_feat.points)
  {
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter == feat_map.end())
    {
      PointType pp = p_c;
      pp.curvature = 1;
      feat_map[position] = pp;
    }
    else
    {
      PointType& pp = iter->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;
    }
  }

  pl_feat.clear();
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);
}

// Keep one real point closest to the voxel mean
inline void down_sampling_close(pcl::PointCloud<PointType>& pl_feat, double voxel_size)
{
  if (voxel_size < 0.001)
    return;

  unordered_map<VOXEL_LOC, pcl::PointCloud<PointType>::Ptr> feat_map;
  float loc_xyz[3];
  for (PointType& p_c : pl_feat.points)
  {
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter == feat_map.end())
    {
      pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>);
      pl_ptr->push_back(p_c);
      feat_map[position] = pl_ptr;
    }
    else
    {
      iter->second->push_back(p_c);
    }
  }

  pl_feat.clear();
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
  {
    pcl::PointCloud<PointType>::Ptr pl_ptr = iter->second;

    PointType pb = pl_ptr->points[0];
    int plsize = pl_ptr->size();
    for (int i = 1; i < plsize; i++)
    {
      PointType& pp = pl_ptr->points[i];
      pb.x += pp.x;
      pb.y += pp.y;
      pb.z += pp.z;
    }
    pb.x /= plsize;
    pb.y /= plsize;
    pb.z /= plsize;

    double ndis = 100;
    int mnum = 0;
    for (int i = 0; i < plsize; i++)
    {
      PointType& pp = pl_ptr->points[i];
      double xx = pb.x - pp.x;
      double yy = pb.y - pp.y;
      double zz = pb.z - pp.z;
      double dis = xx * xx + yy * yy + zz * zz;
      if (dis < ndis)
      {
        mnum = i;
        ndis = dis;
      }
    }

    pl_feat.push_back(pl_ptr->points[mnum]);
  }
}

void calcBodyVar(Eigen::Vector3d& pb, const float range_inc, const float degree_inc, Eigen::Matrix3d& var);

void var_init(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr, double dept_err, double beam_err);

void pvec_update(PVecPtr pptr, IMUST& x_curr, PLV(3) & pwld);
