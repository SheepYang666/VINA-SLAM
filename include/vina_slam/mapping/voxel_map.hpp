#pragma once

#include "vina_slam/mapping/octree.hpp"
#include <unordered_map>

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

int matchVoxelMap(unordered_map<VOXEL_LOC, OctoTree*>& feat_map, Eigen::Vector3d& wld, Plane*& pla,
                  Eigen::Matrix3d& var_wld, double& sigma_d, OctoTree*& oc);

void down_sampling_pvec(PVec& pvec, double voxel_size, pcl::PointCloud<PointType>& pl_keep);
