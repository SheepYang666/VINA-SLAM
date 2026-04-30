#pragma once

#include "vina_slam/core/types.hpp"
#include <vector>

// The sliding window in each voxel node
class SlideWindow
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  std::vector<PVec> points;  /// The set of points for each frame in the sliding window, containing covariance
                             /// information (PVec = vector<pointVar>)
  std::vector<PointCluster> pcrs_local;  /// Statistics of point clouds for each frame in the sliding window (center of
                                         /// mass, covariance, etc.)

  SlideWindow(int wdsize);
  void resize(int wdsize);

  void clear();
};
