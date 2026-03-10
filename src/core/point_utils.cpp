/**
 * @file point_utils.cpp
 * @brief Implementation of point cloud utility functions
 */

#include "vina_slam/core/point_utils.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace vina_slam {
namespace core {

void calcBodyVar(Eigen::Vector3d& pb, float range_inc, float degree_inc, Eigen::Matrix3d& var) {
  if (pb[2] == 0) {
    pb[2] = 0.0001;
  }

  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;

  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

  Eigen::Vector3d direction(pb);
  direction.normalize();

  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1),
                   direction(2), 0, -direction(0),
                   -direction(1), direction(0), 0;

  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();

  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();

  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0),
       base_vector1(1), base_vector2(1),
       base_vector1(2), base_vector2(2);

  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;

  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void varInit(IMUST& ext, pcl::PointCloud<PointType>& pl_cur, PVecPtr pptr,
             double dept_err, double beam_err) {
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);

  for (int i = 0; i < plsize; i++) {
    PointType& ap = pl_cur[i];
    pointVar& pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
    pv.intensity = pl_cur[i].intensity;
  }
}

void pvecUpdate(PVecPtr pptr, IMUST& x_curr, PLV(3)& pwld) {
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for (pointVar& pv : *pptr) {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

double getMemoryUsage() {
  std::ifstream infile("/proc/self/status");
  double mem = -1;
  std::string lineStr, str;
  while (std::getline(infile, lineStr)) {
    std::stringstream ss(lineStr);
    bool is_find = false;
    while (ss >> str) {
      if (str == "VmRSS:") {
        is_find = true;
        continue;
      }
      if (is_find) mem = std::stod(str);
      break;
    }
    if (is_find) break;
  }
  return mem / (1048576);
}

} // namespace core
} // namespace vina_slam
