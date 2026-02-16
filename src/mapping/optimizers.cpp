/**
 * @file optimizers.cpp
 * @brief Implementation of bundle adjustment optimizers
 */

#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/core/math.hpp"
#include <thread>
#include <cstdio>

namespace vina_slam {
namespace mapping {

// Global IMU coefficient
double imu_coef = 1e-4;

// ============================================================================
// LidarBAOptimizer implementation
// ============================================================================

double LidarBAOptimizer::divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
  double residual = 0;

  Hess.setZero();
  JacT.setZero();

  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);

  for (int i = 0; i < thd_num; i++) {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  std::vector<double> resis(tthd_num, 0);

  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;

  std::vector<std::thread*> mthreads(tthd_num);
  double part = 1.0 * g_size / tthd_num;

  for (int i = 1; i < tthd_num; i++) {
    mthreads[i] = new std::thread(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
                                  part * i, part * (i + 1), std::ref(hessians[i]),
                                  std::ref(jacobins[i]), std::ref(resis[i]));
  }

  for (int i = 0; i < tthd_num; i++) {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
    Hess += hessians[i];
    JacT += jacobins[i];
    residual += resis[i];

    delete mthreads[i];
  }

  return residual;
}

double LidarBAOptimizer::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess) {
  double residual1 = 0;

  std::vector<double> residuals(thd_num, 0);
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < thd_num) {
    thd_num = 1;
  }

  if (g_size < thd_num) {
    printf("Too Less Voxel");
    exit(0);
  }
  std::vector<std::thread*> mthreads(thd_num, nullptr);
  double part = 1.0 * g_size / thd_num;
  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new std::thread(&LidarFactor::evaluateOnlyResidual, &voxhess, std::ref(x_stats),
                                  part * i, part * (i + 1), std::ref(residuals[i]));

  for (int i = 0; i < thd_num; i++) {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.evaluateOnlyResidual(x_stats, part * i, part * (i + 1), residuals[i]);
    residual1 += residuals[i];
    delete mthreads[i];
  }

  return residual1;
}

bool LidarBAOptimizer::dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                   Eigen::MatrixXd* hess, std::vector<double>& resis,
                                   int max_iter, bool is_display) {
  win_size = voxhess.win_size;
  jac_leng = win_size * 6;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
  Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);
  hess->resize(jac_leng, jac_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;

  bool is_converge = true;

  for (int i = 0; i < max_iter; i++) {
    if (is_calc_hess) {
      residual1 = divideThread(x_stats, voxhess, Hess, JacT);
      *hess = Hess;
    }

    if (i == 0)
      resis.push_back(residual1);

    Hess.topRows(6).setZero();
    Hess.leftCols(6).setZero();
    Hess.block<6, 6>(0, 0).setIdentity();
    JacT.head(6).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    for (int j = 0; j < win_size; j++) {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(6 * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(6 * j + 3, 0);
    }
    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

    residual2 = onlyResidual(x_stats_temp, voxhess);

    q = (residual1 - residual2);
    if (is_display) {
      printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n",
             i, residual1, residual2, u, v, q / q1, q1, q);
    }

    if (q > 0) {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3;

      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    } else {
      u = u * v;
      v = 2 * v;
      is_calc_hess = false;
      is_converge = false;
    }

    if (fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
  resis.push_back(residual2);
  return is_converge;
}

// ============================================================================
// LIBAOptimizer implementation
// ============================================================================

void LIBAOptimizer::hessPlus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                             Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
  for (int i = 0; i < win_size; i++) {
    JacT.block<DVEL, 1>(i * DIM, 0) += js.block<DVEL, 1>(i * DVEL, 0);
    for (int j = 0; j < win_size; j++)
      Hess.block<DVEL, DVEL>(i * DIM, j * DIM) += hs.block<DVEL, DVEL>(i * DVEL, j * DVEL);
  }
}

double LIBAOptimizer::divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                   std::deque<estimation::ImuPreintegration*>& imus_factor,
                                   Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
  int thd_num = 5;
  double residual = 0;
  Hess.setZero();
  JacT.setZero();
  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);
  std::vector<double> resis(thd_num, 0);

  for (int i = 0; i < thd_num; i++) {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;
  double part = 1.0 * g_size / tthd_num;

  std::vector<std::thread*> mthreads(tthd_num);
  for (int i = 1; i < tthd_num; i++) {
    mthreads[i] = new std::thread(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
                                  part * i, part * (i + 1), std::ref(hessians[i]),
                                  std::ref(jacobins[i]), std::ref(resis[i]));
  }

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  // IMU factors
  for (int i = 0; i < win_size - 1; i++) {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj;
    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg;
  }

  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  for (int i = 0; i < tthd_num; i++) {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
    hessPlus(Hess, JacT, hessians[i], jacobins[i]);
    residual += resis[i];
    delete mthreads[i];
  }

  return residual;
}

double LIBAOptimizer::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                   std::deque<estimation::ImuPreintegration*>& imus_factor) {
  double residual = 0;

  // IMU residual
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; i++) {
    residual += imus_factor[i]->evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  }
  residual *= (imu_coef * 0.5);

  // LiDAR residual
  double lidar_res = 0;
  voxhess.evaluateOnlyResidual(x_stats, 0, voxhess.plvec_voxels.size(), lidar_res);
  residual += lidar_res;

  return residual;
}

void LIBAOptimizer::dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                std::deque<estimation::ImuPreintegration*>& imus_factor,
                                Eigen::MatrixXd* hess) {
  win_size = voxhess.win_size;
  jac_leng = win_size * DIM;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
  Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;

  for (int iter = 0; iter < 3; iter++) {
    if (is_calc_hess)
      residual1 = divideThread(x_stats, voxhess, imus_factor, Hess, JacT);

    // Fix first pose
    Hess.topRows(DIM).setZero();
    Hess.leftCols(DIM).setZero();
    Hess.block<DIM, DIM>(0, 0).setIdentity();
    JacT.head(DIM).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    // Apply state update
    for (int j = 0; j < win_size; j++) {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
    }

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = onlyResidual(x_stats_temp, voxhess, imus_factor);

    q = residual1 - residual2;

    if (q > 0) {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3;
      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    } else {
      u = u * v;
      v = 2 * v;
      is_calc_hess = false;
    }
  }

  if (hess)
    *hess = Hess;
}

void LIBAOptimizer::printBreakdown(const char* tag, std::vector<core::IMUST>& xs,
                                   LidarFactor& lidar, NormalFactor& normal,
                                   std::deque<estimation::ImuPreintegration*>& imus,
                                   double& imu_res, double& lidar_res, double& normal_res,
                                   double& total_res) const {
  imu_res = 0;
  lidar_res = 0;
  normal_res = 0;

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  for (size_t i = 0; i < imus.size(); i++) {
    imu_res += imus[i]->evaluate(xs[i], xs[i + 1], jtj, gg, false);
  }

  lidar.evaluateOnlyResidual(xs, 0, lidar.plvec_voxels.size(), lidar_res);
  normal.evaluateOnlyResidual(xs, 0, normal.plvec_voxels.size(), normal_res);

  total_res = imu_coef * imu_res + lidar_res + normal_res;

  printf("[%s] IMU: %.4f (x%.0e), LiDAR: %.4f, Normal: %.4f, Total: %.4f\n",
         tag, imu_res, imu_coef, lidar_res, normal_res, total_res);
}

// ============================================================================
// LIBAOptimizerGravity implementation
// ============================================================================

void LIBAOptimizerGravity::hessPlus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                                    Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
  int offset = win_size * DIM;
  Hess.block(0, 0, offset, offset) += hs.block(0, 0, offset, offset);
  Hess.block(0, offset, offset, 3) += hs.block(0, offset, offset, 3);
  Hess.block(offset, 0, 3, offset) += hs.block(offset, 0, 3, offset);
  Hess.block<3, 3>(offset, offset) += hs.block<3, 3>(offset, offset);
  JacT.head(offset) += js.head(offset);
  JacT.block<3, 1>(offset, 0) += js.block<3, 1>(offset, 0);
}

double LIBAOptimizerGravity::divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                          std::deque<estimation::ImuPreintegration*>& imus_factor,
                                          Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
  int thd_num = 5;
  double residual = 0;

  Hess.setZero();
  JacT.setZero();
  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);
  std::vector<double> resis(thd_num, 0);

  for (int i = 0; i < thd_num; i++) {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;
  double part = 1.0 * g_size / tthd_num;

  std::vector<std::thread*> mthreads(tthd_num);
  for (int i = 1; i < tthd_num; i++) {
    mthreads[i] = new std::thread(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
                                  part * i, part * (i + 1), std::ref(hessians[i]),
                                  std::ref(jacobins[i]), std::ref(resis[i]));
  }

  Eigen::MatrixXd jtj(2 * DIM + 3, 2 * DIM + 3);
  Eigen::VectorXd gg(2 * DIM + 3);

  // IMU factors with gravity
  for (int i = 0; i < win_size - 1; i++) {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->evaluateWithGravity(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block(0, 0, 2 * DIM, 2 * DIM) += jtj.block(0, 0, 2 * DIM, 2 * DIM);
    Hess.block(0, jac_leng, 2 * DIM, 3) += jtj.block(0, 2 * DIM, 2 * DIM, 3);
    Hess.block(jac_leng, 0, 3, 2 * DIM) += jtj.block(2 * DIM, 0, 3, 2 * DIM);
    Hess.block<3, 3>(jac_leng, jac_leng) += jtj.block<3, 3>(2 * DIM, 2 * DIM);
    JacT.head(2 * DIM) += gg.head(2 * DIM);
    JacT.block<3, 1>(jac_leng, 0) += gg.block<3, 1>(2 * DIM, 0);
  }

  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  for (int i = 0; i < tthd_num; i++) {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);

    Hess.block(0, 0, jac_leng, jac_leng) += hessians[i];
    JacT.head(jac_leng) += jacobins[i];
    residual += resis[i];
    delete mthreads[i];
  }

  return residual;
}

double LIBAOptimizerGravity::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                          std::deque<estimation::ImuPreintegration*>& imus_factor) {
  double residual = 0;
  Eigen::MatrixXd jtj(2 * DIM + 3, 2 * DIM + 3);
  Eigen::VectorXd gg(2 * DIM + 3);

  for (int i = 0; i < win_size - 1; i++) {
    residual += imus_factor[i]->evaluateWithGravity(x_stats[i], x_stats[i + 1], jtj, gg, false);
  }
  residual *= (imu_coef * 0.5);

  double lidar_res = 0;
  voxhess.evaluateOnlyResidual(x_stats, 0, voxhess.plvec_voxels.size(), lidar_res);
  residual += lidar_res;

  return residual;
}

void LIBAOptimizerGravity::dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                       std::deque<estimation::ImuPreintegration*>& imus_factor,
                                       std::vector<double>& resis, Eigen::MatrixXd* hess,
                                       int max_iter) {
  win_size = voxhess.win_size;
  jac_leng = win_size * DIM;
  int total_size = jac_leng + 3;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(total_size, total_size), Hess(total_size, total_size);
  Eigen::VectorXd JacT(total_size), dxi(total_size);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;
  Eigen::Vector3d g_temp = Eigen::Vector3d::Zero();  // Initialize to avoid -Wmaybe-uninitialized

  resis.clear();

  for (int iter = 0; iter < max_iter; iter++) {
    if (is_calc_hess)
      residual1 = divideThread(x_stats, voxhess, imus_factor, Hess, JacT);

    if (iter == 0)
      resis.push_back(residual1);

    // Fix first pose
    Hess.topRows(DIM).setZero();
    Hess.leftCols(DIM).setZero();
    Hess.block<DIM, DIM>(0, 0).setIdentity();
    JacT.head(DIM).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    for (int j = 0; j < win_size; j++) {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
      g_temp = x_stats[j].g + dxi.block<3, 1>(jac_leng, 0);
      x_stats_temp[j].g = g_temp;
    }

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = onlyResidual(x_stats_temp, voxhess, imus_factor);

    q = residual1 - residual2;

    if (q > 0) {
      x_stats = x_stats_temp;
      for (int j = 0; j < win_size; j++)
        x_stats[j].g = g_temp;

      double one_three = 1.0 / 3;
      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    } else {
      u = u * v;
      v = 2 * v;
      is_calc_hess = false;
    }
  }

  resis.push_back(residual2);
  if (hess)
    *hess = Hess;
}

} // namespace mapping
} // namespace vina_slam
