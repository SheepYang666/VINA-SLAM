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

  double part = 1.0 * g_size / tthd_num;

  // Launch worker threads (thread 0 runs on main thread)
  std::vector<std::thread> threads;
  threads.reserve(tthd_num - 1);
  for (int i = 1; i < tthd_num; i++) {
    threads.emplace_back(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
                         part * i, part * (i + 1), std::ref(hessians[i]),
                         std::ref(jacobins[i]), std::ref(resis[i]));
  }

  // Main thread computes partition 0
  voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);

  // Join worker threads and aggregate
  for (auto& t : threads) t.join();

  for (int i = 0; i < tthd_num; i++) {
    Hess += hessians[i];
    JacT += jacobins[i];
    residual += resis[i];
  }

  return residual;
}

double LidarBAOptimizer::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess) {
  double residual1 = 0;

  int local_thd_num = thd_num;
  std::vector<double> residuals(local_thd_num, 0);
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < local_thd_num)
    local_thd_num = 1;

  if (g_size == 0)
    return 0.0;

  double part = 1.0 * g_size / local_thd_num;

  std::vector<std::thread> threads;
  threads.reserve(local_thd_num - 1);
  for (int i = 1; i < local_thd_num; i++)
    threads.emplace_back(&LidarFactor::evaluateOnlyResidual, &voxhess, std::ref(x_stats),
                         part * i, part * (i + 1), std::ref(residuals[i]));

  voxhess.evaluateOnlyResidual(x_stats, 0, part, residuals[0]);

  for (auto& t : threads) t.join();

  for (int i = 0; i < local_thd_num; i++)
    residual1 += residuals[i];

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

  std::vector<std::thread> threads;
  threads.reserve(tthd_num - 1);
  for (int i = 1; i < tthd_num; i++) {
    threads.emplace_back(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
                         part * i, part * (i + 1), std::ref(hessians[i]),
                         std::ref(jacobins[i]), std::ref(resis[i]));
  }

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  // IMU factors (serial)
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

  // Main thread computes LiDAR partition 0
  voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);

  // Join worker threads
  for (auto& t : threads) t.join();

  // Aggregate LiDAR results
  for (int i = 0; i < tthd_num; i++) {
    hessPlus(Hess, JacT, hessians[i], jacobins[i]);
    residual += resis[i];
  }

  return residual;
}

double LIBAOptimizer::divideThread(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                                   NormalFactor& normal,
                                   std::deque<estimation::ImuPreintegration*>& imus_factor,
                                   Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
  int thd_num = 5;
  double residual = 0.0;
  Hess.setZero();
  JacT.setZero();

  PLM(-1) hessians_l(thd_num), hessians_n(thd_num);
  PLV(-1) jacobins_l(thd_num), jacobins_n(thd_num);
  std::vector<double> resis_l(thd_num, 0.0), resis_n(thd_num, 0.0);

  for (int i = 0; i < thd_num; ++i) {
    hessians_l[i].resize(jac_leng, jac_leng);
    jacobins_l[i].resize(jac_leng);
    hessians_n[i].resize(jac_leng, jac_leng);
    jacobins_n[i].resize(jac_leng);
  }

  // LidarFactor parallel
  int tthd_num_l = thd_num;
  int gL = lidar.plvec_voxels.size();
  if (gL < tthd_num_l) tthd_num_l = 1;
  double partL = 1.0 * gL / tthd_num_l;

  std::vector<std::thread> threads_l;
  threads_l.reserve(tthd_num_l - 1);
  for (int i = 1; i < tthd_num_l; ++i) {
    threads_l.emplace_back(&LidarFactor::accEvaluate2, &lidar, std::ref(x_stats),
                           partL * i, partL * (i + 1),
                           std::ref(hessians_l[i]), std::ref(jacobins_l[i]), std::ref(resis_l[i]));
  }

  // IMU serial
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; ++i) {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj;
    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg;
  }
  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  // Main thread: LiDAR partition 0
  lidar.accEvaluate2(x_stats, 0, partL, hessians_l[0], jacobins_l[0], resis_l[0]);
  for (auto& t : threads_l) t.join();

  for (int i = 0; i < tthd_num_l; ++i) {
    hessPlus(Hess, JacT, hessians_l[i], jacobins_l[i]);
    residual += resis_l[i];
  }

  // NormalFactor parallel
  int tthd_num_n = thd_num;
  int gN = normal.plvec_voxels.size();
  if (gN < tthd_num_n) tthd_num_n = 1;
  double partN = 1.0 * gN / tthd_num_n;

  std::vector<std::thread> threads_n;
  threads_n.reserve(tthd_num_n - 1);
  for (int i = 1; i < tthd_num_n; ++i) {
    threads_n.emplace_back(&NormalFactor::accEvaluate2, &normal, std::ref(x_stats),
                           partN * i, partN * (i + 1),
                           std::ref(hessians_n[i]), std::ref(jacobins_n[i]), std::ref(resis_n[i]));
  }

  normal.accEvaluate2(x_stats, 0, partN, hessians_n[0], jacobins_n[0], resis_n[0]);
  for (auto& t : threads_n) t.join();

  for (int i = 0; i < tthd_num_n; ++i) {
    hessPlus(Hess, JacT, hessians_n[i], jacobins_n[i]);
    residual += resis_n[i];
  }

  return residual;
}

double LIBAOptimizer::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                   std::deque<estimation::ImuPreintegration*>& imus_factor) {
  double residual = 0;

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; i++)
    residual += imus_factor[i]->evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual *= (imu_coef * 0.5);

  double lidar_res = 0;
  voxhess.evaluateOnlyResidual(x_stats, 0, voxhess.plvec_voxels.size(), lidar_res);
  residual += lidar_res;

  return residual;
}

double LIBAOptimizer::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                                   NormalFactor& normal,
                                   std::deque<estimation::ImuPreintegration*>& imus_factor) {
  double residual = 0;

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; ++i)
    residual += imus_factor[i]->evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual *= (imu_coef * 0.5);

  double lidar_res = 0, normal_res = 0;
  lidar.evaluateOnlyResidual(x_stats, 0, lidar.plvec_voxels.size(), lidar_res);
  normal.evaluateOnlyResidual(x_stats, 0, normal.plvec_voxels.size(), normal_res);

  return residual + lidar_res + normal_res;
}

void LIBAOptimizer::dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                std::deque<estimation::ImuPreintegration*>& imus_factor,
                                Eigen::MatrixXd* hess) {
  win_size = voxhess.win_size;
  jac_leng = win_size * 6;
  imu_leng = win_size * DIM;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
  Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);
  hess->resize(imu_leng, imu_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;

  int max_iter = 10;
  for (int i = 0; i < max_iter; i++) {
    if (is_calc_hess) {
      residual1 = divideThread(x_stats, voxhess, imus_factor, Hess, JacT);
      *hess = Hess;
    }

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
    }

    for (int j = 0; j < win_size - 1; j++)
      imus_factor[j]->updateState(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = onlyResidual(x_stats_temp, voxhess, imus_factor);

    q = (residual1 - residual2);

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

      for (int j = 0; j < win_size - 1; j++) {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
}

void LIBAOptimizer::dampingIter(std::vector<core::IMUST>& x_stats, LidarFactor& lidar,
                                NormalFactor& normal,
                                std::deque<estimation::ImuPreintegration*>& imus_factor,
                                Eigen::MatrixXd* hess) {
  win_size = lidar.win_size;
  jac_leng = win_size * 6;
  imu_leng = win_size * DIM;

  double u = 0.01, v = 2.0;
  Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
  Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);
  hess->resize(imu_leng, imu_leng);

  D.setIdentity();
  double residual1 = 0.0, residual2 = 0.0, q = 0.0;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;

  for (int iter = 0; iter < 3; ++iter) {
    if (is_calc_hess) {
      residual1 = divideThread(x_stats, lidar, normal, imus_factor, Hess, JacT);
      *hess = Hess;
    }

    Hess.topRows(DIM).setZero();
    Hess.leftCols(DIM).setZero();
    Hess.block<DIM, DIM>(0, 0).setIdentity();
    JacT.head(DIM).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    for (int j = 0; j < win_size; ++j) {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
    }
    for (int j = 0; j < win_size - 1; ++j)
      imus_factor[j]->updateState(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = onlyResidual(x_stats_temp, lidar, normal, imus_factor);
    q = (residual1 - residual2);

    if (q > 0) {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3.0;
      q = q / q1;
      v = 2.0;
      q = 1.0 - std::pow(2.0 * q - 1.0, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    } else {
      u *= v;
      v *= 2.0;
      is_calc_hess = false;

      for (int j = 0; j < win_size - 1; ++j) {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (std::fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
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

  for (size_t i = 0; i < imus.size(); i++)
    imu_res += imus[i]->evaluate(xs[i], xs[i + 1], jtj, gg, false);
  imu_res *= (imu_coef * 0.5);

  lidar.evaluateOnlyResidual(xs, 0, lidar.plvec_voxels.size(), lidar_res);
  normal.evaluateOnlyResidual(xs, 0, normal.plvec_voxels.size(), normal_res);

  total_res = imu_res + lidar_res + normal_res;

  printf("[BA][%s]  E_imu=%.6f   E_lidar=%.6f   E_normal=%.6f   |  Total=%.6f\n",
         tag, imu_res, lidar_res, normal_res, total_res);
}

// ============================================================================
// LIBAOptimizerGravity implementation
// ============================================================================

void LIBAOptimizerGravity::hessPlus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                                    Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
  for (int i = 0; i < win_size; i++) {
    JacT.block<DVEL, 1>(i * DIM, 0) += js.block<DVEL, 1>(i * DVEL, 0);
    for (int j = 0; j < win_size; j++)
      Hess.block<DVEL, DVEL>(i * DIM, j * DIM) += hs.block<DVEL, DVEL>(i * DVEL, j * DVEL);
  }
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

  std::vector<std::thread> threads;
  threads.reserve(tthd_num - 1);
  for (int i = 1; i < tthd_num; i++) {
    threads.emplace_back(&LidarFactor::accEvaluate2, &voxhess, std::ref(x_stats),
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
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj.block<2 * DIM, 2 * DIM>(0, 0);
    Hess.block(i * DIM, imu_leng - 3, DIM * 2, 3) += jtj.block<2 * DIM, 3>(0, 2 * DIM);
    Hess.block(imu_leng - 3, i * DIM, 3, DIM * 2) += jtj.block<3, 2 * DIM>(2 * DIM, 0);
    Hess.block<3, 3>(imu_leng - 3, imu_leng - 3) += jtj.block<3, 3>(2 * DIM, 2 * DIM);
    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg.head(2 * DIM);
    JacT.tail(3) += gg.tail(3);
  }

  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  // Main thread: LiDAR partition 0
  voxhess.accEvaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
  for (auto& t : threads) t.join();

  for (int i = 0; i < tthd_num; i++) {
    hessPlus(Hess, JacT, hessians[i], jacobins[i]);
    residual += resis[i];
  }

  return residual;
}

double LIBAOptimizerGravity::onlyResidual(std::vector<core::IMUST>& x_stats, LidarFactor& voxhess,
                                          std::deque<estimation::ImuPreintegration*>& imus_factor) {
  double residual = 0;
  Eigen::MatrixXd jtj(2 * DIM + 3, 2 * DIM + 3);
  Eigen::VectorXd gg(2 * DIM + 3);

  for (int i = 0; i < win_size - 1; i++)
    residual += imus_factor[i]->evaluateWithGravity(x_stats[i], x_stats[i + 1], jtj, gg, false);
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
  jac_leng = win_size * 6;
  imu_leng = win_size * DIM + 3;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
  Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  std::vector<core::IMUST> x_stats_temp = x_stats;

  for (int i = 0; i < max_iter; i++) {
    if (is_calc_hess) {
      residual1 = divideThread(x_stats, voxhess, imus_factor, Hess, JacT);
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

    x_stats_temp[0].g += dxi.tail(3);
    for (int j = 0; j < win_size; j++) {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
      x_stats_temp[j].g = x_stats_temp[0].g;
    }

    for (int j = 0; j < win_size - 1; j++)
      imus_factor[j]->updateState(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = onlyResidual(x_stats_temp, voxhess, imus_factor);
    q = (residual1 - residual2);

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

      for (int j = 0; j < win_size - 1; j++) {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
  resis.push_back(residual2);
}

} // namespace mapping
} // namespace vina_slam
