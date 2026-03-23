#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/core/math.hpp"
#include <cmath>
#include <cstdio>
#include <rclcpp/clock.hpp>
#include <thread>

double imu_coef = 1e-4;

// The LM optimizer for LiDAR BA

double Lidar_BA_Optimizer::divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd& Hess,
                                         Eigen::VectorXd& JacT)
{
  double residual = 0;

  Hess.setZero();
  JacT.setZero();

  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);

  for (int i = 0; i < thd_num; i++)
  {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  vector<double> resis(tthd_num, 0);

  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;

  vector<thread*> mthreads(tthd_num);
  double part = 1.0 * g_size / tthd_num;

  for (int i = 1; i < tthd_num; i++)
  {
    mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1), ref(hessians[i]),
                             ref(jacobins[i]), ref(resis[i]));
  }

  for (int i = 0; i < tthd_num; i++)
  {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
    Hess += hessians[i];
    JacT += jacobins[i];
    residual += resis[i];

    delete mthreads[i];
  }

  return residual;
}

double Lidar_BA_Optimizer::only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess)
{
  double residual1 = 0;

  vector<double> residuals(thd_num, 0);
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < thd_num)
  {
    printf("Too Less Voxel");
    exit(0);
  }
  vector<thread*> mthreads(thd_num, nullptr);
  double part = 1.0 * g_size / thd_num;
  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part * i, part * (i + 1),
                             ref(residuals[i]));

  for (int i = 0; i < thd_num; i++)
  {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1), residuals[i]);
    residual1 += residuals[i];
    delete mthreads[i];
  }

  return residual1;
}

bool Lidar_BA_Optimizer::damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd* hess,
                                      vector<double>& resis, int max_iter, bool is_display)
{
  win_size = voxhess.win_size;
  jac_leng = win_size * 6;

  double u = 0.01, v = 2;
  Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
  Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);
  hess->resize(jac_leng, jac_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  vector<IMUST> x_stats_temp = x_stats;

  bool is_converge = true;

  for (int i = 0; i < max_iter; i++)
  {
    if (is_calc_hess)
    {
      residual1 = divide_thread(x_stats, voxhess, Hess, JacT);
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

    for (int j = 0; j < win_size; j++)
    {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(6 * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(6 * j + 3, 0);
    }
    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

    residual2 = only_residual(x_stats_temp, voxhess);

    q = (residual1 - residual2);
    if (is_display)
    {
      printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q / q1, q1, q);
    }

    if (q > 0)
    {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3;

      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    }
    else
    {
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

// The LiDAR-Inertial BA optimizer

void LI_BA_Optimizer::hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs, Eigen::VectorXd& js)
{
  for (int i = 0; i < win_size; i++)
  {
    JacT.block<DVEL, 1>(i * DIM, 0) += js.block<DVEL, 1>(i * DVEL, 0);
    for (int j = 0; j < win_size; j++)
      Hess.block<DVEL, DVEL>(i * DIM, j * DIM) += hs.block<DVEL, DVEL>(i * DVEL, j * DVEL);
  }
}

double LI_BA_Optimizer::divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                                      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT)
{
  int thd_num = 5;
  double residual = 0;
  Hess.setZero();
  JacT.setZero();
  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);
  vector<double> resis(thd_num, 0);

  for (int i = 0; i < thd_num; i++)
  {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;
  double part = 1.0 * g_size / tthd_num;

  vector<thread*> mthreads(tthd_num);
  for (int i = 1; i < tthd_num; i++)
  {
    mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1), ref(hessians[i]),
                             ref(jacobins[i]), ref(resis[i]));
  }

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  // imu
  for (int i = 0; i < win_size - 1; i++)
  {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj;
    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg;
  }

  Eigen::Matrix<double, DIM, DIM> joc;
  Eigen::Matrix<double, DIM, 1> rr;
  joc.setIdentity();
  rr.setZero();

  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  for (int i = 0; i < tthd_num; i++)
  {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
    hess_plus(Hess, JacT, hessians[i], jacobins[i]);
    residual += resis[i];
    delete mthreads[i];
  }

  return residual;
}

double LI_BA_Optimizer::divide_thread(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                                      std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT)
{
  int thd_num = 5;
  double residual = 0.0;
  Hess.setZero();
  JacT.setZero();

  PLM(-1) hessians_l(thd_num), hessians_n(thd_num);
  PLV(-1) jacobins_l(thd_num), jacobins_n(thd_num);
  std::vector<double> resis_l(thd_num, 0.0), resis_n(thd_num, 0.0);

  for (int i = 0; i < thd_num; ++i)
  {
    hessians_l[i].resize(jac_leng, jac_leng);
    jacobins_l[i].resize(jac_leng);
    hessians_n[i].resize(jac_leng, jac_leng);
    jacobins_n[i].resize(jac_leng);
  }

  int tthd_num_l = thd_num;
  int gL = lidar.plvec_voxels.size();
  if (gL < tthd_num_l)
    tthd_num_l = 1;
  double partL = 1.0 * gL / tthd_num_l;

  std::vector<std::thread*> tl(tthd_num_l, nullptr);
  for (int i = 1; i < tthd_num_l; ++i)
  {
    tl[i] = new std::thread(&LidarFactor::acc_evaluate2, &lidar, x_stats, partL * i, partL * (i + 1),
                            std::ref(hessians_l[i]), std::ref(jacobins_l[i]), std::ref(resis_l[i]));
  }

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; ++i)
  {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj;
    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg;
  }
  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  for (int i = 0; i < tthd_num_l; ++i)
  {
    if (i != 0)
      tl[i]->join();
    else
      lidar.acc_evaluate2(x_stats, 0, partL, hessians_l[0], jacobins_l[0], resis_l[0]);
    hess_plus(Hess, JacT, hessians_l[i], jacobins_l[i]);
    residual += resis_l[i];
    if (tl[i])
    {
      delete tl[i];
      tl[i] = nullptr;
    }
  }

  int tthd_num_n = thd_num;
  int gN = normal.plvec_voxels.size();
  if (gN < tthd_num_n)
    tthd_num_n = 1;
  double partN = 1.0 * gN / tthd_num_n;

  std::vector<std::thread*> tn(tthd_num_n, nullptr);
  for (int i = 1; i < tthd_num_n; ++i)
  {
    tn[i] = new std::thread(&NormalFactor::acc_evaluate2, &normal, x_stats, partN * i, partN * (i + 1),
                            std::ref(hessians_n[i]), std::ref(jacobins_n[i]), std::ref(resis_n[i]));
  }

  for (int i = 0; i < tthd_num_n; ++i)
  {
    if (i != 0)
      tn[i]->join();
    else
      normal.acc_evaluate2(x_stats, 0, partN, hessians_n[0], jacobins_n[0], resis_n[0]);
    hess_plus(Hess, JacT, hessians_n[i], jacobins_n[i]);
    residual += resis_n[i];
    if (tn[i])
    {
      delete tn[i];
      tn[i] = nullptr;
    }
  }

  return residual;
}

double LI_BA_Optimizer::only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor)
{
  double residual1 = 0, residual2 = 0;
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  int thd_num = 5;
  vector<double> residuals(thd_num, 0);
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < thd_num)
  {
    thd_num = 1;
  }
  vector<thread*> mthreads(thd_num, nullptr);
  double part = 1.0 * g_size / thd_num;
  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part * i, part * (i + 1),
                             ref(residuals[i]));

  for (int i = 0; i < win_size - 1; i++)
    residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual1 *= (imu_coef * 0.5);

  for (int i = 0; i < thd_num; i++)
  {
    if (i != 0)
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
    else
      voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1), residuals[i]);
    residual2 += residuals[i];
  }

  return (residual1 + residual2);
}

double LI_BA_Optimizer::only_residual(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                                      std::deque<IMU_PRE*>& imus_factor)
{
  double residual1 = 0.0, residual2 = 0.0, residual3 = 0.0;

  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; ++i)
    residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual1 *= (imu_coef * 0.5);

  int thd_num = 5;
  int gL = lidar.plvec_voxels.size();
  if (gL < thd_num)
    thd_num = 1;
  std::vector<std::thread*> tl(thd_num, nullptr);
  std::vector<double> resL(thd_num, 0.0);
  double partL = 1.0 * gL / thd_num;
  for (int i = 1; i < thd_num; ++i)
    tl[i] = new std::thread(&LidarFactor::evaluate_only_residual, &lidar, x_stats, partL * i, partL * (i + 1),
                            std::ref(resL[i]));
  lidar.evaluate_only_residual(x_stats, 0, partL, resL[0]);
  for (int i = 1; i < thd_num; ++i)
  {
    tl[i]->join();
    delete tl[i];
  }
  for (double v : resL)
    residual2 += v;

  thd_num = 5;
  int gN = normal.plvec_voxels.size();
  if (gN < thd_num)
    thd_num = 1;
  std::vector<std::thread*> tn(thd_num, nullptr);
  std::vector<double> resN(thd_num, 0.0);
  double partN = 1.0 * gN / thd_num;
  for (int i = 1; i < thd_num; ++i)
    tn[i] = new std::thread(&NormalFactor::evaluate_only_residual, &normal, x_stats, partN * i, partN * (i + 1),
                            std::ref(resN[i]));
  normal.evaluate_only_residual(x_stats, 0, partN, resN[0]);
  for (int i = 1; i < thd_num; ++i)
  {
    tn[i]->join();
    delete tn[i];
  }
  for (double v : resN)
    residual3 += v;

  return residual1 + residual2 + residual3;
}

void LI_BA_Optimizer::damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                                   Eigen::MatrixXd* hess)
{
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
  vector<IMUST> x_stats_temp = x_stats;

  double hesstime = 0;
  double resitime = 0;

  int max_iter = 10;
  for (int i = 0; i < max_iter; i++)
  {
    if (is_calc_hess)
    {
      double tm = rclcpp::Clock().now().seconds();
      residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
      hesstime += rclcpp::Clock().now().seconds() - tm;
      *hess = Hess;
    }

    Hess.topRows(DIM).setZero();
    Hess.leftCols(DIM).setZero();
    Hess.block<DIM, DIM>(0, 0).setIdentity();
    JacT.head(DIM).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    for (int j = 0; j < win_size; j++)
    {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
    }

    for (int j = 0; j < win_size - 1; j++)
      imus_factor[j]->update_state(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

    double tl1 = rclcpp::Clock().now().seconds();
    residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
    double tl2 = rclcpp::Clock().now().seconds();

    resitime += tl2 - tl1;

    q = (residual1 - residual2);

    if (q > 0)
    {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3;

      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    }
    else
    {
      u = u * v;
      v = 2 * v;
      is_calc_hess = false;

      for (int j = 0; j < win_size - 1; j++)
      {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
}

void LI_BA_Optimizer::damping_iter(std::vector<IMUST>& x_stats, LidarFactor& lidar, NormalFactor& normal,
                                   std::deque<IMU_PRE*>& imus_factor, Eigen::MatrixXd* hess)
{
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
  std::vector<IMUST> x_stats_temp = x_stats;

  for (int iter = 0; iter < 3; ++iter)
  {
    if (is_calc_hess)
    {
      residual1 = divide_thread(x_stats, lidar, normal, imus_factor, Hess, JacT);
      *hess = Hess;
    }

    Hess.topRows(DIM).setZero();
    Hess.leftCols(DIM).setZero();
    Hess.block<DIM, DIM>(0, 0).setIdentity();
    JacT.head(DIM).setZero();

    D.diagonal() = Hess.diagonal();
    dxi = (Hess + u * D).ldlt().solve(-JacT);

    for (int j = 0; j < win_size; ++j)
    {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
    }
    for (int j = 0; j < win_size - 1; ++j)
      imus_factor[j]->update_state(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

    residual2 = only_residual(x_stats_temp, lidar, normal, imus_factor);
    q = (residual1 - residual2);

    if (q > 0)
    {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3.0;
      q = q / q1;
      v = 2.0;
      q = 1.0 - std::pow(2.0 * q - 1.0, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    }
    else
    {
      u *= v;
      v *= 2.0;
      is_calc_hess = false;

      for (int j = 0; j < win_size - 1; ++j)
      {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (std::fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
}

void LI_BA_Optimizer::print_breakdown(const char* tag, std::vector<IMUST>& xs, LidarFactor& lidar, NormalFactor& normal,
                                      std::deque<IMU_PRE*>& imus, double& imu_res, double& lidar_res,
                                      double& normal_res, double& total_res) const
{
  double e_imu = 0.0;
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < (int)xs.size() - 1; ++i)
  {
    jtj.setZero();
    gg.setZero();
    e_imu += imus[i]->give_evaluate(xs[i], xs[i + 1], jtj, gg, false);
  }
  e_imu *= (imu_coef * 0.5);

  double e_lidar = 0.0;
  lidar.evaluate_only_residual(xs, 0, (int)lidar.plvec_voxels.size(), e_lidar);

  double e_normal = 0.0;
  normal.evaluate_only_residual(xs, 0, (int)normal.plvec_voxels.size(), e_normal);

  const double e_total = e_imu + e_lidar + e_normal;
  std::printf("[BA][%s]  E_imu=%.6f   E_lidar=%.6f   E_normal=%.6f   |  Total=%.6f\n", tag, e_imu, e_lidar, e_normal,
              e_total);

  imu_res = e_imu;
  lidar_res = e_lidar;
  normal_res = e_normal;
  total_res = e_total;
}

// The LiDAR-Inertial BA optimizer with gravity optimization

void LI_BA_OptimizerGravity::hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, Eigen::MatrixXd& hs,
                                       Eigen::VectorXd& js)
{
  for (int i = 0; i < win_size; i++)
  {
    JacT.block<DVEL, 1>(i * DIM, 0) += js.block<DVEL, 1>(i * DVEL, 0);
    for (int j = 0; j < win_size; j++)
      Hess.block<DVEL, DVEL>(i * DIM, j * DIM) += hs.block<DVEL, DVEL>(i * DVEL, j * DVEL);
  }
}

double LI_BA_OptimizerGravity::divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                                             Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT)
{
  int thd_num = 5;
  double residual = 0;
  Hess.setZero();
  JacT.setZero();
  PLM(-1) hessians(thd_num);
  PLV(-1) jacobins(thd_num);
  vector<double> resis(thd_num, 0);

  for (int i = 0; i < thd_num; i++)
  {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;
  double part = 1.0 * g_size / tthd_num;

  vector<thread*> mthreads(tthd_num);
  for (int i = 1; i < tthd_num; i++)
    mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1), ref(hessians[i]),
                             ref(jacobins[i]), ref(resis[i]));

  Eigen::MatrixXd jtj(2 * DIM + 3, 2 * DIM + 3);
  Eigen::VectorXd gg(2 * DIM + 3);

  for (int i = 0; i < win_size - 1; i++)
  {
    jtj.setZero();
    gg.setZero();
    residual += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i + 1], jtj, gg, true);
    Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj.block<2 * DIM, 2 * DIM>(0, 0);
    Hess.block<DIM * 2, 3>(i * DIM, imu_leng - 3) += jtj.block<2 * DIM, 3>(0, 2 * DIM);
    Hess.block<3, DIM * 2>(imu_leng - 3, i * DIM) += jtj.block<3, 2 * DIM>(2 * DIM, 0);
    Hess.block<3, 3>(imu_leng - 3, imu_leng - 3) += jtj.block<3, 3>(2 * DIM, 2 * DIM);

    JacT.block<DIM * 2, 1>(i * DIM, 0) += gg.head(2 * DIM);
    JacT.tail(3) += gg.tail(3);
  }

  Eigen::Matrix<double, DIM, DIM> joc;
  Eigen::Matrix<double, DIM, 1> rr;
  joc.setIdentity();
  rr.setZero();

  Hess *= imu_coef;
  JacT *= imu_coef;
  residual *= (imu_coef * 0.5);

  for (int i = 0; i < tthd_num; i++)
  {
    if (i != 0)
      mthreads[i]->join();
    else
      voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
    hess_plus(Hess, JacT, hessians[i], jacobins[i]);
    residual += resis[i];
    delete mthreads[i];
  }

  return residual;
}

double LI_BA_OptimizerGravity::only_residual(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor)
{
  double residual1 = 0, residual2 = 0;
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);

  int thd_num = 5;
  vector<double> residuals(thd_num, 0);
  int g_size = voxhess.plvec_voxels.size();
  if (g_size < thd_num)
  {
    thd_num = 1;
  }
  vector<thread*> mthreads(thd_num, nullptr);
  double part = 1.0 * g_size / thd_num;
  for (int i = 1; i < thd_num; i++)
    mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part * i, part * (i + 1),
                             ref(residuals[i]));

  for (int i = 0; i < win_size - 1; i++)
    residual1 += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual1 *= (imu_coef * 0.5);

  for (int i = 0; i < thd_num; i++)
  {
    if (i != 0)
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
    else
      voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1), residuals[i]);
    residual2 += residuals[i];
  }

  return (residual1 + residual2);
}

void LI_BA_OptimizerGravity::damping_iter(vector<IMUST>& x_stats, LidarFactor& voxhess, deque<IMU_PRE*>& imus_factor,
                                          vector<double>& resis, Eigen::MatrixXd* hess, int max_iter)
{
  win_size = voxhess.win_size;
  jac_leng = win_size * 6;
  imu_leng = win_size * DIM + 3;
  double u = 0.01, v = 2;
  Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
  Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

  D.setIdentity();
  double residual1, residual2, q;
  bool is_calc_hess = true;
  vector<IMUST> x_stats_temp = x_stats;

  for (int i = 0; i < max_iter; i++)
  {
    if (is_calc_hess)
    {
      residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
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
    for (int j = 0; j < win_size; j++)
    {
      x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
      x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
      x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
      x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
      x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
      x_stats_temp[j].g = x_stats_temp[0].g;
    }

    for (int j = 0; j < win_size - 1; j++)
      imus_factor[j]->update_state(dxi.block<DIM, 1>(DIM * j, 0));

    double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
    residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
    q = (residual1 - residual2);

    if (q > 0)
    {
      x_stats = x_stats_temp;
      double one_three = 1.0 / 3;

      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      is_calc_hess = true;
    }
    else
    {
      u = u * v;
      v = 2 * v;
      is_calc_hess = false;

      for (int j = 0; j < win_size - 1; j++)
      {
        imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
        imus_factor[j]->dba = imus_factor[j]->dba_buf;
      }
    }

    if (fabs((residual1 - residual2) / residual1) < 1e-6)
      break;
  }
  resis.push_back(residual2);
}
