#include "vina_slam/voxel_map.hpp"
#include <cstdio>
#include <Eigen/Geometry>

namespace
{
// Generate a stable voxel ID from center coordinates (used for visualization markers)
[[maybe_unused]] static int voxel_id_from_center(const double center[3], int layer)
{
  // Quantize center to millimeters for a stable id across runs.
  const int64_t ix = llround(center[0] * 1000.0);
  const int64_t iy = llround(center[1] * 1000.0);
  const int64_t iz = llround(center[2] * 1000.0);
  int64_t h = ix * 73856093LL ^ iy * 19349663LL ^ iz * 83492791LL ^ (int64_t(layer) * 2654435761LL);
  if (h < 0)
    h = -h;
  return static_cast<int>(h & 0x7fffffff);
}

void map_jet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g, uint8_t& b)
{
  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;

  double dr = 1.0, dg = 1.0, db = 1.0;
  if (v < 0.1242)
  {
    db = 0.504 + ((1.0 - 0.504) / 0.1242) * v;
    dg = dr = 0.0;
  }
  else if (v < 0.3747)
  {
    db = 1.0;
    dr = 0.0;
    dg = (v - 0.1242) * (1.0 / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1.0 / (0.6253 - 0.3747));
    dg = 1.0;
    dr = (v - 0.3747) * (1.0 / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.0;
    dr = 1.0;
    dg = (0.8758 - v) * (1.0 / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.0;
    dg = 0.0;
    dr = 1.0 - (v - 0.8758) * ((1.0 - 0.504) / (1.0 - 0.8758));
  }

  r = static_cast<uint8_t>(255 * dr);
  g = static_cast<uint8_t>(255 * dg);
  b = static_cast<uint8_t>(255 * db);
}
}  // namespace

Eigen::Vector4d min_point;
double min_eigen_value;
int max_layer = 2;
int max_points = 100;
double voxel_size = 1.0;
int min_ba_point = 20;
vector<double> plane_eigen_value_thre;

std::vector<int> mp;
double imu_coef = 1e-4;

// Note: Plane::Plane() is now defined in core/types.cpp

LidarFactor::LidarFactor(int _w) : win_size(_w)
{
}

void LidarFactor::push_voxel(vector<PointCluster>& vec_orig, PointCluster& fix, double coe, Eigen::Vector3d& eig_value,
                             Eigen::Matrix3d& eig_vector, PointCluster& pcr_add)
{
  plvec_voxels.push_back(vec_orig);
  sig_vecs.push_back(fix);
  coeffs.push_back(coe);
  eig_values.push_back(eig_value);
  eig_vectors.push_back(eig_vector);
  pcr_adds.push_back(pcr_add);
}

void LidarFactor::acc_evaluate2(const vector<IMUST>& xs, int head, int end, Eigen::MatrixXd& Hess,
                                Eigen::VectorXd& JacT, double& residual)
{
  Hess.setZero();
  JacT.setZero();
  residual = 0;
  const int kk = 0;

  PLV(3) viRiTuk(win_size);
  PLM(3) viRiTukukT(win_size);

  vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
  Eigen::Matrix3d umumT;

  for (int a = head; a < end; a++)
  {
    vector<PointCluster>& sig_orig = plvec_voxels[a];
    double coe = coeffs[a];

    Eigen::Vector3d lmbd = eig_values[a];
    Eigen::Matrix3d U = eig_vectors[a];
    int NN = pcr_adds[a].N;
    Eigen::Vector3d vBar = pcr_adds[a].v / NN;

    Eigen::Vector3d u[3] = { U.col(0), U.col(1), U.col(2) };
    Eigen::Vector3d& uk = u[kk];
    Eigen::Matrix3d ukukT = uk * uk.transpose();
    umumT.setZero();
    for (int i = 0; i < 3; i++)
    {
      if (i != kk)
        umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();
    }

    // lambda
    for (int i = 0; i < win_size; i++)
    {
      if (sig_orig[i].N != 0)
      {
        Eigen::Matrix3d Pi = sig_orig[i].P;
        Eigen::Vector3d vi = sig_orig[i].v;
        Eigen::Matrix3d Ri = xs[i].R;
        double ni = sig_orig[i].N;

        Eigen::Matrix3d vihat;
        vihat << SKEW_SYM_MATRX(vi);
        Eigen::Vector3d RiTuk = Ri.transpose() * uk;
        Eigen::Matrix3d RiTukhat;
        RiTukhat << SKEW_SYM_MATRX(RiTuk);

        Eigen::Vector3d PiRiTuk = Pi * RiTuk;
        viRiTuk[i] = vihat * RiTuk;
        viRiTukukT[i] = viRiTuk[i] * uk.transpose();

        Eigen::Vector3d ti_v = xs[i].p - vBar;
        double ukTti_v = uk.dot(ti_v);

        Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
        Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
        Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
        Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
        Auk[i] /= NN;

        const Eigen::Matrix<double, 6, 1>& jjt = Auk[i].transpose() * uk;
        JacT.block<6, 1>(6 * i, 0) += coe * jjt;

        const Eigen::Matrix3d& HRt = 2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
        Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
        Hb.block<3, 3>(0, 0) += 2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat -
                                2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() - 0.5 * hat(jjt.block<3, 1>(0, 0));
        Hb.block<3, 3>(0, 3) += HRt;
        Hb.block<3, 3>(3, 0) += HRt.transpose();
        Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

        Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
      }
    }

    for (int i = 0; i < win_size - 1; i++)
    {
      if (sig_orig[i].N != 0)
      {
        double ni = sig_orig[i].N;
        for (int j = i + 1; j < win_size; j++)
          if (sig_orig[j].N != 0)
          {
            double nj = sig_orig[j].N;
            Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[j];
            Hb.block<3, 3>(0, 0) += -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
            Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
            Hb.block<3, 3>(3, 0) += -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
            Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

            Hess.block<6, 6>(6 * i, 6 * j) += coe * Hb;
          }
      }
    }

    residual += coe * lmbd[kk];
  }

  for (int i = 1; i < win_size; i++)
    for (int j = 0; j < i; j++)
      Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
}

void LidarFactor::evaluate_only_residual(const vector<IMUST>& xs, int head, int end, double& residual)
{
  residual = 0;
  int kk = 0;  // The kk-th lambda value

  PointCluster pcr;

  for (int a = head; a < end; a++)
  {
    const vector<PointCluster>& sig_orig = plvec_voxels[a];
    PointCluster sig = sig_vecs[a];

    for (int i = 0; i < win_size; i++)
      if (sig_orig[i].N != 0)
      {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }

    Eigen::Vector3d vBar = sig.v / sig.N;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P / sig.N - vBar * vBar.transpose());
    Eigen::Vector3d lmbd = saes.eigenvalues();
    Eigen::Matrix3d U = saes.eigenvectors();

    eig_values[a] = lmbd;
    eig_vectors[a] = U;
    pcr_adds[a] = sig;

    residual += coeffs[a] * lmbd[kk];
  }
}

void LidarFactor::clear()
{
  sig_vecs.clear();
  plvec_voxels.clear();
  eig_values.clear();
  eig_vectors.clear();
  pcr_adds.clear();
  coeffs.clear();
}

// NormalFactor
NormalFactor::NormalFactor(int _w) : win_size(_w)
{
}

void NormalFactor::push_voxel(std::vector<PointCluster>& vec_orig, PointCluster& fix, double coe,
                              Eigen::Vector3d& n_ref, PointCluster& pcr_add)
{
  plvec_voxels.push_back(vec_orig);
  sig_vecs.push_back(fix);
  coeffs.push_back(coe);
  n_refs.push_back(n_ref.normalized());
  pcr_adds.push_back(pcr_add);
}

void NormalFactor::acc_evaluate2(const std::vector<IMUST>& xs, int head, int end, Eigen::MatrixXd& Hess,
                                 Eigen::VectorXd& JacT, double& residual)
{
  Hess.setZero();
  JacT.setZero();
  residual = 0;

  const int kk = 0;
  const Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();

  // 【说明】和雷达因子一致：Auk[i] 把 “位姿微分 ξ_i” 映射到 “dC * uk”
  std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);

  auto hat = [](const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    return W;
  };

  const double eps_lambda = 1e-9;  // 【修改2】特征值差分母的退化保护

  for (int a = head; a < end; ++a)
  {
    std::vector<PointCluster>& sig_orig = plvec_voxels[a];
    const double coe = coeffs[a];

    // ========= 先做当前聚合 =========
    PointCluster sig = sig_vecs[a];
    PointCluster pcr;
    for (int i = 0; i < win_size; ++i)
    {
      if (sig_orig[i].N != 0)
      {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }
    }
    if (sig.N == 0)
      continue;  // 【修改3】空体素直接跳过，避免除零

    // 【修改4】用“当前 sig”求 NN 和 vBar（不要再用 pcr_adds 的缓存）
    const int NN = sig.N;
    const Eigen::Vector3d vBar = sig.v / double(NN);

    // ========= 协方差与特征 =========
    const Eigen::Matrix3d C = sig.P / double(NN) - vBar * vBar.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(C);
    const Eigen::Vector3d lmbd = saes.eigenvalues();
    const Eigen::Matrix3d U = saes.eigenvectors();

    const Eigen::Vector3d u[3] = { U.col(0), U.col(1), U.col(2) };
    const Eigen::Vector3d& uk = u[kk];  // 列向量本身单位，无需再 normalized

    // ========= 残差 r = S * uk =========
    const Eigen::Vector3d& n_ref = n_refs[a];  // 在 push_voxel 已单位化
    const Eigen::Matrix3d S = I33 - n_ref * n_ref.transpose();
    const Eigen::Vector3d r = S * uk;
    residual += 0.5 * coe * r.squaredNorm();

    // ========= 特征向量扰动核 Tn =========
    Eigen::Matrix3d Tn = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
    {
      if (i == kk)
        continue;
      double denom = lmbd[kk] - lmbd[i];
      if (std::abs(denom) < eps_lambda)  // 【修改5】退化保护：夹紧分母
        denom = (denom >= 0 ? eps_lambda : -eps_lambda);
      Tn += (u[i] * u[i].transpose()) / denom;
    }

    // ========= 构造 Auk[i]（与雷达因子完全一致） =========
    for (int i = 0; i < win_size; ++i)
    {
      Auk[i].setZero();
      if (sig_orig[i].N == 0)
        continue;

      const Eigen::Matrix3d& Pi = sig_orig[i].P;
      const Eigen::Vector3d& vi = sig_orig[i].v;
      const Eigen::Matrix3d& Ri = xs[i].R;
      const double ni = static_cast<double>(sig_orig[i].N);

      const Eigen::Matrix3d vihat = hat(vi);
      const Eigen::Vector3d RiTuk = Ri.transpose() * uk;
      const Eigen::Matrix3d RiTukhat = hat(RiTuk);
      const Eigen::Vector3d PiRiTuk = Pi * RiTuk;

      const Eigen::Vector3d ti_v = xs[i].p - vBar;
      const double ukTti_v = uk.dot(ti_v);

      const Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
      const Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;

      Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
      Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
      Auk[i] /= double(NN);  // 【修改6】用 double(NN)
    }

    // ========= 先缓存 Ji，避免交叉块重复乘 =========
    std::vector<Eigen::Matrix<double, 3, 6>> Ji_cache(win_size);
    for (int i = 0; i < win_size; ++i)
    {
      if (sig_orig[i].N == 0)
        continue;
      Ji_cache[i] = S * Tn * Auk[i];  // J_i = S * Tn * Auk[i]

      JacT.block<6, 1>(6 * i, 0) += coe * Ji_cache[i].transpose() * r;
      Hess.block<6, 6>(6 * i, 6 * i) += coe * (Ji_cache[i].transpose() * Ji_cache[i]);
    }

    for (int i = 0; i < win_size - 1; ++i)
    {
      if (sig_orig[i].N == 0)
        continue;
      for (int j = i + 1; j < win_size; ++j)
      {
        if (sig_orig[j].N == 0)
          continue;
        Hess.block<6, 6>(6 * i, 6 * j) += coe * (Ji_cache[i].transpose() * Ji_cache[j]);  // 【修改7】复用缓存
      }
    }
  }

  // 对称化（与雷达因子一致）
  for (int i = 1; i < win_size; ++i)
    for (int j = 0; j < i; ++j)
      Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
}

void NormalFactor::evaluate_only_residual(const std::vector<IMUST>& xs, int head, int end, double& residual)
{
  residual = 0.0;

  PointCluster pcr;

  for (int a = head; a < end; ++a)
  {
    const std::vector<PointCluster>& sig_orig = plvec_voxels[a];
    PointCluster sig = sig_vecs[a];

    for (int i = 0; i < win_size; ++i)
    {
      if (sig_orig[i].N != 0)
      {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }
    }

    // 【修改1】空体素保护，避免后面除以 sig.N
    if (sig.N == 0)
    {
      pcr_adds[a] = sig;
      continue;
    }

    // 【修改2】显式用 double，避免整除精度问题
    const Eigen::Vector3d vBar = sig.v / static_cast<double>(sig.N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P / static_cast<double>(sig.N) - vBar * vBar.transpose());
    const Eigen::Matrix3d U = saes.eigenvectors();
    const Eigen::Vector3d uk = U.col(0);  // 【修改3】特征向量本来就是单位向量，不再 normalized()

    const Eigen::Vector3d& n = n_refs[a];

    // 【修改4】如果你在 push_voxel() 里已 n_ref.normalize()，就用正交投影 S = I - nn^T：
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity() - n * n.transpose();

    const Eigen::Vector3d r = S * uk;
    residual += 0.5 * coeffs[a] * r.squaredNorm();

    pcr_adds[a] = sig;
  }
}

void NormalFactor::clear()
{
  sig_vecs.clear();
  plvec_voxels.clear();
  coeffs.clear();
  n_refs.clear();
  pcr_adds.clear();
}

// The LM optimizer for LiDAR BA

double Lidar_BA_Optimizer::divide_thread(vector<IMUST>& x_stats, LidarFactor& voxhess, Eigen::MatrixXd& Hess,
                                         Eigen::VectorXd& JacT)
{
  double residual = 0;

  Hess.setZero();
  JacT.setZero();

  PLM(-1) hessians(thd_num);  // Heisen matrix storage for each thread
  PLV(-1) jacobins(thd_num);  // Jacobian matrix storage for each thread

  for (int i = 0; i < thd_num; i++)
  {
    hessians[i].resize(jac_leng, jac_leng);
    jacobins[i].resize(jac_leng);
  }

  int tthd_num = thd_num;
  vector<double> resis(tthd_num, 0);

  int g_size = voxhess.plvec_voxels.size();
  if (g_size < tthd_num)
    tthd_num = 1;  // Only 1 thread is used when the number of voxels is smaller than the number of threads

  // Task segmentation and thread startup
  vector<thread*> mthreads(tthd_num);
  double part = 1.0 * g_size / tthd_num;

  // Start the child thread (starting from the first thread, the main thread handles part 0)
  for (int i = 1; i < tthd_num; i++)
  {
    mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1), ref(hessians[i]),
                             ref(jacobins[i]), ref(resis[i]));
  }

  // Wait for the child thread to complete and aggregate the results
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

  // printf("resi: %lf\n", residual);

  for (int i = 0; i < tthd_num; i++)
  {
    // mthreads[i]->join();
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

  // ---------- 线程缓存（IMU 仍然串行累加） ----------
  PLM(-1) hessians_l(thd_num), hessians_n(thd_num);  // lidar & normal
  PLV(-1) jacobins_l(thd_num), jacobins_n(thd_num);
  std::vector<double> resis_l(thd_num, 0.0), resis_n(thd_num, 0.0);

  for (int i = 0; i < thd_num; ++i)
  {
    hessians_l[i].resize(jac_leng, jac_leng);
    jacobins_l[i].resize(jac_leng);
    hessians_n[i].resize(jac_leng, jac_leng);
    jacobins_n[i].resize(jac_leng);
  }

  // ---------- LidarFactor 并行 ----------
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

  // ---------- IMU 串行 ----------
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

  // ---------- 等待 Lidar 子线程，叠加 ----------
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

  // ---------- NormalFactor 并行（与 Lidar 同构） ----------
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
    // printf("Too Less Voxel"); exit(0);
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

  // ---------- IMU ----------
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < win_size - 1; ++i)
    residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
  residual1 *= (imu_coef * 0.5);

  // ---------- Lidar 并行 ----------
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

  // ---------- Normal 并行 ----------
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
  // for (int i = 0; i < 3; i++)
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

    // printf("onlyresi: %lf\n", tl2-tl1);
    resitime += tl2 - tl1;

    q = (residual1 - residual2);
    // printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q/q1, q1,
    // q);

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

  // printf("ba: %lf %lf %zu\n", hesstime, resitime, voxhess.plvec_voxels.size());
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
      residual1 = divide_thread(x_stats, lidar, normal, imus_factor, Hess, JacT);  // ★ 新重载
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

    residual2 = only_residual(x_stats_temp, lidar, normal, imus_factor);  // ★ 新重载
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
  // 1) IMU 残差（与 only_residual 中完全一致的缩放：imu_coef * 0.5）
  double e_imu = 0.0;
  Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
  Eigen::VectorXd gg(2 * DIM);
  for (int i = 0; i < (int)xs.size() - 1; ++i)
  {
    jtj.setZero();
    gg.setZero();
    // false 表示只取残差能量（不累加 Hess/Jac）
    e_imu += imus[i]->give_evaluate(xs[i], xs[i + 1], jtj, gg, false);
  }
  e_imu *= (imu_coef * 0.5);

  // 2) LiDAR 残差：∑ c_j * λ_min
  double e_lidar = 0.0;
  lidar.evaluate_only_residual(xs, 0, (int)lidar.plvec_voxels.size(), e_lidar);

  // 3) Normal 残差：∑ (1/2) α_j ||r_j||^2
  double e_normal = 0.0;
  normal.evaluate_only_residual(xs, 0, (int)normal.plvec_voxels.size(), e_normal);

  // 4) 打印
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
  // for(int i=0; i<tthd_num; i++)
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

  // printf("resi: %lf\n", residual);

  for (int i = 0; i < tthd_num; i++)
  {
    // mthreads[i]->join();
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
    // printf("Too Less Voxel"); exit(0);
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

    // Hess.rightCols(3).setZero();
    // Hess.bottomRows(3).setZero();
    // Hess.block<3, 3>(imu_leng-3, imu_leng-3).setIdentity();
    // JacT.tail(3).setZero();

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
    // printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q/q1, q1,
    // q);

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

Keyframe::Keyframe(IMUST& _x0) : x0(_x0), exist(0)
{
  plptr.reset(new pcl::PointCloud<PointType>());
}

void Keyframe::generate(pcl::PointCloud<PointType>& pl_send, Eigen::Matrix3d rot, Eigen::Vector3d tra)
{
  Eigen::Vector3d v3;
  for (PointType ap : plptr->points)
  {
    v3 << ap.x, ap.y, ap.z;
    v3 = rot * v3 + tra;
    ap.x = v3[0];
    ap.y = v3[1];
    ap.z = v3[2];
    pl_send.push_back(ap);
  }
}

// The sldingwindow in each voxel nodes
SlideWindow::SlideWindow(int wdsize)
{
  pcrs_local.resize(wdsize);
  points.resize(wdsize);
  for (int i = 0; i < wdsize; i++)
    points[i].reserve(20);
}

void SlideWindow::resize(int wdsize)
{
  if (static_cast<int>(points.size()) != wdsize)
  {
    points.resize(wdsize);
    pcrs_local.resize(wdsize);
  }
}

void SlideWindow::clear()
{
  int wdsize = points.size();
  for (int i = 0; i < wdsize; i++)
  {
    points[i].clear();
    pcrs_local[i].clear();
  }
}

// The octotree map for odometry and local mapping

extern std::vector<int> mp;

OctoTree::OctoTree(int _l, int _w) : layer(_l), octo_state(0), wdsize(_w)
{
  for (int i = 0; i < 8; i++)
    leaves[i] = nullptr;
  cov_add.setZero();
}

void OctoTree::push(int ord, const pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws)
{
  mVox.lock();
  if (sw == nullptr)
  {
    if (sws.size() != 0)
    {
      sw = sws.back();
      sws.pop_back();
      sw->resize(wdsize);
    }
    else
      sw = new SlideWindow(wdsize);
  }
  if (!isexist)
    isexist = true;

  int mord = mp[ord];
  if (layer < max_layer)
    sw->points[mord].push_back(pv);
  sw->pcrs_local[mord].push(pv.pnt);
  pcr_add.push(pw);
  Eigen::Matrix<double, 9, 9> Bi;
  Bf_var(pv, Bi, pw);
  cov_add += Bi;
  mVox.unlock();
}

void OctoTree::push_fix(pointVar& pv)
{
  if (layer < max_layer)
    point_fix.push_back(pv);
  pcr_fix.push(pv.pnt);
  pcr_add.push(pv.pnt);
  Eigen::Matrix<double, 9, 9> Bi;
  Bf_var(pv, Bi, pv.pnt);
  cov_add += Bi;
}

void OctoTree::push_fix_novar(pointVar& pv)
{
  if (layer < max_layer)
    point_fix.push_back(pv);
  pcr_fix.push(pv.pnt);
  pcr_add.push(pv.pnt);
}

bool OctoTree::plane_judge(Eigen::Vector3d& eig_values)
{
  return (eig_values[0] < min_eigen_value && (eig_values[0] / eig_values[2]) < plane_eigen_value_thre[layer]);
}

void OctoTree::allocate(int ord, const pointVar& pv, const Eigen::Vector3d& pw, std::vector<SlideWindow*>& sws)
{
  if (octo_state == 0)
  {
    push(ord, pv, pw, sws);
  }
  else
  {
    int xyz[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
      if (pw[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (leaves[leafnum] == nullptr)
    {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->allocate(ord, pv, pw, sws);
  }
}

void OctoTree::allocate_fix(pointVar& pv)
{
  if (octo_state == 0)
  {
    push_fix_novar(pv);
  }
  else if (layer < max_layer)
  {
    int xyz[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
      if (pv.pnt[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (leaves[leafnum] == nullptr)
    {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->allocate_fix(pv);
  }
}

void OctoTree::fix_divide(std::vector<SlideWindow*>& /*sws*/)
{
  for (pointVar& pv : point_fix)
  {
    int xyz[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
      if (pv.pnt[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves[leafnum] == nullptr)
    {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->push_fix(pv);
  }
}

void OctoTree::subdivide(int si, IMUST& xx, std::vector<SlideWindow*>& sws)
{
  for (pointVar& pv : sw->points[mp[si]])
  {
    Eigen::Vector3d pw = xx.R * pv.pnt + xx.p;
    int xyz[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
      if (pw[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves[leafnum] == nullptr)
    {
      leaves[leafnum] = new OctoTree(layer + 1, wdsize);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }

    leaves[leafnum]->push(si, pv, pw, sws);
  }
}

void OctoTree::plane_update()
{
  plane.center = pcr_add.v / pcr_add.N;
  int l = 0;
  Eigen::Vector3d u[3] = { eig_vector.col(0), eig_vector.col(1), eig_vector.col(2) };
  double nv = 1.0 / pcr_add.N;

  Eigen::Matrix<double, 3, 9> u_c;
  u_c.setZero();
  for (int k = 0; k < 3; k++)
    if (k != l)
    {
      Eigen::Matrix3d ukl = u[k] * u[l].transpose();
      Eigen::Matrix<double, 1, 9> fkl;
      fkl.head(6) << ukl(0, 0), ukl(1, 0) + ukl(0, 1), ukl(2, 0) + ukl(0, 2), ukl(1, 1), ukl(1, 2) + ukl(2, 1),
          ukl(2, 2);
      fkl.tail(3) = -(u[k].dot(plane.center) * u[l] + u[l].dot(plane.center) * u[k]);

      u_c += nv / (eig_value[l] - eig_value[k]) * u[k] * fkl;
    }

  Eigen::Matrix<double, 3, 9> Jc = u_c * cov_add;
  plane.plane_var.block<3, 3>(0, 0) = Jc * u_c.transpose();
  Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
  plane.plane_var.block<3, 3>(0, 3) = Jc_N;
  plane.plane_var.block<3, 3>(3, 0) = Jc_N.transpose();
  plane.plane_var.block<3, 3>(3, 3) = nv * nv * cov_add.block<3, 3>(6, 6);
  plane.normal = u[0];
  plane.radius = eig_value[2];
  plane.is_update = true;
  plane.is_normal_update = true;
}

void OctoTree::recut(int win_count, std::vector<IMUST>& x_buf, std::vector<SlideWindow*>& sws)
{
  if (octo_state == 0)
  {
    if (plane.is_plane && plane.normal.squaredNorm() > 1e-12)
    {
      plane.normal_prev = plane.normal;
    }
    else
    {
      plane.normal_prev.setZero();
    }
  }

  if (octo_state == 0)
  {
    if (layer >= 0)
    {
      opt_state = -1;
      if (pcr_add.N <= min_point[layer])
      {
        plane.is_plane = false;
        return;
      }
      if (!isexist || sw == nullptr)
        return;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
      eig_value = saes.eigenvalues();
      eig_vector = saes.eigenvectors();
      plane.is_plane = plane_judge(eig_value);

      if (plane.is_plane)
      {
        return;
      }
      else if (layer >= max_layer)
        return;
    }

    if (pcr_fix.N != 0)
    {
      fix_divide(sws);
      PVec().swap(point_fix);
    }

    for (int i = 0; i < win_count; i++)
      subdivide(i, x_buf[i], sws);

    sw->clear();
    sws.push_back(sw);
    sw = nullptr;
    octo_state = 1;
  }

  for (int i = 0; i < 8; i++)
    if (leaves[i] != nullptr)
      leaves[i]->recut(win_count, x_buf, sws);
}

void OctoTree::margi(int win_count, int mgsize, std::vector<IMUST>& x_buf, const LidarFactor& vox_opt)
{
  if (octo_state == 0 && layer >= 0)
  {
    if (!isexist || sw == nullptr)
      return;
    mVox.lock();
    vector<PointCluster> pcrs_world(wdsize);

    if (opt_state >= int(vox_opt.pcr_adds.size()))
    {
      printf("Error: opt_state: %d %zu\n", opt_state, vox_opt.pcr_adds.size());
      exit(0);
    }

    if (opt_state >= 0)
    {
      pcr_add = vox_opt.pcr_adds[opt_state];
      eig_value = vox_opt.eig_values[opt_state];
      eig_vector = vox_opt.eig_vectors[opt_state];
      opt_state = -1;

      for (int i = 0; i < mgsize; i++)
        if (sw->pcrs_local[mp[i]].N != 0)
        {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
        }
    }
    else
    {
      pcr_add = pcr_fix;
      for (int i = 0; i < win_count; i++)
        if (sw->pcrs_local[mp[i]].N != 0)
        {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
          pcr_add += pcrs_world[i];
        }

      if (plane.is_plane)
      {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        eig_value = saes.eigenvalues();
        eig_vector = saes.eigenvectors();
      }
    }

    if (pcr_fix.N < max_points && plane.is_plane)
      if (pcr_add.N - last_num >= 5 || last_num <= 10)
      {
        plane_update();
        last_num = pcr_add.N;
      }

    if (pcr_fix.N < max_points)
    {
      for (int i = 0; i < mgsize; i++)
        if (pcrs_world[i].N != 0)
        {
          pcr_fix += pcrs_world[i];
          for (pointVar pv : sw->points[mp[i]])
          {
            pv.pnt = x_buf[i].R * pv.pnt + x_buf[i].p;
            point_fix.push_back(pv);
          }
        }
    }
    else
    {
      for (int i = 0; i < mgsize; i++)
        if (pcrs_world[i].N != 0)
          pcr_add -= pcrs_world[i];

      if (point_fix.size() != 0)
        PVec().swap(point_fix);
    }

    for (int i = 0; i < mgsize; i++)
      if (sw->pcrs_local[mp[i]].N != 0)
      {
        sw->pcrs_local[mp[i]].clear();
        sw->points[mp[i]].clear();
      }

    if (pcr_fix.N >= pcr_add.N)
      isexist = false;
    else
      isexist = true;

    mVox.unlock();
  }
  else
  {
    isexist = false;
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
      {
        leaves[i]->margi(win_count, mgsize, x_buf, vox_opt);
        isexist = isexist || leaves[i]->isexist;
      }
  }
}

// Extract the LiDAR factor
void OctoTree::tras_opt(LidarFactor& vox_opt)
{
  if (octo_state == 0)
  {
    if (layer >= 0 && isexist && plane.is_plane && sw != nullptr)
    {
      if (eig_value[0] / eig_value[1] > 0.12)
        return;

      double coe = 1;
      vector<PointCluster> pcrs(wdsize);
      for (int i = 0; i < wdsize; i++)
        pcrs[i] = sw->pcrs_local[mp[i]];
      opt_state = vox_opt.plvec_voxels.size();
      vox_opt.push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->tras_opt(vox_opt);
  }
}

// Extract the Normal factor
void OctoTree::tras_opt(NormalFactor& vox_opt)
{
  if (octo_state == 0)
  {
    if (layer >= 0 && isexist && plane.is_plane && sw != nullptr)
    {
      if (eig_value[0] / eig_value[1] > 0.12)
        return;

      std::vector<PointCluster> pcrs(wdsize);
      for (int i = 0; i < wdsize; ++i)
        pcrs[i] = sw->pcrs_local[mp[i]];

      double alpha = 1.0;
      Eigen::Vector3d n_ref = plane.normal.normalized();

      vox_opt.push_voxel(pcrs, pcr_fix, alpha, n_ref, pcr_add);
    }
  }
  else
  {
    for (int i = 0; i < 8; ++i)
      if (leaves[i] != nullptr)
        leaves[i]->tras_opt(vox_opt);
  }
}

int OctoTree::match(Eigen::Vector3d& wld, Plane*& pla, double& max_prob, Eigen::Matrix3d& var_wld, double& sigma_d,
                    OctoTree*& oc)
{
  int flag = 0;
  if (octo_state == 0)
  {
    if (plane.is_plane)
    {
      float dis_to_plane = fabs(plane.normal.dot(wld - plane.center));
      float dis_to_center = (plane.center - wld).squaredNorm();
      float range_dis = (dis_to_center - dis_to_plane * dis_to_plane);
      if (range_dis <= 3 * 3 * plane.radius)
      {
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = wld - plane.center;
        J_nq.block<1, 3>(0, 3) = -plane.normal;
        double sigma_l = J_nq * plane.plane_var * J_nq.transpose();
        sigma_l += plane.normal.transpose() * var_wld * plane.normal;
        if (dis_to_plane < 3 * sqrt(sigma_l))
        {
          {
            oc = this;
            sigma_d = sigma_l;
            pla = &plane;
          }

          flag = 1;
        }
      }
    }
  }
  else
  {
    int xyz[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
      if (wld[k] > voxel_center[k])
        xyz[k] = 1;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (leaves[leafnum] != nullptr)
      flag = leaves[leafnum]->match(wld, pla, max_prob, var_wld, sigma_d, oc);
  }

  return flag;
}

void OctoTree::tras_ptr(vector<OctoTree*>& octos_release)
{
  if (octo_state == 1)
  {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
      {
        octos_release.push_back(leaves[i]);
        leaves[i]->tras_ptr(octos_release);
      }
  }
}

void OctoTree::delete_ptr()
{
  for (int i = 0; i < 8; i++)
  {
    if (leaves[i] != nullptr)
    {
      leaves[i]->delete_ptr();
      delete leaves[i];
      leaves[i] = nullptr;
    }
  }
  if (sw != nullptr)
  {
    delete sw;
    sw = nullptr;
  }
}

void OctoTree::tras_display(int win_count, pcl::PointCloud<PointType>& pl_fixd, pcl::PointCloud<PointType>& pl_wind,
                            std::vector<IMUST>& x_buf)
{
  if (octo_state == 0)
  {
    if (sw == nullptr)
      return;
    const int sw_size = static_cast<int>(sw->points.size());
    if (sw_size == 0)
      return;
    const int cnt = std::min(win_count, sw_size);
    (void)pcr_add;  // Covariance computation retained for potential future use

    PointType ap;

    if (plane.is_plane)
    {
      for (int i = 0; i < cnt; i++)
      {
        const int idx = mp[i];
        if (idx < 0 || idx >= sw_size)
          continue;
        if (i >= static_cast<int>(x_buf.size()))
          break;
        for (pointVar& pv : sw->points[idx])
        {
          Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;
          ap.x = pvec[0];
          ap.y = pvec[1];
          ap.z = pvec[2];

          pl_wind.push_back(ap);
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->tras_display(win_count, pl_fixd, pl_wind, x_buf);
  }
}

bool OctoTree::inside(Eigen::Vector3d& wld)
{
  double hl = quater_length * 2;
  return (wld[0] >= voxel_center[0] - hl && wld[0] <= voxel_center[0] + hl && wld[1] >= voxel_center[1] - hl &&
          wld[1] <= voxel_center[1] + hl && wld[2] >= voxel_center[2] - hl && wld[2] <= voxel_center[2] + hl);
}

void OctoTree::clear_slwd(std::vector<SlideWindow*>& sws)
{
  if (octo_state != 0)
  {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
      {
        leaves[i]->clear_slwd(sws);
      }
  }

  if (sw != nullptr)
  {
    sw->clear();
    sws.push_back(sw);
    sw = nullptr;
  }
}

void OctoTree::collect_plane_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                                     std::unordered_set<int>& used_ids, float alpha, double max_trace, double pow_num)
{
  if (layer > max_layer)
    return;

  if (octo_state == 0)
  {
    if (!plane.is_plane && plane.is_published)
    {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (used_ids.insert(id).second)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.ns = "plane";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        out.markers.push_back(marker);
      }
      plane.is_published = false;
      plane.is_update = false;
    }

    if (plane.is_plane && plane.is_update)
    {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (!used_ids.insert(id).second)
        return;
      const Eigen::Vector3d cov_diag = plane.plane_var.block<3, 3>(0, 0).diagonal();
      double trace = cov_diag.sum();
      if (trace >= max_trace)
        trace = max_trace;
      trace = trace * (1.0 / max_trace);
      trace = std::pow(trace, pow_num);

      uint8_t r = 255, g = 255, b = 255;
      map_jet(trace, 0.0, 1.0, r, g, b);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "plane";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = plane.center[0];
      marker.pose.position.y = plane.center[1];
      marker.pose.position.z = plane.center[2];

      const Eigen::Vector3d n = plane.normal.normalized();
      const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n);
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      const double ev0 = std::max(0.0, eig_value[0]);
      const double ev1 = std::max(0.0, eig_value[1]);
      const double ev2 = std::max(0.0, eig_value[2]);
      marker.scale.x = 3.0 * std::sqrt(ev2);
      marker.scale.y = 3.0 * std::sqrt(ev1);
      marker.scale.z = 2.0 * std::sqrt(ev0);

      marker.color.a = alpha;
      marker.color.r = r / 255.0f;
      marker.color.g = g / 255.0f;
      marker.color.b = b / 255.0f;

      out.markers.push_back(marker);
      plane.is_update = false;
      plane.is_published = true;
    }
    return;
  }
  else if (plane.is_published)
  {
    const int id = voxel_id_from_center(voxel_center, layer);
    if (used_ids.insert(id).second)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "plane";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(marker);
    }
    plane.is_published = false;
    plane.is_update = false;
  }

  for (int i = 0; i < 8; ++i)
    if (leaves[i] != nullptr)
      leaves[i]->collect_plane_markers(out, max_layer, used_ids, alpha, max_trace, pow_num);
}

void OctoTree::collect_normal_markers(visualization_msgs::msg::MarkerArray& out, int max_layer,
                                      std::unordered_set<int>& used_ids, float alpha, double max_trace, double pow_num)
{
  if (layer > max_layer)
    return;

  if (octo_state == 0)
  {
    if (!plane.is_plane && plane.is_normal_published)
    {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (used_ids.insert(id).second)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.ns = "normal";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        out.markers.push_back(marker);
      }
      plane.is_normal_published = false;
      plane.is_normal_update = false;
    }

    if (plane.is_plane && plane.is_normal_update)
    {
      const int id = voxel_id_from_center(voxel_center, layer);
      if (!used_ids.insert(id).second)
        return;

      const Eigen::Vector3d cov_diag = plane.plane_var.block<3, 3>(0, 0).diagonal();
      double trace = cov_diag.sum();
      if (trace >= max_trace)
        trace = max_trace;
      trace = trace * (1.0 / max_trace);
      trace = std::pow(trace, pow_num);

      uint8_t r = 255, g = 255, b = 255;
      map_jet(trace, 0.0, 1.0, r, g, b);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "normal";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      const double length = 2.0 * quater_length;
      geometry_msgs::msg::Point p0;
      p0.x = plane.center[0];
      p0.y = plane.center[1];
      p0.z = plane.center[2];

      const Eigen::Vector3d n = plane.normal.normalized();
      geometry_msgs::msg::Point p1;
      p1.x = plane.center[0] + n[0] * length;
      p1.y = plane.center[1] + n[1] * length;
      p1.z = plane.center[2] + n[2] * length;

      marker.points.push_back(p0);
      marker.points.push_back(p1);

      marker.scale.x = 0.1 * length;
      marker.scale.y = 0.2 * length;
      marker.scale.z = 0.0;

      marker.color.a = alpha;
      marker.color.r = r / 255.0f;
      marker.color.g = g / 255.0f;
      marker.color.b = b / 255.0f;

      out.markers.push_back(marker);
      plane.is_normal_update = false;
      plane.is_normal_published = true;
    }
    return;
  }
  else if (plane.is_normal_published)
  {
    const int id = voxel_id_from_center(voxel_center, layer);
    if (used_ids.insert(id).second)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "normal";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(marker);
    }
    plane.is_normal_published = false;
    plane.is_normal_update = false;
  }

  for (int i = 0; i < 8; ++i)
    if (leaves[i] != nullptr)
      leaves[i]->collect_normal_markers(out, max_layer, used_ids, alpha, max_trace, pow_num);
}

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

  // Output downsampled point cloud, each point contains mean coordinates and diagonal covariance
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
  {
    pointVar& pv = iter->second.first;
    ap.x = pv.pnt[0];
    ap.y = pv.pnt[1];
    ap.z = pv.pnt[2];
    ap.normal_x = pv.var(0, 0);  // Covariance diagonals are stored as normal
    ap.normal_y = pv.var(1, 1);
    ap.normal_z = pv.var(2, 2);
    pl_keep.push_back(ap);
  }
}

void Bf_var(const pointVar& pv, Eigen::Matrix<double, 9, 9>& bcov, const Eigen::Vector3d& vec)
{
  Eigen::Matrix<double, 6, 3> Bi;
  Bi << 2 * vec(0), 0, 0, vec(1), vec(0), 0, vec(2), 0, vec(0), 0, 2 * vec(1), 0, 0, vec(2), vec(1), 0, 0, 2 * vec(2);
  Eigen::Matrix<double, 6, 3> Biup = Bi * pv.var;
  bcov.block<6, 6>(0, 0) = Biup * Bi.transpose();
  bcov.block<6, 3>(0, 6) = Biup;
  bcov.block<3, 6>(6, 0) = Biup.transpose();
  bcov.block<3, 3>(6, 6) = pv.var;
}
