/**
 * @file factors.cpp
 * @brief Implementation of LidarFactor and NormalFactor
 */

#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/core/math.hpp"

namespace vina_slam {
namespace mapping {

LidarFactor::LidarFactor(int _w) : win_size(_w) {}

void LidarFactor::pushVoxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                            double coe, Eigen::Vector3d& eig_value, Eigen::Matrix3d& eig_vector,
                            core::PointCluster& pcr_add) {
  plvec_voxels.push_back(vec_orig);
  sig_vecs.push_back(fix);
  coeffs.push_back(coe);
  eig_values.push_back(eig_value);
  eig_vectors.push_back(eig_vector);
  pcr_adds.push_back(pcr_add);
}

void LidarFactor::accEvaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                               Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual) {
  Hess.setZero();
  JacT.setZero();
  residual = 0;
  const int kk = 0;

  PLV(3) viRiTuk(win_size);
  PLM(3) viRiTukukT(win_size);

  std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
  Eigen::Matrix3d umumT;

  for (int a = head; a < end; a++) {
    std::vector<core::PointCluster>& sig_orig = plvec_voxels[a];
    double coe = coeffs[a];

    Eigen::Vector3d lmbd = eig_values[a];
    Eigen::Matrix3d U = eig_vectors[a];
    int NN = pcr_adds[a].N;
    Eigen::Vector3d vBar = pcr_adds[a].v / NN;

    Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
    Eigen::Vector3d& uk = u[kk];
    Eigen::Matrix3d ukukT = uk * uk.transpose();
    umumT.setZero();
    for (int i = 0; i < 3; i++) {
      if (i != kk)
        umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();
    }

    // lambda
    for (int i = 0; i < win_size; i++) {
      if (sig_orig[i].N != 0) {
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
        Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * core::I33;
        Auk[i] /= NN;

        const Eigen::Matrix<double, 6, 1>& jjt = Auk[i].transpose() * uk;
        JacT.block<6, 1>(6 * i, 0) += coe * jjt;

        const Eigen::Matrix3d& HRt = 2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
        Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
        Hb.block<3, 3>(0, 0) +=
            2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat - 2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
            0.5 * hat(jjt.block<3, 1>(0, 0));
        Hb.block<3, 3>(0, 3) += HRt;
        Hb.block<3, 3>(3, 0) += HRt.transpose();
        Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

        Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
      }
    }

    for (int i = 0; i < win_size - 1; i++) {
      if (sig_orig[i].N != 0) {
        double ni = sig_orig[i].N;
        for (int j = i + 1; j < win_size; j++)
          if (sig_orig[j].N != 0) {
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

void LidarFactor::evaluateOnlyResidual(const std::vector<core::IMUST>& xs, int head, int end, double& residual) {
  residual = 0;
  int kk = 0; // The kk-th lambda value

  core::PointCluster pcr;

  for (int a = head; a < end; a++) {
    const std::vector<core::PointCluster>& sig_orig = plvec_voxels[a];
    core::PointCluster sig = sig_vecs[a];

    for (int i = 0; i < win_size; i++)
      if (sig_orig[i].N != 0) {
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

void LidarFactor::clear() {
  sig_vecs.clear();
  plvec_voxels.clear();
  eig_values.clear();
  eig_vectors.clear();
  pcr_adds.clear();
  coeffs.clear();
}

// NormalFactor implementation
NormalFactor::NormalFactor(int _w) : win_size(_w) {}

void NormalFactor::pushVoxel(std::vector<core::PointCluster>& vec_orig, core::PointCluster& fix,
                             double coe, Eigen::Vector3d& n_ref, core::PointCluster& pcr_add) {
  plvec_voxels.push_back(vec_orig);
  sig_vecs.push_back(fix);
  coeffs.push_back(coe);
  n_refs.push_back(n_ref.normalized());
  pcr_adds.push_back(pcr_add);
}

void NormalFactor::accEvaluate2(const std::vector<core::IMUST>& xs, int head, int end,
                                Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual) {
  Hess.setZero();
  JacT.setZero();
  residual = 0;

  const int kk = 0;
  const Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();

  std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);

  auto hat_fn = [](const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    return W;
  };

  const double eps_lambda = 1e-9;

  for (int a = head; a < end; ++a) {
    std::vector<core::PointCluster>& sig_orig = plvec_voxels[a];
    const double coe = coeffs[a];

    // Aggregate points
    core::PointCluster sig = sig_vecs[a];
    core::PointCluster pcr;
    for (int i = 0; i < win_size; ++i) {
      if (sig_orig[i].N != 0) {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }
    }
    if (sig.N == 0)
      continue;

    const int NN = sig.N;
    const Eigen::Vector3d vBar = sig.v / double(NN);

    // Covariance and eigen decomposition
    const Eigen::Matrix3d C = sig.P / double(NN) - vBar * vBar.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(C);
    const Eigen::Vector3d lmbd = saes.eigenvalues();
    const Eigen::Matrix3d U = saes.eigenvectors();

    const Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
    const Eigen::Vector3d& uk = u[kk];

    // Residual r = S * uk
    const Eigen::Vector3d& n_ref = n_refs[a];
    const Eigen::Matrix3d S = I33 - n_ref * n_ref.transpose();
    const Eigen::Vector3d r = S * uk;
    residual += 0.5 * coe * r.squaredNorm();

    // Eigenvector perturbation kernel Tn
    Eigen::Matrix3d Tn = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
      if (i == kk)
        continue;
      double denom = lmbd[kk] - lmbd[i];
      if (std::abs(denom) < eps_lambda)
        denom = (denom >= 0 ? eps_lambda : -eps_lambda);
      Tn += (u[i] * u[i].transpose()) / denom;
    }

    // Construct Auk[i]
    for (int i = 0; i < win_size; ++i) {
      Auk[i].setZero();
      if (sig_orig[i].N == 0)
        continue;

      const Eigen::Matrix3d& Pi = sig_orig[i].P;
      const Eigen::Vector3d& vi = sig_orig[i].v;
      const Eigen::Matrix3d& Ri = xs[i].R;
      const double ni = static_cast<double>(sig_orig[i].N);

      const Eigen::Matrix3d vihat = hat_fn(vi);
      const Eigen::Vector3d RiTuk = Ri.transpose() * uk;
      const Eigen::Matrix3d RiTukhat = hat_fn(RiTuk);
      const Eigen::Vector3d PiRiTuk = Pi * RiTuk;

      const Eigen::Vector3d ti_v = xs[i].p - vBar;
      const double ukTti_v = uk.dot(ti_v);

      const Eigen::Matrix3d combo1 = hat_fn(PiRiTuk) + vihat * ukTti_v;
      const Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;

      Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
      Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
      Auk[i] /= double(NN);
    }

    // Cache Ji
    std::vector<Eigen::Matrix<double, 3, 6>> Ji_cache(win_size);
    for (int i = 0; i < win_size; ++i) {
      if (sig_orig[i].N == 0)
        continue;
      Ji_cache[i] = S * Tn * Auk[i];

      JacT.block<6, 1>(6 * i, 0) += coe * Ji_cache[i].transpose() * r;
      Hess.block<6, 6>(6 * i, 6 * i) += coe * (Ji_cache[i].transpose() * Ji_cache[i]);
    }

    for (int i = 0; i < win_size - 1; ++i) {
      if (sig_orig[i].N == 0)
        continue;
      for (int j = i + 1; j < win_size; ++j) {
        if (sig_orig[j].N == 0)
          continue;
        Hess.block<6, 6>(6 * i, 6 * j) += coe * (Ji_cache[i].transpose() * Ji_cache[j]);
      }
    }
  }

  // Symmetrize
  for (int i = 1; i < win_size; ++i)
    for (int j = 0; j < i; ++j)
      Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
}

void NormalFactor::evaluateOnlyResidual(const std::vector<core::IMUST>& xs, int head, int end, double& residual) {
  residual = 0.0;

  core::PointCluster pcr;

  for (int a = head; a < end; ++a) {
    const std::vector<core::PointCluster>& sig_orig = plvec_voxels[a];
    core::PointCluster sig = sig_vecs[a];

    for (int i = 0; i < win_size; ++i) {
      if (sig_orig[i].N != 0) {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }
    }

    if (sig.N == 0) {
      pcr_adds[a] = sig;
      continue;
    }

    const Eigen::Vector3d vBar = sig.v / static_cast<double>(sig.N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(
        sig.P / static_cast<double>(sig.N) - vBar * vBar.transpose());
    const Eigen::Matrix3d U = saes.eigenvectors();
    const Eigen::Vector3d uk = U.col(0);

    const Eigen::Vector3d& n = n_refs[a];
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity() - n * n.transpose();

    const Eigen::Vector3d r = S * uk;
    residual += 0.5 * coeffs[a] * r.squaredNorm();

    pcr_adds[a] = sig;
  }
}

void NormalFactor::clear() {
  sig_vecs.clear();
  plvec_voxels.clear();
  coeffs.clear();
  n_refs.clear();
  pcr_adds.clear();
}

} // namespace mapping
} // namespace vina_slam
