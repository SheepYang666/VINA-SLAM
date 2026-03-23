#include "vina_slam/pipeline/initialization.hpp"
#include "vina_slam/core/point_utils.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/mapping/voxel_map.hpp"
#include "vina_slam/sensor/sync.hpp"

#include <Eigen/Eigenvalues>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

Initialization::Initialization(const rclcpp::Node::SharedPtr& node_in) : node(node_in)
{
}

Initialization& Initialization::instance(const rclcpp::Node::SharedPtr& node_in)
{
  static Initialization inst(node_in);
  return inst;
}

Initialization& Initialization::instance()
{
  rclcpp::Node::SharedPtr node_temp;
  return instance(node_temp);
}

void Initialization::align_gravity(vector<IMUST>& xs)
{
  Eigen::Vector3d g0 = xs[0].g;
  Eigen::Vector3d n0 = g0.normalized();

  Eigen::Vector3d n1(0, 0, 1);

  // If the current direction is facing down, set the target direction to the negative z axis.
  if (n0[2] < 0)
  {
    n1[2] = -1;
  }

  Eigen::Vector3d rotvec = n0.cross(n1);  // Calculate the rotation vector that rotates n0 to n1 (axis = n0xn1)
  double rnorm = rotvec.norm();           // The rotation angle (sin(theta)) is the modular length of the rotation vector
  rotvec = rotvec / rnorm;                // Uniform rotation axis

  // Construct rotation: the angle is asin(rnorm), the axis is rotvec
  Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
  Eigen::Matrix3d rot = angaxis.toRotationMatrix();
  g0 = rot * g0;  // Rotate the gravity vector of the first state to obtain a new unified gravity direction

  // Get the first position as the reference point and apply rotation transformation to all states
  Eigen::Vector3d p0 = xs[0].p;
  for (int i = 0; i < xs.size(); i++)
  {
    xs[i].p = rot * (xs[i].p - p0) + p0;

    xs[i].R = rot * xs[i].R;

    xs[i].v = rot * xs[i].v;

    xs[i].g = g0;
  }
}

void Initialization::motion_blur(pcl::PointCloud<PointType>& pl, PVec& pvec, IMUST xc, IMUST xl,
                                 deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus, double pcl_beg_time,
                                 IMUST& extrin_para)
{
  // Initialize bias + current IMU status variable
  xc.bg = xl.bg;
  xc.ba = xl.ba;
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
  Eigen::Matrix3d R_imu(xc.R);
  vector<IMUST> imu_poses;

  // IMU Integral build status sequence imu_poses
  for (auto it_imu = imus.end() - 1; it_imu != imus.begin(); it_imu--)
  {
    sensor_msgs::msg::Imu& head = **(it_imu - 1);  // Early time
    sensor_msgs::msg::Imu& tail = **(it_imu);      // Time tonight

    /*Mean filtering (interpolated angular velocity and acceleration)*/
    angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
        0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
        0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
    acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
        0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
        0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

    angvel_avr -= xc.bg;
    acc_avr = acc_avr * imupre_scale_gravity - xc.ba;

    // Pose increment (exp map)
    double dt = rclcpp::Time(head.header.stamp).seconds() - rclcpp::Time(tail.header.stamp).seconds();
    Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
    Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

    // Linear acceleration (for integration) in the world coordinate system, state integral: position, velocity,
    // posture
    acc_imu = R_imu * acc_avr + xc.g;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;
    R_imu = R_imu * Exp_f;

    // Save the current status to imu_poses
    double offt = rclcpp::Time(head.header.stamp).seconds() - pcl_beg_time;
    imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
  }

  pointVar pv;
  pv.var.setIdentity();

  // IMU compensation is not done without a time sequence number, and only point-to-point conversion to the IMU
  // coordinate system
  if (point_notime)
  {
    for (PointType& ap : pl.points)
    {
      pv.pnt << ap.x, ap.y, ap.z;
      pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
      pvec.push_back(pv);
    }

    return;
  }

  // If you have a time serial number, you can make IMU compensation

  auto it_pcl = pl.end() - 1;

  for (auto it_kp = imu_poses.begin(); it_kp != imu_poses.end(); it_kp++)
  {
    IMUST& head = *it_kp;
    R_imu = head.R;
    acc_imu = head.ba;
    vel_imu = head.v;
    pos_imu = head.p;
    angvel_avr = head.bg;

    for (; it_pcl->curvature > head.t; it_pcl--)
    {
      double dt = it_pcl->curvature - head.t;
      Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
      Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

      pv.pnt = P_compensate;
      pvec.push_back(pv);
      if (it_pcl == pl.begin())
      {
        break;
      }
    }
  }
}

int Initialization::motion_init(vector<pcl::PointCloud<PointType>::Ptr>& pl_origs,
                                vector<deque<std::shared_ptr<sensor_msgs::msg::Imu>>>& vec_imus,
                                vector<double>& beg_times, Eigen::MatrixXd* hess, LidarFactor& voxhess,
                                vector<IMUST>& x_buf, unordered_map<VOXEL_LOC, OctoTree*>& surf_map,
                                unordered_map<VOXEL_LOC, OctoTree*>& surf_map_slide, vector<PVecPtr>& pvec_buf,
                                int win_size, vector<vector<SlideWindow*>>& sws, IMUST& x_curr,
                                deque<IMU_PRE*>& imu_pre_buf, IMUST& extrin_para)
{
  PLV(3) pwld;

  double last_g_norm = x_buf[0].g.norm();
  int converge_flag = 0;

  double min_eigen_value_orig = min_eigen_value;
  vector<double> eigen_value_array_orig = plane_eigen_value_thre;

  min_eigen_value = 0.02;
  for (double& iter : plane_eigen_value_thre)
  {
    iter = 1.0 / 4;
  }

  double t0 = node->now().seconds();

  double converge_thre = 0.05;
  int converge_times = 0;

  bool is_degrade = true;

  Eigen::Vector3d eigvalue;
  eigvalue.setZero();

  // Reference to global dept_err, beam_err declared in platform/ros2/node
  extern double dept_err, beam_err;

  for (int iterCnt = 0; iterCnt < 10; iterCnt++)
  {
    if (converge_flag == 1)
    {
      min_eigen_value = min_eigen_value_orig;
      plane_eigen_value_thre = eigen_value_array_orig;
    }

    vector<OctoTree*> octos;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for (int i = 0; i < octos.size(); i++)
    {
      delete octos[i];
    }
    surf_map.clear();
    octos.clear();
    surf_map_slide.clear();

    for (int i = 0; i < win_size; i++)
    {
      pwld.clear();
      pvec_buf[i]->clear();

      int l = i == 0 ? i : i - 1;
      motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

      if (converge_flag == 1)
      {
        for (pointVar& pv : *pvec_buf[i])
        {
          calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
        }
        pvec_update(pvec_buf[i], x_buf[i], pwld);
      }
      else
      {
        for (pointVar& pv : *pvec_buf[i])
        {
          pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }
      }

      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    voxhess.clear();
    voxhess.win_size = win_size;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
    {
      iter->second->recut(win_size, x_buf, sws[0]);
      iter->second->tras_opt(voxhess);
    }

    if (voxhess.plvec_voxels.size() < 10)
    {
      break;
    }

    LI_BA_OptimizerGravity opt_lsv;
    vector<double> resis;

    opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
    Eigen::Matrix3d nnt;
    nnt.setZero();
    for (int i = 0; i < win_size - 1; i++)
    {
      delete imu_pre_buf[i];
    }
    imu_pre_buf.clear();

    for (int i = 1; i < win_size; i++)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[i - 1].bg, x_buf[i - 1].ba));
      imu_pre_buf.back()->push_imu(vec_imus[i]);
    }

    if (fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
    {
      for (Eigen::Matrix3d& iter : voxhess.eig_vectors)
      {
        Eigen::Vector3d v3 = iter.col(0);
        nnt += v3 * v3.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
      eigvalue = saes.eigenvalues();

      is_degrade = eigvalue[0] < 15 ? true : false;

      converge_thre = 0.01;
      if (converge_flag == 0)
      {
        align_gravity(x_buf);
        converge_flag = 1;
        continue;
      }
      else
      {
        break;
      }
    }
  }

  x_curr = x_buf[win_size - 1];
  double gnm = x_curr.g.norm();

  double g_min_value = 9.6;
  double g_max_vaue = 10.0;

  std::cout << "\teigvalue[0] = " << eigvalue[0] << std::endl;
  std::cout << "\teigvalue[1] = " << eigvalue[1] << std::endl;
  std::cout << "\teigvalue[2] = " << eigvalue[2] << std::endl;
  std::cout << "\tlast data frame |G| = " << gnm << std::endl;

  if (is_degrade)
  {
    std::cout << "\033[31m\tgravity direction degrade\033[0m" << std::endl;

    converge_flag = 0;
  }
  if (gnm < g_min_value || gnm > g_max_vaue)
  {
    std::cout << "\033[31m\tgravity value anomaly\033[0m" << std::endl;

    converge_flag = 0;
  }

  if (converge_flag == 0)
  {
    vector<OctoTree*> octos;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    for (int i = 0; i < octos.size(); i++)
      delete octos[i];
    surf_map.clear();
    octos.clear();
    surf_map_slide.clear();
  }

  Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y,
                       vec_imus[0][0]->angular_velocity.z);
  Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y,
                      vec_imus[0][0]->linear_acceleration.z);
  acc *= 9.8;

  pl_origs.clear();
  vec_imus.clear();
  beg_times.clear();
  double t1 = rclcpp::Clock().now().seconds();

  pcl::PointCloud<PointType> pcl_send;
  PointType pt;
  for (int i = 0; i < win_size; i++)
  {
    for (pointVar& pv : *pvec_buf[i])
    {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0];
      pt.y = vv[1];
      pt.z = vv[2];
      pcl_send.push_back(pt);
    }
  }

  return converge_flag;
}
