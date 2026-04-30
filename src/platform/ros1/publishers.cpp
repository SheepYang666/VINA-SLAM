#include "vina_slam/platform/ros1/publishers.hpp"

#include <tf/transform_broadcaster.h>

ros::Publisher pub_scan;
ros::Publisher pub_cmap;
ros::Publisher pub_curr_path;
ros::Publisher pub_voxel_plane;
ros::Publisher pub_voxel_normal;

ResultOutput& ResultOutput::instance()
{
  static ResultOutput inst;
  return inst;
}

void ResultOutput::pub_odom_func(IMUST& xc)
{
  Eigen::Quaterniond q_this(xc.R);
  Eigen::Vector3d t_this = xc.p;
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(t_this.x(), t_this.y(), t_this.z()));
  transform.setRotation(tf::Quaternion(q_this.x(), q_this.y(), q_this.z(), q_this.w()));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_init", "aft_mapped"));
}

void ResultOutput::pub_localtraj(PLV(3) & pwld, double jour, IMUST& x_curr, int cur_session,
                                 pcl::PointCloud<PointType>& pcl_path)
{
  pub_odom_func(x_curr);

  pcl::PointCloud<PointType> pcl_send;
  pcl_send.reserve(pwld.size());

  for (Eigen::Vector3d& pw : pwld)
  {
    Eigen::Vector3d pvec = pw;
    PointType ap;
    ap.x = pvec.x();
    ap.y = pvec.y();
    ap.z = pvec.z();
    pcl_send.push_back(ap);
  }

  pub_pl_func(pcl_send, pub_scan);

  Eigen::Vector3d pcurr = x_curr.p;

  PointType ap;
  ap.x = pcurr[0];
  ap.y = pcurr[1];
  ap.z = pcurr[2];
  ap.curvature = jour;
  ap.intensity = cur_session;
  pcl_path.push_back(ap);

  pub_pl_func(pcl_path, pub_curr_path);
}

void ResultOutput::pub_localmap(int mgsize, int cur_session, vector<PVecPtr>& pvec_buf, vector<IMUST>& x_buf,
                                pcl::PointCloud<PointType>& pcl_path, int win_base, int win_count)
{
  pcl::PointCloud<PointType> pcl_loc_map;

  for (int i = 0; i < mgsize; i++)
  {
    for (int j = 0; j < pvec_buf[i]->size(); j += 3)
    {
      pointVar& pv = pvec_buf[i]->at(j);
      Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;

      PointType ap;
      ap.x = pvec[0];
      ap.y = pvec[1];
      ap.z = pvec[2];
      ap.intensity = pv.intensity;
      pcl_loc_map.push_back(ap);
    }
  }

  for (int i = 0; i < win_count; i++)
  {
    Eigen::Vector3d pcurr = x_buf[i].p;
    pcl_path[i + win_base].x = pcurr[0];
    pcl_path[i + win_base].y = pcurr[1];
    pcl_path[i + win_base].z = pcurr[2];
  }

  pub_pl_func(pcl_path, pub_curr_path);
  pub_pl_func(pcl_loc_map, pub_cmap);
}
