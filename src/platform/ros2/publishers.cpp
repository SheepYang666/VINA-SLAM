#include "vina_slam/platform/ros2/publishers.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Global publisher definitions (moved from VINASlam.hpp)
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_plane;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_normal;

// ResultOutput static members
std::unique_ptr<ResultOutput> ResultOutput::inst_ = nullptr;
bool ResultOutput::initialized_ = false;

ResultOutput::ResultOutput(const rclcpp::Node::SharedPtr& node_in) : node(node_in)
{
}

ResultOutput& ResultOutput::instance(const rclcpp::Node::SharedPtr& node_in)
{
  if (!inst_)
  {
    inst_.reset(new ResultOutput(node_in));
    initialized_ = true;
  }
  return *inst_;
}

ResultOutput& ResultOutput::instance()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("vina_slam"), "ResultOutput::instance() called without node initialization!");
    throw std::runtime_error("ResultOutput not initialized with node");
  }
  return *inst_;
}

void ResultOutput::pub_odom_func(IMUST& xc)
{
  Eigen::Quaterniond q_this(xc.R);
  Eigen::Vector3d t_this = xc.p;
  static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  br = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = node->now();
  transformStamped.header.frame_id = "camera_init";
  transformStamped.child_frame_id = "aft_mapped";

  transformStamped.transform.translation.x = t_this.x();
  transformStamped.transform.translation.y = t_this.y();
  transformStamped.transform.translation.z = t_this.z();
  transformStamped.transform.rotation.x = q_this.x();
  transformStamped.transform.rotation.y = q_this.y();
  transformStamped.transform.rotation.z = q_this.z();
  transformStamped.transform.rotation.w = q_this.w();

  br->sendTransform(transformStamped);
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

  pub_pl_func(pcl_send, pub_scan, node);

  Eigen::Vector3d pcurr = x_curr.p;

  PointType ap;
  ap.x = pcurr[0];
  ap.y = pcurr[1];
  ap.z = pcurr[2];
  ap.curvature = jour;
  ap.intensity =
      cur_session;         // Use intensity to represent the current session number (such as different track segments)
  pcl_path.push_back(ap);  // Add to the trajectory point cloud

  pub_pl_func(pcl_path, pub_curr_path, node);
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
      Eigen::Vector3d pvec =
          x_buf[i].R * pv.pnt + x_buf[i].p;

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

  pub_pl_func(pcl_path, pub_curr_path, node);
  pub_pl_func(pcl_loc_map, pub_cmap, node);
}
