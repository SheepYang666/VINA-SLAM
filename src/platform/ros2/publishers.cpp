/**
 * @file publishers.cpp
 * @brief Implementation of ROS2 publishers
 */

#include "vina_slam/platform/ros2/publishers.hpp"
#include "vina_slam/mapping/keyframe.hpp"
#include "vina_slam/mapping/octree.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace vina_slam {
namespace platform {
namespace ros2 {

ResultPublisher::ResultPublisher(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {
  pub_odom = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
  pub_pmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 10);
  pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/curr_path", 10);
  pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 10);
  pub_pmap_livox = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/pmap_livox", 10);
  pub_path = node->create_publisher<visualization_msgs::msg::Marker>("/path", 10);
  pub_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/plane_markers", 10);
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

ResultPublisher& ResultPublisher::instance(const rclcpp::Node::SharedPtr& node_in) {
  static ResultPublisher inst(node_in);
  return inst;
}

ResultPublisher& ResultPublisher::instance() {
  return instance(rclcpp::Node::SharedPtr());
}

void ResultPublisher::publishOdometry(const core::IMUST& xc) {
  Eigen::Quaterniond q_this(xc.R);
  Eigen::Vector3d t_this = xc.p;

  double stamp_sec = 0.0;
  pcl_time_lock.lock();
  stamp_sec = pcl_end_time;
  pcl_time_lock.unlock();

  rclcpp::Time stamp = (stamp_sec > 0.0) ? rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9)) : node->now();

  // TF broadcast
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = "camera_init";
  transformStamped.child_frame_id = "aft_mapped";

  transformStamped.transform.translation.x = t_this.x();
  transformStamped.transform.translation.y = t_this.y();
  transformStamped.transform.translation.z = t_this.z();
  transformStamped.transform.rotation.x = q_this.x();
  transformStamped.transform.rotation.y = q_this.y();
  transformStamped.transform.rotation.z = q_this.z();
  transformStamped.transform.rotation.w = q_this.w();

  tf_broadcaster->sendTransform(transformStamped);

  // Odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.header.frame_id = transformStamped.header.frame_id;
  odom_msg.child_frame_id = transformStamped.child_frame_id;
  odom_msg.pose.pose.position.x = t_this.x();
  odom_msg.pose.pose.position.y = t_this.y();
  odom_msg.pose.pose.position.z = t_this.z();
  odom_msg.pose.pose.orientation.x = q_this.x();
  odom_msg.pose.pose.orientation.y = q_this.y();
  odom_msg.pose.pose.orientation.z = q_this.z();
  odom_msg.pose.pose.orientation.w = q_this.w();
  pub_odom->publish(odom_msg);
}

void ResultPublisher::publishPointCloud(const pcl::PointCloud<core::PointType>& pl, int publisher_type) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(pl, cloud_msg);
  cloud_msg.header.stamp = node->now();
  cloud_msg.header.frame_id = "camera_init";

  if (publisher_type == 0 && pub_scan) {
    pub_scan->publish(cloud_msg);
  } else if (publisher_type == 1 && pub_pmap) {
    pub_pmap->publish(cloud_msg);
  }
}

void ResultPublisher::publishPointCloudTo(const pcl::PointCloud<core::PointType>& pl,
                                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
  if (!pub) return;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(pl, cloud_msg);
  cloud_msg.header.stamp = node->now();
  cloud_msg.header.frame_id = "camera_init";
  pub->publish(cloud_msg);
}

void ResultPublisher::publishPmapLivox(const pcl::PointCloud<core::PointType>& pl) {
  if (!pub_pmap_livox) return;

  livox_ros_driver2::msg::CustomMsg msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = "camera_init";
  msg.timebase = static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL + msg.header.stamp.nanosec;
  msg.point_num = static_cast<uint32_t>(pl.size());
  msg.lidar_id = 0;
  msg.rsvd[0] = 0;
  msg.rsvd[1] = 0;
  msg.rsvd[2] = 0;

  msg.points.resize(pl.size());
  for (size_t i = 0; i < pl.size(); ++i) {
    const auto& pt = pl[i];
    auto& out = msg.points[i];
    out.x = pt.x;
    out.y = pt.y;
    out.z = pt.z;
    if (pt.intensity <= 0.0f) {
      out.reflectivity = 0;
    } else if (pt.intensity >= 255.0f) {
      out.reflectivity = 255;
    } else {
      out.reflectivity = static_cast<uint8_t>(pt.intensity);
    }
    out.tag = 0;
    out.line = 0;
    out.offset_time = 0;
  }

  pub_pmap_livox->publish(msg);
}

void ResultPublisher::publishLocalTrajectory(PLV(3)& pwld, double jour, const core::IMUST& x_curr,
                                             int cur_session, pcl::PointCloud<core::PointType>& pcl_path) {
  publishOdometry(x_curr);

  pcl::PointCloud<core::PointType> pcl_send;
  pcl_send.reserve(pwld.size());

  for (Eigen::Vector3d& pw : pwld) {
    core::PointType ap;
    ap.x = pw.x();
    ap.y = pw.y();
    ap.z = pw.z();
    pcl_send.push_back(ap);
  }

  publishPointCloud(pcl_send, 0);

  // Add current position to path
  Eigen::Vector3d pcurr = x_curr.p;
  core::PointType ap;
  ap.x = pcurr[0];
  ap.y = pcurr[1];
  ap.z = pcurr[2];
  ap.curvature = jour;
  ap.intensity = cur_session;
  pcl_path.push_back(ap);

  publishPointCloudTo(pcl_path, pub_curr_path);
}

template<typename OctoTreePtr>
void ResultPublisher::publishLocalMap(int /*mgsize*/,
                                      std::unordered_map<core::VOXEL_LOC, OctoTreePtr>& surf_map,
                                      std::vector<core::IMUST>& /*x_buf*/) {
  pcl::PointCloud<core::PointType> pcl_send;

  for (auto& kv : surf_map) {
    auto* octo = kv.second;
    if (octo && octo->plane.is_plane) {
      core::PointType pt;
      pt.x = octo->plane.center.x();
      pt.y = octo->plane.center.y();
      pt.z = octo->plane.center.z();
      pcl_send.push_back(pt);
    }
  }

  publishPointCloud(pcl_send, 0);
}

void ResultPublisher::publishLocalMapFull(int mgsize, int /*cur_session*/, std::vector<core::PVecPtr>& pvec_buf,
                                          std::vector<core::IMUST>& x_buf, pcl::PointCloud<core::PointType>& pcl_path,
                                          int win_base, int win_count) {
  pcl::PointCloud<core::PointType> pcl_loc_map;

  // Transform and accumulate points from buffer
  for (int i = 0; i < mgsize; i++) {
    for (int j = 0; j < static_cast<int>(pvec_buf[i]->size()); j += 1) {
      core::pointVar& pv = pvec_buf[i]->at(j);
      Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;

      core::PointType ap;
      ap.x = pvec[0];
      ap.y = pvec[1];
      ap.z = pvec[2];
      ap.intensity = pv.intensity;
      pcl_loc_map.push_back(ap);
    }
  }

  // Update path positions
  for (int i = 0; i < win_count; i++) {
    Eigen::Vector3d pcurr = x_buf[i].p;
    pcl_path[i + win_base].x = pcurr[0];
    pcl_path[i + win_base].y = pcurr[1];
    pcl_path[i + win_base].z = pcurr[2];
  }

  publishPointCloudTo(pcl_path, pub_curr_path);
  publishPointCloudTo(pcl_loc_map, pub_cmap);
  publishPmapLivox(pcl_loc_map);
}

void ResultPublisher::publishGlobalMap(std::vector<std::vector<mapping::Keyframe*>*>& relc_submaps,
                                       std::vector<int>& ids,
                                       rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
  pcl::PointCloud<core::PointType> pl;
  publishPointCloudTo(pl, pub);  // Initial empty publish

  core::PointType pp;
  uint interval_size = 5e6;

  // Calculate total points and determine downsampling
  uint psize = 0;
  for (int id : ids) {
    std::vector<mapping::Keyframe*>& smps = *(relc_submaps[id]);
    for (int i = 0; i < static_cast<int>(smps.size()); i++) {
      psize += smps[i]->plptr->size();
    }
  }

  int jump = psize / (10 * interval_size) + 1;

  // Publish downsampled global map
  for (int id : ids) {
    pp.intensity = id;

    std::vector<mapping::Keyframe*>& smps = *(relc_submaps[id]);
    for (int i = 0; i < static_cast<int>(smps.size()); i++) {
      core::IMUST xx = smps[i]->x0;

      for (int j = 0; j < static_cast<int>(smps[i]->plptr->size()); j += jump) {
        core::PointType& ap = smps[i]->plptr->points[j];
        Eigen::Vector3d vv(ap.x, ap.y, ap.z);
        vv = xx.R * vv + xx.p;

        pp.x = vv[0];
        pp.y = vv[1];
        pp.z = vv[2];

        pl.push_back(pp);
      }

      // Publish in batches to avoid memory issues
      if (pl.size() > interval_size) {
        publishPointCloudTo(pl, pub);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        pl.clear();
      }
    }
  }

  publishPointCloudTo(pl, pub);
}

// Explicit instantiation
template void ResultPublisher::publishLocalMap<mapping::OctoTree*>(
    int mgsize, std::unordered_map<core::VOXEL_LOC, mapping::OctoTree*>& surf_map,
    std::vector<core::IMUST>& x_buf);

void ResultPublisher::setPclEndTime(double time) {
  std::lock_guard<std::mutex> lock(pcl_time_lock);
  pcl_end_time = time;
}

} // namespace ros2
} // namespace platform
} // namespace vina_slam
