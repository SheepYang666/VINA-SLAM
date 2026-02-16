/**
 * @file sensor_context.cpp
 * @brief Implementation of SensorContext singleton
 */

#include "vina_slam/core/sensor_context.hpp"
#include "vina_slam/core/constants.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace vina_slam {
namespace core {

SensorContext& SensorContext::Instance() {
    static SensorContext instance;
    return instance;
}

void SensorContext::Initialize(const rclcpp::Node::SharedPtr& node) {
    if (initialized_) {
        return;
    }
    node_ = node;

    // Initialize publishers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile));

    pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", qos);
    pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", qos);
    pub_init = node->create_publisher<sensor_msgs::msg::PointCloud2>("/init_map", qos);
    pub_pmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/prev_map", qos);
    pub_test = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test_map", qos);
    pub_prev_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/prev_path", qos);
    pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/curr_path", qos);
    pub_pmap_livox = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/pmap_livox", qos);
    pub_voxel_plane = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_plane", qos);
    pub_voxel_normal = node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_normal", qos);

    // Load point cloud decoder parameters
    feat.blind = node->declare_parameter("General.blind", 0.1);
    feat.point_filter_num = node->declare_parameter("General.point_filter_num", 1);

    // Load error parameters
    dept_err = node->declare_parameter("Odometry.dept_err", 0.02);
    beam_err = node->declare_parameter("Odometry.beam_err", 0.05);

    initialized_ = true;
}

}  // namespace core
}  // namespace vina_slam

// ============================================================================
// Backward Compatibility: Global Variable Definitions
// ============================================================================
// These global variables redirect to the SensorContext singleton.
// This allows existing code to continue working while gradually
// migrating to the new SensorContext API.

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_init;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pmap;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_test;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prev_path;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;
rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_pmap_livox;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_plane;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_normal;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl;
std::mutex mBuf;
LidarPointCloudDecoder feat;
std::deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
std::deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
std::deque<double> time_buf;
double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;
std::mutex pcl_time_lock;
double pcl_time = 0;
double pcl_end_time = 0;
double dept_err = 0;
double beam_err = 0;
