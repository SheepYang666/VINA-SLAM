/**
 * @file sensor_context.hpp
 * @brief Encapsulated sensor data buffers and ROS interface
 *
 * This class replaces the global variables previously used for
 * sensor data management and ROS communication.
 */

#pragma once

#include "vina_slam/sensor/lidar_decoder.hpp"
#include "vina_slam/core/types.hpp"  // PointType, IMUST

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <deque>
#include <memory>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vina_slam {
namespace core {

/**
 * @brief Singleton class managing sensor buffers and ROS interface
 *
 * This class encapsulates what were previously global variables:
 * - Publishers (pub_scan, pub_cmap, etc.)
 * - Subscribers (sub_imu, sub_pcl)
 * - Data buffers (imu_buf, pcl_buf, time_buf)
 * - Mutexes (mBuf, pcl_time_lock)
 * - Time tracking (imu_last_time, pcl_time, etc.)
 * - Point cloud decoder (feat)
 */
class SensorContext {
public:
    // Delete copy/move constructors
    SensorContext(const SensorContext&) = delete;
    SensorContext& operator=(const SensorContext&) = delete;

    /**
     * @brief Get singleton instance
     */
    static SensorContext& Instance();

    /**
     * @brief Initialize publishers and subscribers
     * @param node ROS2 node
     */
    void Initialize(const rclcpp::Node::SharedPtr& node);

    // =========================================================================
    // Publishers
    // =========================================================================
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

    // =========================================================================
    // Subscribers
    // =========================================================================
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl;

    // =========================================================================
    // Data Buffers (protected by mBuf)
    // =========================================================================
    std::mutex mBuf;
    std::deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
    std::deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
    std::deque<double> time_buf;

    // =========================================================================
    // Time Tracking
    // =========================================================================
    double imu_last_time = -1.0;
    double last_pcl_time = -1.0;
    int point_notime = 0;

    // =========================================================================
    // PCL Time (protected by pcl_time_lock)
    // =========================================================================
    std::mutex pcl_time_lock;
    double pcl_time = 0.0;
    double pcl_end_time = 0.0;

    // =========================================================================
    // Point Cloud Decoder
    // =========================================================================
    LidarPointCloudDecoder feat;

    // =========================================================================
    // Error Parameters
    // =========================================================================
    double dept_err = 0.0;
    double beam_err = 0.0;

    // =========================================================================
    // Utility Functions
    // =========================================================================

    /**
     * @brief Publish point cloud
     */
    template <typename CloudT>
    void PublishPointCloud(CloudT& cloud,
                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                          const rclcpp::Node::SharedPtr& node);

    /**
     * @brief Handle incoming point cloud message
     */
    template <class T>
    void HandlePointCloud(T& msg);

private:
    SensorContext() = default;
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
};

// Template implementation
template <typename CloudT>
void SensorContext::PublishPointCloud(CloudT& cloud,
                                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                                      const rclcpp::Node::SharedPtr& node) {
    cloud.height = 1;
    cloud.width = cloud.size();
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "camera_init";
    output.header.stamp = node->now();
    pub->publish(output);
}

// Global access function for backward compatibility
// These will be deprecated in future versions
inline SensorContext& GetSensorContext() {
    return SensorContext::Instance();
}

}  // namespace core
}  // namespace vina_slam

// Backward compatibility: global variable references
// These are deprecated and will be removed in future versions
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cmap;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_init;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pmap;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_test;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prev_path;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_curr_path;
extern rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_pmap_livox;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_plane;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_voxel_normal;
extern rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
extern rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl;
extern std::mutex mBuf;
extern LidarPointCloudDecoder feat;
extern std::deque<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buf;
extern std::deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
extern std::deque<double> time_buf;
extern double imu_last_time;
extern int point_notime;
extern double last_pcl_time;
extern std::mutex pcl_time_lock;
extern double pcl_time;
extern double pcl_end_time;
extern double dept_err;
extern double beam_err;
