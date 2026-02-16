/**
 * @file io.hpp
 * @brief File I/O utilities for VINA-SLAM
 *
 * Handles:
 * - PCD file reading/writing
 * - Pose file reading/writing
 * - Map save/load
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace vina_slam {
namespace platform {
namespace ros2 {

/**
 * @brief File I/O utilities
 */
class FileIO {
public:
  /**
   * @brief Save point cloud to PCD file
   * @param path File path
   * @param cloud Point cloud to save
   * @return true if successful
   */
  static bool savePcd(const std::string& path, const pcl::PointCloud<core::PointType>& cloud);

  /**
   * @brief Load point cloud from PCD file
   * @param path File path
   * @param cloud Output point cloud
   * @return true if successful
   */
  static bool loadPcd(const std::string& path, pcl::PointCloud<core::PointType>& cloud);

  /**
   * @brief Save poses to file (timestamp, x, y, z, qx, qy, qz, qw)
   * @param path File path
   * @param poses Pose vector
   * @return true if successful
   */
  static bool savePoses(const std::string& path, const std::vector<core::IMUST>& poses);

  /**
   * @brief Load poses from file
   * @param path File path
   * @param poses Output pose vector
   * @return true if successful
   */
  static bool loadPoses(const std::string& path, std::vector<core::IMUST>& poses);

  /**
   * @brief Save trajectory to TUM format
   * @param path File path
   * @param poses Pose vector
   * @return true if successful
   */
  static bool saveTumTrajectory(const std::string& path, const std::vector<core::IMUST>& poses);

  /**
   * @brief Save trajectory to KITTI format
   * @param path File path
   * @param poses Pose vector
   * @return true if successful
   */
  static bool saveKitTiTrajectory(const std::string& path, const std::vector<core::IMUST>& poses);

  /**
   * @brief Check if file exists
   * @param path File path
   * @return true if file exists
   */
  static bool fileExists(const std::string& path);

  /**
   * @brief Create directory if not exists
   * @param path Directory path
   * @return true if successful
   */
  static bool createDirectory(const std::string& path);
};

/**
 * @brief File reader/writer for VINA-SLAM data
 *
 * Handles map and trajectory file operations.
 */
class FileReaderWriter {
public:
  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// Save path
  std::string save_path;

  /// Whether to save map
  int is_save_map = 0;

  /**
   * @brief Default constructor
   */
  FileReaderWriter() = default;

  /**
   * @brief Constructor with node
   * @param node_in ROS node pointer
   */
  explicit FileReaderWriter(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance
   * @param node_in ROS node pointer
   */
  static FileReaderWriter& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton (must be initialized first)
   */
  static FileReaderWriter& instance();

  /**
   * @brief Set save path
   * @param path Save directory path
   */
  void setSavePath(const std::string& path) { save_path = path; }

  /**
   * @brief Save point cloud map
   * @param pl Point cloud to save
   */
  void savePcdMap(const pcl::PointCloud<core::PointType>& pl);

  /**
   * @brief Save point cloud with pose to numbered PCD file
   * @param pptr Point vector with variance
   * @param xx Pose (unused, kept for compatibility)
   * @param count File number
   * @param savename Directory path
   */
  void savePcd(core::PVecPtr pptr, core::IMUST& xx, int count, const std::string& savename);

  /**
   * @brief Clear text file contents
   * @param filePath Path to file
   */
  void clearTxtFile(const std::string& filePath);

  /**
   * @brief Load point cloud map
   * @param path Map file path
   * @param pl Output point cloud
   * @return true if successful
   */
  bool loadPcdMap(const std::string& path, pcl::PointCloud<core::PointType>& pl);

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void SavePcd(core::PVecPtr pptr, core::IMUST& xx, int count, const std::string& savename) {
    savePcd(pptr, xx, count, savename);
  }
  void ClearTxtFile(const std::string& filePath) { clearTxtFile(filePath); }
};

} // namespace ros2
} // namespace platform
} // namespace vina_slam
