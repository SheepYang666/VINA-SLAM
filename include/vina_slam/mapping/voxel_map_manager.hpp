/**
 * @file voxel_map_manager.hpp
 * @brief Voxel map management for VINA-SLAM
 *
 * Handles:
 * - Main voxel map (surf_map)
 * - Sliding voxel map (surf_map_slide)
 * - OctoTree release management
 * - Voxel cleanup and memory management
 */

#pragma once

#include "vina_slam/core/types.hpp"
#include "vina_slam/core/constants.hpp"
#include "vina_slam/voxel_map.hpp"
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace vina_slam {
namespace mapping {

/**
 * @brief Voxel map manager for SLAM optimization
 *
 * Manages voxel maps and OctoTree lifecycle:
 * - Main surface map for tracking
 * - Sliding surface map for optimization
 * - OctoTree release queue for memory management
 */
class VoxelMapManager {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// ROS node pointer
  rclcpp::Node::SharedPtr node;

  /// Main surface voxel map
  std::unordered_map<core::VOXEL_LOC, OctoTree*> surf_map;

  /// Sliding surface voxel map
  std::unordered_map<core::VOXEL_LOC, OctoTree*> surf_map_slide;

  /// Queue of OctoTrees to be released
  std::vector<OctoTree*> octos_release;

  /**
   * @brief Default constructor
   */
  VoxelMapManager() = default;

  /**
   * @brief Constructor with ROS node
   * @param node_in ROS node pointer
   */
  explicit VoxelMapManager(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance with node initialization
   * @param node_in ROS node pointer
   * @return Reference to singleton instance
   */
  static VoxelMapManager& instance(const rclcpp::Node::SharedPtr& node_in);

  /**
   * @brief Get singleton instance (must be initialized first)
   * @return Reference to singleton instance
   */
  static VoxelMapManager& instance();

  /**
   * @brief Destructor - cleanup all OctoTrees
   */
  ~VoxelMapManager();

  /**
   * @brief Release old voxels based on journey distance
   * @param jour Current journey distance
   *
   * Removes voxels that are too far from current position.
   */
  void releaseOldVoxels(double jour);

  /**
   * @brief Cleanup released OctoTree nodes
   *
   * Deletes OctoTrees in the release queue in batches.
   */
  void cleanupReleasedOctos();

  /**
   * @brief Add OctoTree to release queue
   * @param octo OctoTree to release
   */
  void addToReleaseQueue(OctoTree* octo);

  /**
   * @brief Add multiple OctoTrees to release queue
   * @param octos Vector of OctoTrees to release
   */
  void addToReleaseQueue(std::vector<OctoTree*>& octos);

  /**
   * @brief Clear main voxel map
   *
   * Moves all OctoTrees to release queue and clears the map.
   */
  void clearMainMap();

  /**
   * @brief Clear sliding voxel map
   *
   * Moves all OctoTrees to release queue and clears the map.
   */
  void clearSlidingMap();

  /**
   * @brief Clear all maps
   */
  void clearAllMaps();

  /**
   * @brief Check if release queue is empty
   * @return true if no OctoTrees pending release
   */
  bool isReleaseQueueEmpty() const { return octos_release.empty(); }

  /**
   * @brief Get main map size
   * @return Number of voxels in main map
   */
  size_t mainMapSize() const { return surf_map.size(); }

  /**
   * @brief Get sliding map size
   * @return Number of voxels in sliding map
   */
  size_t slidingMapSize() const { return surf_map_slide.size(); }

  // ========================================================================
  // Backward compatibility methods (snake_case aliases)
  // ========================================================================
  void release_old_voxels(double jour) { releaseOldVoxels(jour); }
  void cleanup_released_octos() { cleanupReleasedOctos(); }
  void add_to_release_queue(OctoTree* octo) { addToReleaseQueue(octo); }
  bool is_release_queue_empty() const { return isReleaseQueueEmpty(); }
  size_t main_map_size() const { return mainMapSize(); }
  size_t sliding_map_size() const { return slidingMapSize(); }
};

} // namespace mapping
} // namespace vina_slam

// Backward compatibility
using vina_slam::mapping::VoxelMapManager;
