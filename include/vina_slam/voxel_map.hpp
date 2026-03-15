/**
 * @file voxel_map.hpp
 * @brief Legacy compatibility header - aliases for modular mapping types
 *
 * This header brings all modular mapping types into the global namespace
 * for backward compatibility with legacy code. New code should include
 * the specific modular headers directly.
 */

#pragma once

// Modular headers - provide all implementations
#include "vina_slam/core/common.hpp"
#include "vina_slam/estimation/imu_preintegration.hpp"
#include "vina_slam/mapping/factors.hpp"
#include "vina_slam/mapping/optimizers.hpp"
#include "vina_slam/mapping/octree.hpp"
#include "vina_slam/mapping/voxel_map.hpp"
#include "vina_slam/mapping/keyframe.hpp"
#include "vina_slam/mapping/slide_window.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

// ============================================================================
// Bring frequently used STL types to global scope (legacy code compatibility)
// ============================================================================
using std::deque;
using std::mutex;
using std::pair;
using std::make_pair;
using std::ref;
using std::thread;
using std::unordered_map;
using std::unordered_set;
using std::vector;

// ============================================================================
// Core type aliases (global namespace)
// ============================================================================
using pointVar = vina_slam::core::pointVar;
using PVec = vina_slam::core::PVec;
using PVecPtr = vina_slam::core::PVecPtr;
using Plane = vina_slam::core::Plane;
using IMUST = vina_slam::core::IMUST;
using PointType = vina_slam::core::PointType;
using PointCluster = vina_slam::core::PointCluster;
using VOXEL_LOC = vina_slam::core::VOXEL_LOC;

// ============================================================================
// Mapping class aliases (global namespace)
// ============================================================================
using LidarFactor = vina_slam::mapping::LidarFactor;
using NormalFactor = vina_slam::mapping::NormalFactor;
using OctoTree = vina_slam::mapping::OctoTree;
using SlideWindow = vina_slam::mapping::SlideWindow;
using Keyframe = vina_slam::mapping::Keyframe;
// Optimizer aliases already provided by mapping/optimizers.hpp:
//   Lidar_BA_Optimizer, LI_BA_Optimizer, LI_BA_OptimizerGravity

// ============================================================================
// Global variable access (forwarded from vina_slam::mapping namespace)
// ============================================================================
using vina_slam::mapping::min_point;
using vina_slam::mapping::min_eigen_value;
using vina_slam::mapping::max_layer;
using vina_slam::mapping::max_points;
using vina_slam::mapping::voxel_size;
using vina_slam::mapping::min_ba_point;
using vina_slam::mapping::plane_eigen_value_thre;
using vina_slam::mapping::mp;
using vina_slam::mapping::imu_coef;

// ============================================================================
// Free function aliases
// ============================================================================
using vina_slam::mapping::bfVar;

/// Backward compatibility wrapper for Bf_var -> bfVar
inline void Bf_var(const vina_slam::core::pointVar& pv,
                   Eigen::Matrix<double, 9, 9>& bcov,
                   const Eigen::Vector3d& vec) {
  vina_slam::mapping::bfVar(pv, bcov, vec);
}

// Voxel map operation aliases (from mapping/voxel_map.hpp)
using vina_slam::mapping::cut_voxel;
using vina_slam::mapping::cut_voxel_multi;
using vina_slam::mapping::generate_voxel;
using vina_slam::mapping::match;
using vina_slam::mapping::down_sampling_pvec;
