/**
 * @file constants.hpp
 * @brief Core constants and macros for VINA-SLAM
 *
 * This file contains fundamental constants, macros, and type definitions
 * that are used throughout the entire system.
 */

#pragma once

// ============================================================================
// Hash Constants
// ============================================================================
#define HASH_P 1000033
#define MAX_N 100000000000

// ============================================================================
// Skew Symmetric Matrix Macro
// ============================================================================
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

// ============================================================================
// Eigen Aligned Container Macros
// ============================================================================
#define PLM(a) \
  std::vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) \
  std::vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

// ============================================================================
// Physical Constants
// ============================================================================
#define G_m_s2 9.8       // Gravity constant (m/s^2)
#define DIM 15           // State dimension (rotation=3, translation=3, velocity=3, gyro_bias=3, accel_bias=3)
#define NMATCH 5         // Number of nearest neighbors for plane fitting

// ============================================================================
// Terminal Color Macros (for debugging output)
// ============================================================================
// Basic colors
#define RESET "\033[0m"
#define BLACK "\033[30m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"
#define WHITE "\033[37m"

// Bold colors
#define BOLDRED "\033[1;31m"
#define BOLDGREEN "\033[1;32m"
#define BOLDYELLOW "\033[1;33m"
#define BOLDBLUE "\033[1;34m"
#define BOLDCYAN "\033[1;36m"

// ============================================================================
// Utility Macros (only if not already defined by PCL)
// ============================================================================
#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#endif

// ============================================================================
// EKF Convergence Thresholds
// ============================================================================
// Conversion factor: radians to degrees (180/PI ≈ 57.2957795)
constexpr double kRadToDeg = 57.295779513082321;

// EKF convergence thresholds for state estimation
constexpr double kRotConvergeThreshDeg = 0.01;     // Rotation convergence (degrees)
constexpr double kTraConvergeThreshCm = 0.015;     // Translation convergence (cm)
constexpr double kMeterToCm = 100.0;               // Meters to centimeters conversion

// Eigenvalue threshold for plane quality assessment
constexpr double kPlaneEigenvalueThresh = 14.0;    // Minimum eigenvalue for valid plane

// ============================================================================
// Memory Management Thresholds
// ============================================================================
constexpr int kMaxOctosBatchDelete = 1000;         // Max octos to delete per batch
constexpr int kSlideWindowMaxSize = 10000;         // Max slide window size before cleanup
constexpr int kSlideWindowBatchDelete = 500;       // Batch deletion size for slide windows
constexpr int kSleepUsNoData = 1000;               // Sleep time (us) when no data available

// ============================================================================
// Voxel Map Thresholds
// ============================================================================
constexpr double kVoxelReleaseDistThresh = 30.0;   // Distance threshold for releasing voxels
constexpr double kVoxelDownsampleSize = 0.5;       // Default voxel downsampling size
constexpr int kMinPointsForDownsample = 2000;      // Min points before using smaller voxel

// ============================================================================
// ROS QoS Settings
// ============================================================================
constexpr int kPclQosDepth = 1000;                 // Point cloud QoS queue depth
