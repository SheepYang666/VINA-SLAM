# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

This is a ROS 2 (Humble) package. Build from the workspace root (`/home/zry/Code/VINA_SLAM`):

```bash
# Standard build
colcon build --packages-select vina_slam

# Debug build with AddressSanitizer
colcon build --packages-select vina_slam --cmake-args -DCMAKE_BUILD_TYPE=Debug -DVINA_SLAM_ENABLE_ASAN=ON

# Run
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vina_slam start.launch.py
```

The launch file hardcodes `config/mid360.yaml`. To use a different sensor config, edit `launch/start.launch.py` or pass parameters directly. Other configs: `HILTI.yaml`, `robosense.yaml`, `outdoor_fly.yaml`, `compus_elevator.yaml`.

## Architecture

### Module Layout

```
src/
  core/         — types, math, point utilities, sensor context
  sensor/       — LiDAR decoder (Livox CustomMsg → PointCloud)
  estimation/   — ImuEkf (propagation + motion blur), ImuPreintegration, StateEstimator, SlidingWindowManager
  mapping/       — OctoTree, VoxelMap, LidarFactor, NormalFactor, optimizers
  pipeline/      — Initialization, Odometry, LocalMapping
  platform/ros2/ — Node, Publishers, Subscribers, IO (ROS2 isolation layer)
  VINASlam.cpp   — Main SLAM class (legacy monolith, being refactored)
  main.cpp       — Entry point: creates node, launches odometry thread
```

### Data Flow

1. `platform/ros2/subscribers` receives Livox CustomMsg + IMU → buffers in `imu_buf` / `pcl_buf`
2. `VINASlam::RunOdometryLocalMapping` (runs in its own thread) calls `sync_packages` to pair LiDAR+IMU
3. `estimation/ImuEkf` propagates state and corrects motion blur on the point cloud
4. `LioStateEstimation` / `VNCLio` runs IEKF with point-to-plane constraints against `surf_map`
5. `MultiRecut` rebuilds the octree planes; `MultiMarginalize` marginalizes old states
6. `platform/ros2/publishers` (ResultPublisher singleton) publishes TF, path, map topics

### Key Types (`include/vina_slam/core/types.hpp`)

- `IMUST` — 15-DOF state: R (SO3), p, v, bg, ba, g + covariance. The central currency between all modules.
- `pointVar` / `PVec` / `PVecPtr` — point with 3×3 covariance, used throughout estimation and mapping.
- `PointCluster` — accumulates second-moment statistics (P, v, N) for efficient plane fitting without storing raw points.
- `VOXEL_LOC` — integer 3D key for the `unordered_map<VOXEL_LOC, OctoTree*>` voxel hash map.

### Voxel Map & OctoTree (`mapping/`)

The global map is `unordered_map<VOXEL_LOC, OctoTree*> surf_map`. Each `OctoTree` node:
- Subdivides space up to `max_layer` levels (default 3)
- Stores a `SlideWindow` of points per frame for the current optimization window
- Fits a `Plane` using eigendecomposition of `PointCluster::cov()`; `min_eig_value` is the planarity metric
- `cutVoxel` / `cutVoxelMulti` insert new scan points; `matchVoxelMap` finds the nearest plane for a query point

### Estimation Pipeline

- **IEKF** (`LioStateEstimation`): iterates point-to-plane residuals to converge state. Convergence thresholds in `constants.hpp` (`kRotConvergeThreshDeg`, `kTraConvergeThreshCm`).
- **VNCLio**: extends IEKF with VNC rotation constraints — extracts scan plane normals in body frame, matches to map normals via 27-neighbor `matchVoxelMap` search with normal consistency filtering (dot > 0.7), adds 3D vector residuals `r = S * n_scan_world` (rank-2 per pair) that constrain rotation in planar regions.
- **LocalBA** (`MultiRecut` + `MultiMarginalize`): sliding-window bundle adjustment using `LidarFactor` (point-to-plane) and `NormalFactor` (normal consistency). Window size set by `LocalBA.win_size` in config.

### Namespace Transition

Code is being migrated from global scope to `vina_slam::core`, `vina_slam::mapping`, `vina_slam::estimation`, `vina_slam::pipeline`, `vina_slam::platform::ros2`. Every new header provides backward-compat `using` aliases at the bottom. Prefer the namespaced API for new code; the global aliases exist only for `VINASlam.cpp` compatibility.

### Singletons

Three singletons initialized in `main.cpp`:
- `vina_slam::platform::ros2::ResultPublisher` — all outbound ROS publishers
- `vina_slam::platform::ros2::FileReaderWriter` — map save/load
- `vina_slam::pipeline::Initialization` — system initialization logic

### ROS Topics

| Topic | Type | Description |
|---|---|---|
| `/curr_path` | PointCloud2 | Full accumulated trajectory snapshot |
| `/cloud_registered` | PointCloud2 | Registered scan |
| `/global_map` | PointCloud2 | Global map |
| `/cloud_map` | PointCloud2 | Local map |
| TF `camera_init → aft_mapped` | TF | Live pose |

## Config Parameters

Key parameters in `config/*.yaml` under `vina_slam.ros__parameters`:

- `General.lidar_type`: 0 = Mid360, 5 = sim
- `General.extrinsic_tran/rota`: LiDAR-to-IMU extrinsics
- `Odometry.voxel_size`: voxel edge length for odometry map (default 1.0 m)
- `Odometry.min_eigen_value`: planarity threshold (default 0.0025)
- `LocalBA.win_size`: sliding window size (default 10)
- `LocalBA.plane_eigen_value_thre`: per-layer plane quality thresholds
- `General.is_save_map`: 1 to save map on exit to `General.save_path/bagname`
