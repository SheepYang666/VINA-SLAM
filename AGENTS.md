# AGENTS.md - VINA-SLAM

## Project Overview

VINA-SLAM (Voxel-based Inertial and Normal-Aligned LiDAR-IMU SLAM) is a ROS 2 Humble
C++17 package built with colcon/ament_cmake. Single package workspace: the colcon
workspace root is `/` (this directory) and the package source lives in `src/VINA-SLAM/`.

Key dependencies: rclcpp, Eigen3, PCL, tf2_ros, livox_ros_driver2.

## Build Commands

```bash
# Source ROS 2 environment first
source /opt/ros/humble/setup.bash

# Full build (Release, default)
colcon build --packages-select vina_slam

# Debug build with AddressSanitizer
colcon build --packages-select vina_slam --cmake-args -DCMAKE_BUILD_TYPE=Debug -DVINA_SLAM_ENABLE_ASAN=ON

# Build with compile_commands.json (already ON by default)
colcon build --packages-select vina_slam --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Rebuild from scratch
rm -rf build/ install/ log/ && colcon build --packages-select vina_slam
```

## Testing

```bash
# Run ament lint tests (the only tests currently configured)
colcon test --packages-select vina_slam
colcon test-result --verbose

# There are NO unit tests (GTest, etc.) in this project.
# The BUILD_TESTING section only runs ament_lint_auto checks.
```

## Running

```bash
source install/setup.bash
ros2 launch vina_slam start.launch.py
```

Config files are in `src/VINA-SLAM/config/` (YAML). Sensor profiles: mid360, robosense,
HILTI, outdoor_fly, compus_elevator.

## Project Structure

```
src/VINA-SLAM/
  CMakeLists.txt, package.xml       # Build config
  include/vina_slam/                # Public headers (.hpp)
    core/          # types, math, constants, common, sensor_context, config, point_utils
    estimation/    # imu_ekf, imu_preintegration, state_estimator, sliding_window_manager
    mapping/       # octree, factors, optimizers, voxel_map, voxel_map_manager, plane,
                   # slide_window, keyframe
    pipeline/      # initialization, odometry, local_mapping
    platform/ros2/ # node, publishers, subscribers, io
    sensor/        # lidar_decoder, imu_buffer, sync
    VINASlam.hpp   # SLAM pipeline class header
    voxel_map.hpp  # Legacy compatibility aliases (pure using-declarations, no code)
  src/                              # Implementation (.cpp), mirrors include/ layout
    main.cpp                        # Entry point (node creation, singleton init)
    VINASlam.cpp                    # SLAM pipeline (odometry, BA, sliding window)
    core/                           # types, sensor_context, point_utils
    estimation/                     # imu_ekf, imu_preintegration
    mapping/                        # octree, factors, optimizers, voxel_map, voxel_map_manager
    pipeline/                       # initialization
    platform/ros2/                  # publishers, subscribers, io
    sensor/                         # lidar_decoder
  config/                           # YAML sensor/algorithm configs
  launch/                           # ROS 2 launch files (Python)
```

### Key Architecture Notes

- **No dual implementations.** Legacy `src/voxel_map.cpp` has been removed. All mapping
  code lives in `src/mapping/*.cpp` under `vina_slam::mapping` namespace.
- **`voxel_map.hpp` is a pure alias header.** It includes the modular `mapping/*.hpp`
  headers and provides `using` declarations to keep global-namespace names working.
- **`main.cpp` is the composition root.** It creates the ROS2 node, initializes
  publishers and singletons, then launches the SLAM thread.
- **`VINASlam.cpp` contains the pipeline logic** — IEKF state estimation, bundle
  adjustment orchestration, sliding window management, and sensor callbacks.
- **`LioStateEstimation(pptr, use_vnc)`** is the unified IEKF method. When
  `use_vnc=true`, it adds VNC (Vector Normal Consistency) rotation constraints.
  `VNCLio()` is a thin wrapper that calls `LioStateEstimation(pptr, true)`.

## Code Style Guidelines

### Language Standard
- **C++17** (`cxx_std_17`). Use C++17 features freely (structured bindings, `std::optional`,
  `if constexpr`, `std::filesystem`, etc.).

### Naming Conventions (Target Style for New Code)
The codebase is migrating from snake_case to a new convention. **All new code must follow:**

| Element             | Convention        | Example                            |
|---------------------|-------------------|------------------------------------|
| Classes / Structs   | PascalCase        | `VoxelMapManager`, `ImuEkf`        |
| Methods             | camelCase         | `pushVoxel()`, `dampingIter()`     |
| Free functions      | camelCase         | `downSamplingVoxel()`              |
| Constants           | kPascalCase       | `kRotConvergeThreshDeg`            |
| Member variables    | snake_case + `_`  | `config_`, `current_state_`        |
| Local variables     | snake_case        | `voxel_size`, `point_count`        |
| Macros              | UPPER_SNAKE_CASE  | `SKEW_SYM_MATRX`, `DIM`           |
| Namespaces          | snake_case        | `vina_slam::mapping`               |
| Template params     | PascalCase        | `typename ValueType`               |

When touching legacy code (snake_case names like `push_voxel`, `VINA_SLAM`), provide
backward-compatible aliases:
```cpp
void pushVoxel(...);                                  // New API
void push_voxel(...) { pushVoxel(...); }              // Backward compat wrapper
using Lidar_BA_Optimizer = LidarBAOptimizer;          // Type alias
```

### Namespaces
All new code goes in hierarchical namespaces matching directory structure:
```
vina_slam::core, vina_slam::sensor, vina_slam::estimation,
vina_slam::mapping, vina_slam::pipeline, vina_slam::platform::ros2
```

### Include Guards
Use `#pragma once` (no `#ifndef` guards).

### Include Ordering
1. Own header (in `.cpp` files)
2. Project headers (`#include "vina_slam/..."`)
3. Third-party headers (Eigen, PCL, ROS 2)
4. Standard library headers (`<vector>`, `<memory>`, etc.)

### Header Documentation
All refactored headers use Doxygen-style comments:
```cpp
/**
 * @file optimizers.hpp
 * @brief Bundle adjustment optimizers for LiDAR-Inertial SLAM
 */

/**
 * @brief Run Levenberg-Marquardt damping iteration
 * @param x_stats State estimates (modified in place)
 * @param max_iter Maximum iterations
 * @return true if converged
 */
```

### Error Handling
- **No exceptions.** The codebase avoids `throw`. Use:
  - Return codes (0 = initializing, 1 = success)
  - Boolean success/failure returns
  - Validity flags in structs (`bool success`, `bool is_plane`)
- **No `exit()` calls.** Use RCLCPP_ERROR + return error codes instead.
- One narrow `try-catch` exists around KdTree operations; do not expand exception usage.

### Logging
- **Use ROS 2 logging** everywhere: `RCLCPP_INFO()`, `RCLCPP_WARN()`, `RCLCPP_ERROR()`
- Avoid `printf()` / `std::cout` in new code. Existing `printf` in hot inner loops
  is acceptable for performance reasons.

### Memory Management
- Raw `new`/`delete` is used for performance-critical structures (`OctoTree*`,
  `SlideWindow*`). Follow existing patterns for these types.
- Use `std::shared_ptr` for ROS messages.
- All structs containing Eigen types must include `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`.
- Use `PLM(n)` / `PLV(n)` macros for Eigen-aligned `std::vector` containers.

### Thread Management
- **Use `std::vector<std::thread>` with `emplace_back`.** Never use `new thread`.
- Pattern: main thread handles partition 0, workers handle 1..N, then `join()`:
  ```cpp
  std::vector<std::thread> threads;
  threads.reserve(thd_num - 1);
  for (int i = 1; i < thd_num; i++)
    threads.emplace_back(func, args...);
  func(partition_0_args...);            // main thread
  for (auto& t : threads) t.join();
  ```

### Filesystem Operations
- Use `std::filesystem` (C++17), never `system("mkdir ...")`.

### Singleton Pattern
Major subsystems use singletons with `static instance()` methods:
`VinaSlamNode::instance()`, `VoxelMapManager::instance()`,
`SlidingWindowManager::instance()`, `Initialization::instance()`, etc.
Follow this pattern for new top-level managers.

### Compiler Warnings
Build defaults to `-Wall -Wextra -Wpedantic`. New code should compile warning-free.
Controlled by `VINA_SLAM_ENABLE_WARNINGS` (ON by default).

### Formatting
No `.clang-format` file exists. Follow the existing style observed in refactored headers:
- 2-space indentation (observed in headers and CMakeLists.txt)
- Opening brace on same line for functions and control flow
- Max line length ~100 characters
- Section separators: `// ====...====` comment blocks

### Key Macros and Constants (defined in `core/constants.hpp`)
- `DIM` = 15 (state dimension), `NMATCH` = 5, `G_m_s2` = 9.8
- `PLM(n)` / `PLV(n)` -- Eigen-aligned vector macros
- `SKEW_SYM_MATRX(v)` -- skew symmetric matrix elements
- `DEG2RAD()` / `RAD2DEG()` -- angle conversions
- `kPascalCase` constants for thresholds, sizes, QoS depths

### Configuration
System configuration is defined in `core/config.hpp` as structured types:
- `VoxelMapConfig` — voxel size, octree depth, plane thresholds
- `ImuNoiseConfig` — gyro/accel noise, random walk, IMU coefficient
- `SensorErrorConfig` — range/beam error parameters
- `LidarConfig` — LiDAR type, blind zone, filter settings
- `PipelineConfig` — window size, BA toggle, thread count
- `SystemConfig` — aggregates all above

Currently, parameters are still read via `node->declare_parameter()` in the
`VINA_SLAM` constructor. Future work: migrate to populate `SystemConfig` in
`VinaSlamNode::loadConfig()` and inject into modules.

### Adding New Source Files
1. Create header in `include/vina_slam/<module>/` and source in `src/<module>/`
2. Add the `.cpp` to the appropriate `*_SOURCES` list in `CMakeLists.txt`
3. Place code in the matching `vina_slam::<module>` namespace
4. Provide backward-compatible aliases if replacing existing global-scope types
