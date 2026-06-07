# MID360 PointCloud2 Decoder

## Feature Request
- Add support for Livox MID360 `sensor_msgs/PointCloud2` packets with `x`, `y`, `z`, `intensity`, `tag`, and `timestamp` fields.
- Keep the existing Livox `CustomMsg` decoder compatible.
- Apply the change to both `main` and `ros1/noetic`.

## Current Context
- `ros1/noetic` uses `livox_ros_driver::CustomMsg` when `General/lidar_type == 0`.
- `main` uses `livox_ros_driver2::msg::CustomMsg` when `General.lidar_type == 0`.
- All other LiDAR types use the standard `PointCloud2` subscription path.
- `blind` is read from config as meters and squared during node initialization, so decoder comparisons use squared distance.

## Constraints
- Do not repurpose `LIVOX=0`; existing CustomMsg configurations must continue to work.
- MID360 PointCloud2 relative point time must be stored in `PointType::curvature` in seconds.
- Livox tags are filtered with `(tag & 0x30) == 0x00 || (tag & 0x30) == 0x10`.
- Avoid parallel reads from partially initialized output buffers.

## Confirmed Decisions
- Add a new enum value for Livox MID360 PointCloud2 instead of changing the meaning of `LIVOX`.
- Update `config/mid360.yaml` to use the new PointCloud2 type.
- Keep `config/outdoor_fly.yaml` and `config/compus_elevator.yaml` on `LIVOX=0` because they may rely on `CustomMsg`.

## Open Questions
- None blocking. If a deployment still publishes `CustomMsg` on `/livox/lidar`, use a config with `lidar_type: 0`.

## Implementation Plan
1. Add a decoder test that constructs MID360 PointCloud2 data and verifies tag filtering, duplicate filtering, intensity copy, and nanosecond timestamp conversion.
2. Register the MID360 PointCloud2 PCL point struct in the decoder header.
3. Add a MID360 PointCloud2 handler in the standard PointCloud2 switch path.
4. Update `mid360.yaml` and enum comments in config files.
5. Apply equivalent ROS2 branch changes in `main`.
6. Build or run the closest available verification for each branch.

## Known Pitfalls Checklist
- Parallel output buffer hazard: the provided snippet writes `pl_surf[i]` in parallel and compares `pl_surf[i - 1]`, which can read an unwritten point. Guardrail: compare against `pl_orig.points[i - 1]` in a simple loop. Verification: decoder test includes a duplicate point.
- Config ambiguity: `lidar_type: 0` already means Livox CustomMsg. Guardrail: add a new enum value and update comments. Verification: `mid360.yaml` uses the new value while other Livox configs remain unchanged.
- Time unit confusion: MID360 PointCloud2 `timestamp` is nanoseconds. Guardrail: multiply the relative timestamp by `1e-9`. Verification: decoder test checks `0.02` seconds from a `20,000,000 ns` delta.

## Status
- Implemented on `ros1/noetic`.
- Implemented and verified on `main` in `/home/zry/Code/VINA_SLAM/src/VINA-SLAM-main-worktree`.

## Verification
- `ros1/noetic`: `git diff --check` passed. Build could not run because this machine only has `/opt/ros/humble`; `/opt/ros/noetic` and `catkin` are not installed.
- `main`: `cmake --build build --target lidar_mid360_pointcloud_decoder_test vina_slam` passed.
- `main`: `./build/lidar_mid360_pointcloud_decoder_test` passed.
- `main`: `git diff --check` passed.

## Next Step
- If ROS1 Noetic is available on another machine, run the new `lidar_mid360_pointcloud_decoder_test` catkin gtest target there.
