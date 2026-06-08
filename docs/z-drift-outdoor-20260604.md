# 2026-06-04 室外数据集 Z 轴漂移调试记录

## 调试范围

本文记录以下数据集上的 Z 轴漂移问题：

```text
/home/zry/data_sets/test/outdoor_mapping_20260604_203924
```

当前调试分支：

```text
debug/z-drift-outdoor-20260604
```

本轮诊断只关注普通 LIO 前端路径：

```cpp
lio_state_estimation(pptr)
```

调试对照中已经移除 VNC 路径，避免前端日志混入 `VNC_lio()` 或 `no_ds_pptr` 相关变量。

## 本地分析产物

生成的分析结果已经从 `/tmp` 移动到本地目录：

```text
/home/zry/Code/VINA_SLAM/vina_slam_z_drift
```

本文主要引用以下完整回放结果：

```text
/home/zry/Code/VINA_SLAM/vina_slam_z_drift/full_5x_ba1_dir
```

关键文件：

```text
z_drift_frontend.csv
z_drift_ba.csv
trajectory.txt
direction_analysis.md
imu_degenerate_analysis.md
plots/01_z_timeline_with_degenerate_frame.png
plots/02_final_stair_direction_diagnostics.png
plots/03_xy_trajectory_weak_direction_arrows.png
plots/04_imu_diagnostics_degenerate_window.png
```

## 已增加的调试能力

新增 Debug 参数：

```yaml
Debug.enable_z_drift_log
Debug.run_label
Debug.log_root
Debug.fail_on_frontend_degenerate
```

调试 CSV 路径由 `vina_slam::core::make_z_drift_log_paths()` 生成。`run_label` 会先做路径安全清洗，避免空标签、路径分隔符和非法字符导致日志目录异常。该逻辑由 `debug_logging_test` 覆盖。

前端 CSV 记录字段：

```text
frame_id
pcl_beg_time
imu_count
raw_point_count
downsampled_point_count
z_after_imu
z_after_lio
delta_z_lio
delta_norm_lio
lio_success
last_match_num
nnt_eig0
nnt_eig1
nnt_eig2
nnt_min_dir_x
nnt_min_dir_y
nnt_min_dir_z
iterations
frontend_path
x_curr_modified_no_rollback
```

BA CSV 记录字段：

```text
frame_id
timestamp
ba_enabled
window_size
lidar_factor_count
normal_factor_count
imu_factor_count
z_before_ba
z_after_ba
delta_z_ba
delta_norm_ba
imu_res_before
lidar_res_before
normal_res_before
total_res_before
imu_res_after
lidar_res_after
normal_res_after
total_res_after
```

日志中特别记录了一个重要行为：

```text
如果 LioStateEstimation() 返回 false，x_curr 此时已经被修改。
当前流程没有在后续 local mapping 使用 x_curr 前做 rollback。
```

## 主要现象

### 关闭 BA

Run：

```text
full_5x_ba0_wait
```

摘要：

```text
front rows: 7583
BA rows:    7584
退化帧数: 0
最终 z: -18.402 m
frames 6768 -> 6819 的 z 变化: -2.001 m
```

### 开启 BA，原 Lidar+IMU BA

Run：

```text
full_5x_ba1_dir
```

摘要：

```text
front rows: 7583
BA rows:    7584
退化帧数: 12
连续退化段: 8
最终 z: -25.029 m
frames 6768 -> 6819 的 z 变化: -6.312 m
```

退化帧：

```text
6798, 6799,
6805, 6806,
6810,
6816,
6820,
6823,
6826, 6827,
6831, 6832
```

连续退化段：

```text
6798-6799
6805-6806
6810
6816
6820
6823
6826-6827
6831-6832
```

所以这组数据上，开启 BA 后结果更差：前端出现更多退化帧，最终 Z 下坠也明显更大。

## 退化方向

首个退化帧：

```text
frame: 6798
t_rel: 678.800 s
nnt_eig0: 5.545
last_match_num: 1148
weak direction: (-0.0379, 0.0078, 0.9993)
angle to vertical: 2.22 deg
```

最小特征值对应的弱约束方向几乎就是世界系 Z 方向。这说明该段局部 LIO 几何对竖直方向的观测很弱。此时即使只有很小的 residual 偏置、线性化误差或地图不一致，也可能被放大成明显的 Z 更新。

在 `6788-6805` 帧附近，平均弱约束方向为：

```text
(-0.0185, -0.3437, 0.9389)
angle to vertical: 20.13 deg
```

这个方向仍然以 Z 为主，同时带有一定 Y 分量，符合楼梯/走廊场景下沿通道方向和竖直方向约束变弱的现象。

## IMU 检查

退化区间附近的 `/livox/imu` 原始值看起来正常，没有明显断流、缺包或离群爆值。

原始 IMU 摘要：

```text
全包 IMU 样本数: 151911
全包 dt mean: 0.005000 s
全包 dt max:  0.006089 s

退化窗口样本数: 820
退化窗口 dt mean: 0.004999 s
退化窗口 dt max:  0.006020 s
```

加速度模长：

```text
全包 mean: 1.6586
全包 p99:  6.0228
全包 max:  6.9393

退化窗口 mean: 1.6727
退化窗口 p99:  5.6766
退化窗口 max:  6.3555
```

角速度模长：

```text
全包 mean: 0.7193
全包 p99:  3.5617
全包 max:  6.4565

退化窗口 mean: 0.9156
退化窗口 p99:  3.1307
退化窗口 max:  4.3710
```

退化窗口内有高分位运动样本，但仍在整包分布范围内，不像原始 IMU 传感器故障。

BA 日志中，退化区间 BA 前的 IMU residual 偏高，但 BA 后会被压回较低水平。这更像是 BA 前状态或轨迹与 IMU 预积分约束不一致，不是原始 IMU 测量本身异常。

## 为什么 BA 会让结果更差

BA 优化的是局部一致性，不知道真实全局高度：

```text
min E(x) = E_lidar(x) + imu_coef * E_imu(x)
```

当 Hessian 在 Z 方向弱约束时，优化器可以沿 Z 方向移动滑窗内的状态，同时 LiDAR residual 变化很小。如果这个移动还能降低 IMU residual，局部优化器就可能接受一个 Z 偏移后的解，即使该解的全局高度是错的。

当前反馈链路是：

```text
LIO 估计当前帧
BA 优化 x_buf
优化后的 x_buf 末帧写回 x_curr
优化后的窗口参与局部地图边缘化/更新
下一帧 LIO 匹配已经被 BA 偏移过的局部地图
```

因此，错误 BA 更新不只是轨迹输出上的显示问题。它会通过 `x_curr` 和局部地图污染下一帧前端匹配问题。

`6798` 帧示例：

```text
z_lio:       -18.333
z_ba:        -18.824
delta_z_ba:   -0.490

imu_res_before:    0.896
imu_res_after:     0.006
lidar_res_before:  0.071
lidar_res_after:   0.072
```

优化器显著降低了 IMU residual，同时 LiDAR residual 几乎没有变化。从局部目标函数看，这一步可能是可接受的；但它把 Z 推向了错误方向。

## 本调试分支包含的代码变化

本分支包含：

- Debug 配置字段和 ROS2 参数读取。
- 前端 CSV 和 BA CSV 日志。
- `LioDebugStats`，记录匹配点数、迭代次数、`nnt` 三个特征值和最小特征值方向。
- 明确记录前端路径为 `lio_state_estimation`。
- 新增调试配置文件 `config/debug_z_drift.yaml`。
- 新增 `run_label` 和日志路径生成单测。
- 新增 BA residual breakdown 辅助函数，分别记录 IMU、LiDAR、Normal residual。
- 当前 BA 调用已切到包含 `normalFactor` 的 overload。

旧 BA 调用被有意保留为注释：

```cpp
// opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
opt_lsv.damping_iter(x_buf, voxhess, normalFactor, imu_pre_buf, &hess);
```

这样后续可以直接对比旧的 Lidar+IMU BA 和新的 Lidar+Normal+IMU BA。

## 当前建议

短期保护措施：

```text
1. 如果 lio_success == false，不要把已经修改过的 x_curr 推入 x_buf 或局部地图。
2. BA 写回 x_curr 前增加 accept/reject gate。
3. 当 nnt_eig0 很低且最弱方向接近 Z 时，拒绝或阻尼 BA 的 Z 更新。
4. 使用同一完整数据集对比带 normalFactor 和不带 normalFactor 的 BA。
5. 扫描 LocalBA.imu_coef，验证更强 IMU 权重是否能减轻 Z 漂移。
```

建议实验：

```text
General.if_BA: 0
General.if_BA: 1，使用旧 Lidar+IMU BA
General.if_BA: 1，使用 Lidar+Normal+IMU BA
General.if_BA: 1，并提高 LocalBA.imu_coef
General.if_BA: 1，但检测到前端退化时不执行 BA 写回和地图更新
```

## 验证记录

构建：

```text
colcon build --packages-select vina_slam --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

结果：

```text
通过
```

聚焦测试：

```text
colcon test --packages-select vina_slam --ctest-args -R "debug_logging_test|lidar_mid360_pointcloud_decoder_test" --event-handlers console_direct+
```

结果：

```text
通过
```

完整包测试：

```text
colcon test --packages-select vina_slam --event-handlers console_direct+
```

结果：

```text
GTest 目标通过。
仓库级 lint/copyright/uncrustify 检查仍因既有大范围风格和版权问题失败。
```

完整测试失败不是 BA overload 切换导致的；`colcon build` 通过，相关 GTest 也通过。
