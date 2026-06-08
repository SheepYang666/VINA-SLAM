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

本节解释的是 `full_5x_ba1_dir` 这次历史回放中观察到的现象。该 run 使用的是切换到 `normalFactor` overload 之前的原 Lidar+IMU BA；当前代码已经改成：

```cpp
// opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
opt_lsv.damping_iter(x_buf, voxhess, normalFactor, imu_pre_buf, &hess);
```

所以后续仍需要用同一数据集重新回放，确认加入 `normalFactor` 后是否改善。但“为什么开启 BA 反而会把 Z 拉偏”的机制，可以从这次 CSV 数据里看得比较清楚。

### 1. BA 不是全局高度约束，而是局部相对一致性优化

BA 优化目标可以近似理解为：

```text
min E(x) = E_lidar(x) + imu_coef * E_imu(x)
```

这里没有 GNSS 高程、地面高度、气压计或其他绝对 Z 约束。IMU 预积分也是相邻状态之间的相对约束，不是绝对高度锚点。因此 BA 只能判断“窗口内状态是否互相一致”，不能直接判断“全局 Z 是否真实”。

当局部几何对 Z 方向约束很弱时，Hessian 在 Z 方向会接近病态。直观地说，优化器沿 Z 方向移动窗口内一批位姿，LiDAR residual 可能变化很小；如果这个移动还能让 IMU residual 下降，局部目标函数就会认为这是一个更好的解，即使全局高度被带偏。

### 2. 数据显示：开 BA 后不是最后一帧才出问题，而是前端约束逐步变弱

同一段数据中，关闭 BA 和开启 BA 的差异很明显：

```text
不开 BA full_5x_ba0_wait:
  退化帧数: 0
  final z_ba: -18.402 m
  frames 6768 -> 6819 的 z 变化: -2.001 m
  frames 6798 -> 6832 的 z 变化: -0.755 m

开 BA full_5x_ba1_dir:
  退化帧数: 12
  final z_ba: -25.029 m
  frames 6768 -> 6819 的 z 变化: -6.312 m
  frames 6798 -> 6832 的 z 变化: -4.571 m
```

也就是说，开 BA 后最后一段并不是多漂了几十厘米，而是比不开 BA 多下坠了约 `6.6 m`。更关键的是，前端 `nnt_eig0` 和匹配数在退化前已经开始明显恶化：

| frame | BA off nnt0 | BA off match | BA on nnt0 | BA on match | BA on lio_success |
|---:|---:|---:|---:|---:|---:|
| 6785 | 356.710 | 2454 | 320.199 | 2406 | 1 |
| 6790 | 356.939 | 2287 | 217.466 | 2081 | 1 |
| 6795 | 207.335 | 2121 | 14.218 | 1138 | 1 |
| 6798 | 294.432 | 2162 | 5.545 | 1148 | 0 |
| 6799 | 234.827 | 2254 | 5.247 | 863 | 0 |
| 6805 | 175.484 | 2154 | 4.770 | 638 | 0 |
| 6810 | 216.975 | 2087 | 10.701 | 1158 | 0 |
| 6819 | 219.336 | 1922 | 44.715 | 979 | 1 |

`nnt_eig0` 是当前前端匹配累计法向协方差矩阵的最小特征值。它越小，说明存在越明显的弱约束方向。开 BA 后，`nnt_eig0` 从 `6790` 开始持续变差，到 `6795` 已经接近退化阈值 `14`，`6798` 直接跌到 `5.545` 并返回失败。

这说明 BA 的影响已经反馈到了后续 LIO 的匹配环境：不开 BA 时，同一帧附近还有较稳定的几何约束；开 BA 后，前端看到的局部地图/状态组合已经让匹配质量显著下降。

### 3. 退化方向几乎就是 Z 轴，所以错误主要沿 Z 放大

首个退化帧 `6798` 的弱约束方向为：

```text
weak direction: (-0.0379, 0.0078, 0.9993)
angle to vertical: 2.22 deg
```

这说明当时系统最不确定的方向几乎就是世界系 Z 方向。此时 BA 或 LIO 中任何很小的 Z 方向偏置，都可能被弱约束放大。换句话说，这不是一个普通随机抖动，而是“最容易漂的方向刚好就是 Z”。

退化段附近平均弱约束方向为：

```text
frames 6788-6805 mean weak direction: (-0.0185, -0.3437, 0.9389)
angle to vertical: 20.13 deg
```

所以该区域不是单帧偶然退化，而是一段持续的 Z 主导弱约束区域。

### 4. BA 在局部目标函数上看起来“合理”，但它接受了错误的 Z 更新

`6798` 帧是最典型的例子：

```text
z_lio:       -18.333
z_ba:        -18.824
delta_z_ba:   -0.490

imu_res_before:    0.896
imu_res_after:     0.006
lidar_res_before:  0.071
lidar_res_after:   0.072
normal_res_before: 5.251
normal_res_after:  5.440
```

这一帧 BA 把 Z 又往下拉了 `0.490 m`。从分项 residual 看：

- IMU residual 从 `0.896` 降到 `0.006`，大幅变好。
- LiDAR residual 从 `0.071` 到 `0.072`，几乎没变。
- Normal residual 从 `5.251` 到 `5.440`，略微变差。

由于当时实际优化调用是原 Lidar+IMU BA，`normalFactor` 并没有参与优化，只是被日志评估出来。因此优化器真正看到的是：IMU residual 大幅下降，LiDAR residual 基本不变。局部目标函数会倾向于接受这次更新。

但从全局轨迹看，这次更新把 Z 推到了错误方向。类似的负向 Z 修正在后面还继续出现：

| frame | lio_success | nnt0 | z_lio | z_ba | delta_z_ba | imu_res_before -> after | lidar_res_before -> after |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6785 | 1 | 320.199 | -16.881 | -16.908 | -0.027 | 0.704 -> 0.033 | 0.152 -> 0.157 |
| 6790 | 1 | 217.466 | -17.243 | -17.280 | -0.038 | 1.018 -> 0.039 | 0.112 -> 0.114 |
| 6798 | 0 | 5.545 | -18.333 | -18.824 | -0.490 | 0.896 -> 0.006 | 0.071 -> 0.072 |
| 6805 | 0 | 4.770 | -20.231 | -20.584 | -0.353 | 2.071 -> 0.006 | 0.104 -> 0.104 |
| 6810 | 0 | 10.701 | -20.796 | -21.018 | -0.221 | 1.182 -> 0.023 | 0.088 -> 0.092 |
| 6819 | 1 | 44.715 | -22.356 | -22.673 | -0.317 | 1.545 -> 0.009 | 0.165 -> 0.171 |

这些数据说明：BA 多次通过较大的 Z 方向修正显著降低 IMU residual，而 LiDAR residual 基本没有给出足够强的反对信号。由于该段 Z 方向弱约束，优化器可以在 Z 上移动状态而不显著增加 LiDAR 代价。

### 5. 真正危险的是 BA 结果会反馈进下一帧前端和地图

当前反馈链路是：

```text
LIO 估计当前帧
BA 优化 x_buf
优化后的 x_buf 末帧写回 x_curr
优化后的窗口参与局部地图边缘化/更新
下一帧 LIO 匹配已经被 BA 偏移过的局部地图
```

对应代码流程在 `src/pipeline/local_mapping.cpp` 中：

```text
1. lio_state_estimation(pptr) 更新 x_curr
2. pvec_update(pptr, x_curr, pwld)
3. x_buf.push_back(x_curr)
4. multi_recut(...) 根据当前 x_buf 生成 BA 约束
5. opt_lsv.damping_iter(...) 优化 x_buf
6. x_curr.R / x_curr.p 被替换成优化后的窗口末帧
7. multi_margi(...) 使用优化后的窗口更新局部地图
```

所以 BA 的错误 Z 更新不是只影响一行输出轨迹。它会改变：

```text
x_curr
x_buf
局部地图边缘化结果
下一帧 LIO 的匹配参考
```

这就是为什么开启 BA 后会形成正反馈：

```text
BA 沿 Z 接受一次偏移
-> 偏移写回 x_curr 和局部地图
-> 下一帧 LIO 在偏移地图上匹配
-> nnt_eig0 继续降低，Z 约束继续变弱
-> LIO/BA 更容易继续沿 Z 漂
```

这也解释了为什么不开 BA 反而正常一些：不开 BA 时没有这个“回头改窗口、再写回地图”的反馈环。前端虽然也会有 Z 变化，但不会把一个窗口级错误优化结果持续注入后续局部地图。

### 6. IMU 原始数据不是主要嫌疑

退化窗口 IMU 原始数据统计并不异常：

```text
退化窗口 dt mean: 0.004999 s
退化窗口 dt max:  0.006020 s
每帧 imu_count: 21

acc_norm 退化窗口 p99:  5.6766
acc_norm 全包 p99:      6.0228

gyro_norm 退化窗口 p99: 3.1307
gyro_norm 全包 p99:     3.5617
```

也就是说，退化处没有明显 IMU 断流、缺包或超出全局分布的爆值。BA 前 IMU residual 偏高，更像是前端/局部地图/窗口状态已经不一致，BA 通过移动状态把这个相对约束拉回去；但由于缺少绝对 Z 约束，拉回 IMU residual 的同时把全局 Z 拉偏。

### 7. 当前结论

基于现有数据，开启 BA 反而导致 Z 轴漂移的原因可以概括为：

```text
最后楼梯/走廊段存在 Z 主导的弱约束方向
-> BA 沿 Z 移动窗口状态时 LiDAR residual 几乎不惩罚
-> 该移动能显著降低 IMU residual，因此局部目标函数接受更新
-> BA 结果无条件写回 x_curr，并进入局部地图边缘化
-> 下一帧 LIO 匹配被偏移过的地图，前端 nnt_eig0 和匹配数继续恶化
-> 形成 Z 方向正反馈，最终开 BA 比不开 BA 掉得更多
```

因此问题重点不只是“BA 单帧优化算错”，而是“在 Z 弱约束场景下，BA 的局部最优结果被写回前端和地图，导致后续前端约束继续变差”。这也是后续应优先增加 `lio_success` gate、BA accept/reject gate、Z 弱约束阻尼或跳过写回的原因。

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
