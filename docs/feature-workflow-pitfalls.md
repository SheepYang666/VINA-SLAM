## MID360 PointCloud2 Output Buffer Reads
- Trigger: Implementing a Livox MID360 PointCloud2 decoder with parallel writes into an intermediate output cloud.
- Failure: Comparing `pl_surf[i]` with `pl_surf[i - 1]` can read a neighbor that has not been written yet.
- Guardrail: Compare duplicate points against the immutable input cloud, or keep the decoder loop sequential.
- Verification: `lidar_mid360_pointcloud_decoder_test` includes a duplicate input point and expects it to be filtered.

## Livox Type 0 Ambiguity
- Trigger: Adding MID360 PointCloud2 support while `lidar_type: 0` already selects Livox `CustomMsg`.
- Failure: Reusing type `0` makes the node subscribe to `CustomMsg` and never calls the PointCloud2 decoder.
- Guardrail: Keep `LIVOX=0` for CustomMsg and use `LIVOX_MID360_POINTCLOUD2=7` for standard PointCloud2 packets.
- Verification: `config/mid360.yaml` uses `lidar_type: 7`; existing CustomMsg Livox configs remain on `lidar_type: 0`.

## Z 漂移 BA 反馈链路
- 触发场景：调试 2026-06-04 室外数据集时，在楼梯/走廊末段开启 local BA。
- 失败模式：前端退化更新会在没有 rollback 的情况下修改 `x_curr`；local BA 随后可能把带 Z 偏差的窗口末帧写回 `x_curr`，并边缘化进局部地图，使下一帧 LIO 匹配到已经偏移的地图。
- 防护措施：将 `lio_success == false` 作为硬门限，禁止该状态进入 `x_buf` 或地图；当 LIO 最弱方向接近 Z 时，在 BA 写回前增加 accept/reject gate。
- 验证记录：`docs/z-drift-outdoor-20260604.md` 记录了本地 CSV 证据、退化帧、弱约束方向、IMU 检查和 BA 开关对比。
