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

## Z Drift BA Feedback
- Trigger: Debugging the outdoor 2026-06-04 dataset with local BA enabled near a stair/corridor segment.
- Failure: A front-end degenerate update can modify `x_curr` without rollback; local BA can then write a Z-biased window state back to `x_curr` and marginalize it into the local map, making the next LIO frame match against a biased map.
- Guardrail: Treat `lio_success == false` as a hard gate for pushing state into `x_buf` or the map; add an accept/reject gate before BA writeback when the weakest LIO direction is close to Z.
- Verification: `docs/z-drift-outdoor-20260604.md` records the local CSV evidence, degenerate frames, weak direction, IMU check, and BA-on vs BA-off comparison.
