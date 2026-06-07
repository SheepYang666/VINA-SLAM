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
