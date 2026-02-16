# VINA-SLAM: A Voxel-based Inertial and Normal-Aligned LiDAR–IMU SLAM

Environments with sparse or repetitive geometric structures, such as long corridors and narrow stairwells, remain challenging for LiDAR-IMU SLAM. In these scenarios, insufficient geometric observability and noisy associations commonly degrade the reliability of tracking and optimization modules.

To address these limitations, we propose a novel LiDAR-IMU SLAM framework, \textbf{VINA-SLAM}, which establishes a unified 3D global voxel map and collects global structure cues into the voxel map.
To be specific, VINA-SLAM first continuously tracks surface normals stored in the global voxel map using a normal-guided correspondence strategy.

Then, a tangent-space metric is further proposed to explicitly supplement rotational constraints around planar regions into a local bundle adjustment module, providing robust initial pose estimates even in geometrically degenerate regions.

Finally, we formulate a sliding-window bundle adjustment module that tightly couples IMU factors, normal consistency factors, and planar constraints. A key component is the use of the minimum eigenvalue of each voxel's covariance, as a statistically principled planar factor that improves the Hessian condition number and enhances cross-view geometric consistency.

!(/home/zry/图片/senGuFloor3/2026-01-24_14-24.png)

![2026-01-24_14-23](/home/zry/图片/senGuFloor3/2026-01-24_14-23.png)

![2026-01-24_14-21](/home/zry/图片/senGuFloor3/2026-01-24_14-21.png)
