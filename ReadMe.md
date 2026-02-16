# VINA-SLAM: A Voxel-based Inertial and Normal-Aligned LiDAR–IMU SLAM

Environments with sparse or repetitive geometric structures, such as long corridors and narrow stairwells, remain challenging for LiDAR-IMU SLAM. In these scenarios, insufficient geometric observability and noisy associations commonly degrade the reliability of tracking and optimization modules.

To address these limitations, we propose a novel LiDAR-IMU SLAM framework, \textbf{VINA-SLAM}, which establishes a unified 3D global voxel map and collects global structure cues into the voxel map.
To be specific, VINA-SLAM first continuously tracks surface normals stored in the global voxel map using a normal-guided correspondence strategy.

Then, a tangent-space metric is further proposed to explicitly supplement rotational constraints around planar regions into a local bundle adjustment module, providing robust initial pose estimates even in geometrically degenerate regions.

Finally, we formulate a sliding-window bundle adjustment module that tightly couples IMU factors, normal consistency factors, and planar constraints. A key component is the use of the minimum eigenvalue of each voxel's covariance, as a statistically principled planar factor that improves the Hessian condition number and enhances cross-view geometric consistency.


<img width="2330" height="1257" alt="2026-01-24_14-21" src="https://github.com/user-attachments/assets/568dde16-0b20-4e5e-8117-209ae2bdef01" />

<img width="2158" height="1197" alt="2026-01-24_14-23" src="https://github.com/user-attachments/assets/29b904eb-cd79-43a2-afdb-df285c5ec137" />
