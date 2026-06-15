# LiDAR-Based 3D Perception and Simultaneous Localisation and Mapping for Autonomous Driving in Simulated Urban Environments

---

## Abstract

Autonomous vehicles require the ability to simultaneously perceive their environment and determine their own position within it — a challenge that becomes significantly more complex in dynamic urban settings where GPS-based localisation is unreliable or unavailable. This project presents a complete LiDAR-based perception and mapping pipeline, developed and evaluated within the CARLA open-source driving simulator using ROS2 Humble as the middleware framework.

The implemented system consists of five sequential processing stages applied to data from a simulated 32-channel rotating LiDAR sensor operating at 20 Hz. First, raw point cloud data is analysed to characterise sensor behaviour and establish baseline statistics. Second, road surface points are separated from object points using a RANSAC-based plane fitting algorithm, consistently achieving an 81.5% ground classification rate. Third, above-ground points are grouped into individual objects using DBSCAN density clustering with empirically calibrated parameters, producing 3D bounding box detections at 20 Hz. Fourth, detection accuracy is quantitatively validated against a semantic ground-truth sensor, confirming a 100% vehicle detection rate across all in-range targets. Fifth, a tightly-coupled LiDAR-Inertial Odometry system (LIO-SAM) is integrated to perform simultaneous 3D mapping and ego-motion estimation. A key technical contribution of this work is a preprocessing node that synthetically computes the per-point ring channel and intra-scan timestamp fields required by LIO-SAM — fields not provided by the CARLA sensor driver — enabling seamless integration without modifying the SLAM framework itself.

Trajectory accuracy was evaluated against CARLA's ground-truth odometry using the Absolute Trajectory Error (ATE) metric. In a static environment, the system achieved an ATE RMSE of 2.54 m over a 597.4 m route, corresponding to a relative error of 0.42% — comparable to real-world LiDAR-inertial SLAM benchmarks. A controlled experiment demonstrated that the presence of dynamic NPC vehicles degraded accuracy by a factor of six (ATE RMSE: 15.17 m, relative error: 4.05%), quantifying the impact of moving objects on scan-matching-based localisation and motivating the use of dynamic object filtering in future work.

The complete system runs in real time, producing concurrent object detections and a globally consistent 3D map, and serves as a functional foundation for further autonomous driving perception research.

**Keywords:** LiDAR, Point Cloud Processing, SLAM, ROS2, CARLA, RANSAC, DBSCAN, LIO-SAM, Autonomous Driving, Sensor Fusion

---

*Word count: ~290 words*
