# Section 3: System Architecture

---

## 3. System Architecture

### 3.1 Overview

The system is built on three layers: a physics simulation layer (CARLA), a middleware layer (ROS2), and a processing layer (custom nodes + LIO-SAM). All processing nodes run concurrently, communicating via ROS2 topics at 20 Hz.

> **📌 Figure to add:**
> **Figure 6** — Use the pipeline diagram (Figure 2 from the Introduction) here, or a layered block diagram showing: [CARLA] → [ROS2 Bridge] → [Processing Nodes] → [Outputs]. Keep it simple — 3 horizontal layers.

---

### 3.2 Software Stack

| Component | Version | Role |
|---|---|---|
| CARLA Simulator | 0.9.15 | Physics engine, sensor simulation |
| CARLA ROS2 Bridge | 0.9.15 | Converts CARLA data to ROS2 topics |
| ROS2 | Humble (Ubuntu 22.04) | Inter-node communication |
| LIO-SAM | ros2 branch | LiDAR-Inertial SLAM |
| Python | 3.10 | All custom processing nodes |
| NumPy / scikit-learn | Latest | Point cloud maths and DBSCAN |
| GTSAM | 4.x | Factor graph optimisation (LIO-SAM dependency) |

---

### 3.3 Sensor Configuration

The ego vehicle carries two sensors relevant to this pipeline:

| Sensor | Type | Rate | Key Parameters |
|---|---|---|---|
| LiDAR | Rotating 32-ch | 20 Hz | Range: 50m, FOV: −26.8° to +2.0° |
| IMU | 6-DOF | 20 Hz | Simulated accelerometer + gyroscope |
| Semantic LiDAR | Ground-truth only | 20 Hz | Provides object class labels per point |

The LiDAR is positioned at (0, 0, 2.4m) on the vehicle roof. The IMU is located at (2.0, 0, 2.0m), 0.4m lower and 2m forward. This offset is specified as the LIO-SAM extrinsic calibration parameter (`extrinsicTrans: [2.0, 0.0, -0.4]`).

---

### 3.4 ROS2 Node Graph

Each pipeline stage runs as an independent ROS2 node. The data flow is:

```
/carla/hero/lidar ──┬── ground_remover    → /lidar/objects ── object_clusterer → /clusters/markers
(PointCloud2)       │
                    └── lidar_preprocessor → /lidar_iosam ──── lio_sam ─────────→ /mapping/map_global
                                                                                   /mapping/odometry
/carla/hero/imu ────────────────────────────────────────────── lio_sam

/carla/hero/semantic_lidar ─── detector_validator  (offline validation only)
/carla/hero/lidar ─────────────────────────┘
```

> **📌 Figure to add:**
> **Figure 7** — Recreate the node graph above as a clean box-and-arrow diagram. Use boxes for node names, labelled arrows for topic names. A simple hand-drawn or draw.io version works well here.

---

### 3.5 Recording and Playback

All experiments were conducted by recording ROS2 bag files (`.db3` SQLite format) during live CARLA simulation, then replaying them offline for pipeline execution. This approach allows:
- Reproducible experiments from a single recording
- Multiple pipeline configurations tested against identical input data
- Offline parameter tuning without re-running the simulator

Bags were recorded using a custom `record_stack.sh` script capturing five topics: `/carla/hero/lidar`, `/carla/hero/semantic_lidar`, `/carla/hero/imu`, `/carla/hero/odometry`, and `/tf`.

---

*Word count: ~380 words*
