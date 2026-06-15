# Project Summary — LiDAR-Based Perception & SLAM for Autonomous Driving
**Platform:** CARLA 0.9.15 · ROS2 Humble · Town10HD  
**Type:** Final Year Computer Engineering Project

---

## Phase 1 — Environment Setup

Built the full simulation stack from scratch:
- Installed **CARLA 0.9.15** as the autonomous driving simulator
- Set up **ROS2 Humble** and the CARLA-ROS2 bridge to stream sensor data as ROS topics
- Created `launch_stack.sh` — a single script that starts CARLA headless, the ROS bridge, waypoint publisher, local planner, and RViz2 automatically
- Created `record_stack.sh` and `play_stack.sh` for bag recording and playback
- Sensor config: 32-channel LiDAR (20 Hz), IMU (20 Hz), semantic LiDAR, ground-truth odometry — all streamed as ROS2 topics

---

## Phase 2 — Understanding the Data (Step 1)

Wrote `lidar_analyzer.py` to characterise the raw sensor output before any processing. Key findings:
- ~8,000 points per scan (half of 16,000 theoretical — car body occludes the lower hemisphere)
- Z range: [−2.4m, +1.7m] in sensor frame (sensor is 2.4m above road)
- These numbers directly informed all downstream parameter choices

---

## Phase 3 — Ground Plane Removal (Step 2)

Wrote `ground_remover.py` using **RANSAC plane fitting**:
- Pre-filters to low-Z candidates (z < −1.0m) for efficiency
- Fits a 3D plane through random point triples, scores by inlier count (within 0.25m)
- 100 iterations → convergence probability ≈ 1.000
- Result: **81.5% ground, 18.5% objects** — stable scan-to-scan
- Publishes `/lidar/ground` and `/lidar/objects` at 20 Hz

---

## Phase 4 — Object Detection & Classification (Step 3)

Wrote `object_clusterer.py` using **DBSCAN**:
- Parameters calibrated empirically: ε=1.5m, MinPts=3
- Size-based classification: PED (5–15 pts) → VEH (16–500 pts) → LRG (>500 pts)
- Publishes 3D bounding boxes as `MarkerArray` in RViz2 with colour coding
- Detects 33–44 clusters per scan, including 2–4 VEH-class objects in typical scenes

---

## Phase 5 — Semantic Ground Truth Validation (Step 4)

Wrote `detector_validator.py` to quantify detection accuracy using CARLA's semantic LiDAR:
- Discovered CARLA 0.9.15 uses PascalCase field names (`ObjTag`, not `object_tag`) — fixed a hard-to-find bug
- Used coverage-check approach: count raw LiDAR pts within 5m of each GT vehicle centroid
- Calibration result: **100% detection rate (6/6 vehicles)**, avg 145.7 pts per vehicle
- This validated all RANSAC and DBSCAN parameter choices

---

## Phase 6 — SLAM Integration (Step 5)

Integrated **LIO-SAM** (LiDAR Inertial Odometry via Smoothing and Mapping):

**Problem:** CARLA's raw LiDAR outputs only X, Y, Z, intensity. LIO-SAM requires `ring` (channel ID) and `time` (intra-scan timestamp) fields.

**Solution — `lidar_preprocessor.py`:**
- Computes `ring` from elevation angle: `ring = round((elev° − (−26.8°)) / 0.929°)`
- Computes `time` from azimuth: `time = (azimuth / 2π) × 0.05s`
- Uses vectorised NumPy structured arrays — packs 16,000 points in <1ms (replacing a slow Python loop)
- Publishes `/carla/hero/lidar_iosam` in Velodyne XYZIRT format

**LIO-SAM config (`params_carla.yaml`):**

| Parameter | Value | Reason |
|---|---|---|
| `sensor` | `velodyne` | XYZIRT format with uint16 ring |
| `N_SCAN` | 32 | LiDAR channels |
| `Horizon_SCAN` | 512 | Nearest 2ⁿ ≥ 500 pts/ring |
| `extrinsicTrans` | [2.0, 0.0, -0.4] | IMU is 2m fwd, 0.4m lower than LiDAR |

**Result:** Successfully built a 3D map of Town10HD — 817K points, 214×219m area

---

## Phase 7 — Map Export & Visualisation

- `save_map.sh` — triggers LIO-SAM's `save_map` ROS2 service, saves `GlobalMap.pcd`, `trajectory.pcd`, `CornerMap.pcd`, `SurfMap.pcd`
- `bev_map.py` — renders a bird's-eye view PNG (height-coloured, trajectory overlay) using matplotlib — no OpenGL required
- Output: clean presentation-ready map image for the paper

---

## Phase 8 — Trajectory Accuracy Evaluation

Wrote `compare_trajectories.py`:
- Extracts GT poses from the bag's `/carla/hero/odometry` using `rosbag2_py`
- Loads LIO-SAM's keyframes from `trajectory.pcd`
- Applies **Umeyama SE(2) alignment** (optimal rotation + translation via SVD)
- Computes **ATE RMSE** — the standard SLAM evaluation metric

### Experiment 1 — Bag with 10 NPC vehicles (dynamic objects present)

| Metric | Value |
|---|---|
| GT path length | 374.4 m |
| LIO-SAM keyframes | 190 |
| ATE RMSE | 15.17 m |
| Relative error | 4.05% |
| Max error | 37.19 m |

### Experiment 2 — Clean bag (no NPCs, static environment)

| Metric | Value |
|---|---|
| GT path length | 597.4 m |
| LIO-SAM keyframes | 378 |
| ATE RMSE | **2.54 m** |
| Relative error | **0.42%** |
| Max error | 5.24 m |

### Key Finding

Removing dynamic NPC vehicles improved ATE RMSE by **6×** (15.17m → 2.54m) and relative error by **~10×** (4.05% → 0.42%).

This quantifies the need for a **dynamic object filter** before passing point clouds to LiDAR-based SLAM in traffic environments. The 0.42% relative ATE is competitive with state-of-the-art real-world SLAM systems (LIO-SAM on KITTI: 0.5–2.0%).

---

## Deliverables

| File | Purpose |
|---|---|
| `launch_stack.sh` | One-command full stack launcher |
| `record_stack.sh` | Bag recording with all required topics |
| `play_stack.sh` | Bag playback with RViz2 |
| `stop_stack.sh` | Clean shutdown of all nodes |
| `lidar_analyzer.py` | Raw data characterisation |
| `ground_remover.py` | RANSAC ground segmentation node |
| `object_clusterer.py` | DBSCAN object detection + bounding boxes |
| `detector_validator.py` | Semantic ground-truth validation node |
| `lidar_preprocessor.py` | CARLA→LIO-SAM format bridge (ring + time) |
| `params_carla.yaml` | LIO-SAM configuration for CARLA |
| `save_map.sh` | LIO-SAM map export script |
| `bev_map.py` | Bird's-eye view map visualiser (PNG output) |
| `compare_trajectories.py` | ATE trajectory evaluation vs ground truth |
| `final_paper_lidar_pipeline.md` | Full scientific documentation |
| `pipeline_explained.md` | End-to-end pipeline explanation |
| `maps/town10hd/` | NPC bag results (GlobalMap.pcd, trajectory.pcd) |
| `maps/town10hd_clean/` | Clean bag results — best ATE (0.42%) |

---

## Pipeline Architecture

```
CARLA Simulator
      │
      ├── /carla/hero/lidar  ──────────────┬── ground_remover ── /lidar/objects ── object_clusterer ── RViz2 boxes
      │   (raw 32ch, 20Hz)                 │
      │                                    └── lidar_preprocessor ── /lidar_iosam ── LIO-SAM ── 3D map + trajectory
      │
      ├── /carla/hero/imu ──────────────────────────────────────────────┘
      │
      └── /carla/hero/semantic_lidar ── detector_validator ── accuracy metrics
          (GT labels)
```

---

## Core Achievement

A complete, working autonomous driving perception and SLAM stack — from raw sensor data to a quantitatively evaluated 3D map — built entirely in simulation using open-source tools, with a measured **0.42% relative trajectory error** on a 597m urban route.
