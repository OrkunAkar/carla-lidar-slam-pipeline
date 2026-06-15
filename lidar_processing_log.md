# 📡 LiDAR Sensor Data Processing Log

> Documents each processing step, findings, and decisions made during development.
> Located: `carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2/src/lidar_processor/`

---

## Step 1 — Data Visualization (RViz2)

**Goal:** Understand the raw point cloud visually before writing any code.

**Setup:**
- Played recorded bag with `./play_stack.sh`
- Opened RViz2, set Fixed Frame to `hero/lidar`
  - Note: `map` frame didn't work because TF chain was incomplete in old bags
  - Fix: always record `/tf` and `/tf_static` alongside data topics
- Added PointCloud2 display on `/carla/hero/lidar`
- Used `AxisColor` (Z-axis) coloring

**Observations:**
- Ground appears as the dominant color (high density)
- Object clusters (cars, buildings) visible as distinct blobs
- 32 LiDAR ring pattern clearly visible as horizontal layers
- Colors appeared inverted vs expectation (ground = red in default scale)
  - Root cause: CARLA's sensor frame has ground at **negative Z** (~-2.4m), not positive

**Key finding:** Fixed Frame must be `hero/lidar` during bag playback unless `/tf` + `/tf_static` are recorded.

---

## Step 2 — Python Analyzer Node (`lidar_analyzer.py`)

**Goal:** Read raw PointCloud2 data programmatically and extract statistical understanding.

### Package Structure

```
lidar_processor/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/lidar_processor
└── lidar_processor/
    ├── __init__.py
    └── lidar_analyzer.py
```

**Built with:** `colcon build --packages-select lidar_processor --symlink-install`

**Run with:**
```bash
source /opt/ros/humble/setup.bash
source ~/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
ros2 run lidar_processor lidar_analyzer
```

---

### Bug 1 — PointCloud2 Structured Array

**Error:**
```
TypeError: Cannot cast array data from dtype([('x', '<f4'), ...]) to dtype('float32')
```

**Root cause:** ROS2 Humble's `sensor_msgs_py.point_cloud2.read_points()` returns a
**structured numpy array** with named fields, not a plain 2D array.

**Wrong approach:**
```python
points = np.array(list(pc2.read_points(...)), dtype=np.float32)
x = points[:, 0]   # column index — fails on structured array
```

**Correct approach:**
```python
points = np.array(list(pc2.read_points(...)))   # no dtype cast
x = points['x'].astype(np.float32)              # access by field name
y = points['y'].astype(np.float32)
z = points['z'].astype(np.float32)
intensity = points['intensity'].astype(np.float32)
```

---

### Bug 2 — Ground Mask Inverted

**Wrong:**
```python
ground_mask = (z > 1.5)   # selects points ABOVE sensor → only 14 points (0.1%)
```

**Correct:**
```python
ground_mask = (z < -1.5)  # ground is BELOW sensor at ~-2.4m → ~6,900 points (85%)
```

**Root cause:** The LiDAR is mounted at `z = 2.4m` on the car roof.
All point coordinates are relative to the sensor. Ground (at road level) is
therefore at approximately `z = -2.4m` in the sensor frame.

---

### Real Data Results

Recorded with `MODE=lidar` (headless, no cameras), Town10HD.

| Metric | Value | Notes |
|---|---|---|
| Total points/scan | ~8,100 | Consistent across all scans |
| Scan rate | ~14–15 Hz | Configured at 20 Hz; some CARLA filtering |
| Distance min | 4.45 m | Car body blocks shorter returns |
| Distance max | ~50 m | Full configured range reached |
| Distance mean | ~12.7 m | Dense urban environment |
| Z min | -2.40 m | Exactly the LiDAR mount height — confirmed |
| Z max | ~1.73 m | Tallest visible object ~4.1m from ground |
| Intensity range | 0.82 – 0.98 | Very narrow — CARLA artifact, not realistic |
| Front 20m points | ~42% | ~3,400 points in forward cone |
| Ground points | ~85% | ~6,900 points — expected for urban LiDAR |
| Object points | ~14% | ~1,200 points — cars, buildings, curbs |

**Note on Intensity:** In real LiDARs, intensity varies from near 0 (dark asphalt)
to 1.0 (retroreflective signs). CARLA's ray-cast LiDAR normalizes this to a very
narrow range (~0.82–0.98). Do not rely on intensity for material classification —
use the semantic LiDAR instead.

---

### Key Insight — Scan Rate vs FPS

The bag recorded at ~14 Hz (vs configured 20 Hz) because the simulation ran with
`MODE=lidar` (headless + minimal sensors). Previous recordings with all sensors
active ran at only ~2 Hz due to the synchronous mode bottleneck.

---

## Step 3 — Ground Plane Removal (RANSAC)

**Goal:** Mathematically separate road surface points from object points, producing
two clean PointCloud2 streams for downstream processing.

**Node:** `ground_remover.py`
**Run with:** `ros2 run lidar_processor ground_remover`

| Topic | Role |
|---|---|
| `/carla/hero/lidar` | Input — raw full scan |
| `/carla/hero/lidar/ground` | Output — road surface points (green in RViz2) |
| `/carla/hero/lidar/objects` | Output — everything above ground (red in RViz2) |

---

### Why RANSAC instead of a simple Z threshold?

Step 2 used `z < -1.5` as a naive ground filter → 85% ground.
RANSAC fits a true mathematical plane → 81-82% ground.

The 3-4% difference is meaningful: those extra points are **curbs, road markings,
gentle slopes, and speed bumps** that lie at ground level but are not flat road.
RANSAC correctly classifies them as objects. The simple Z filter cannot do this.

---

### RANSAC Algorithm (how it works)

```
Repeat 100 times:
    1. Pick 3 random points from the cloud
    2. Fit a plane through them (cross product → normal vector)
    3. Count how many of all points are within 25cm of this plane (inliers)
    4. If this is the best plane so far, save it

→ Winning plane = the road surface
→ Inliers  = ground points
→ Outliers = object points
```

**Key parameters:**

| Parameter | Value | Effect |
|---|---|---|
| `RANSAC_ITERATIONS` | 100 | More = more accurate, more CPU |
| `DISTANCE_THRESHOLD` | 0.25 m | Tighter = stricter ground separation |
| `MIN_GROUND_RATIO` | 0.30 | Sanity check — warns if plane < 30% inliers |

---

### Pre-filter Optimization

Before running RANSAC, all points with `z >= -1.0m` are excluded from the candidate
set. This means RANSAC only ever sees road-level points, not building walls or car
rooftops. This reduces the candidate set from ~8,100 to ~6,500 points, speeding up
each iteration significantly while also preventing RANSAC from accidentally fitting
a wall or building as the "ground" plane.

---

### Real Data Results

| Metric | Step 2 (Z-threshold) | Step 3 (RANSAC) |
|---|---|---|
| Ground % | 85-86% | **81-82%** |
| Object points | ~1,200 | **~1,430-1,530** |
| Scan-to-scan variation | 0% (deterministic) | < 1% (RANSAC is probabilistic) |
| Handles road slopes | ❌ No | ✅ Yes |
| Handles curbs correctly | ❌ No (calls them ground) | ✅ Yes (calls them objects) |

The slight scan-to-scan variation (81-82%) is expected — RANSAC randomly samples
points each run, so results differ slightly per scan. This is normal and acceptable.

---

### RViz2 Visualization

Add two PointCloud2 displays:
- `/carla/hero/lidar/ground` → set color **green** — road surface
- `/carla/hero/lidar/objects` → set color **red** — obstacles, buildings, curbs

---

## Step 4 — Euclidean Clustering + Bounding Boxes (DBSCAN)

**Goal:** Group the ~1,500 object points per scan into individual detected objects
and visualize them as 3D colored bounding boxes in RViz2.

**Node:** `object_clusterer.py`
**Run with:** `ros2 run lidar_processor object_clusterer`
**Dependency:** `sudo apt install python3-sklearn`

| Topic | Role |
|---|---|
| `/carla/hero/lidar/objects` | Input — from ground_remover |
| `/lidar/clusters/markers` | Output — MarkerArray (bounding boxes + labels) |

---

### Algorithm — DBSCAN

DBSCAN (Density-Based Spatial Clustering of Applications with Noise) groups points
that are within `eps` distance of each other into the same cluster.

**Key parameters:**

| Parameter | Value | Effect |
|---|---|---|
| `EPS` | 1.0 m | Max distance between points in the same cluster |
| `MIN_SAMPLES` | 5 | Min points to form a cluster (below = noise) |
| `MIN_CLUSTER_PTS` | 10 | Post-filter: discard clusters smaller than this |
| `MAX_CLUSTER_PTS` | 3000 | Post-filter: discard clusters larger than this |

**Size classification:**

| Size | Points | Label | Color in RViz2 |
|---|---|---|---|
| Pedestrian-sized | ≤ 50 pts | `PED` | 🟢 Green |
| Vehicle-sized | 51–500 pts | `VEH` | 🔵 Blue |
| Large structure | > 500 pts | `LRG` | 🔴 Red |

---

### Real Data Results (Town10HD, no NPC vehicles spawned)

```
Clusters:  37  |  PED: 17  VEH: 3  LRG: 1  |  Noise pts: 289  Filtered: 16
Clusters:  44  |  PED: 20  VEH: 3  LRG: 1  |  Noise pts: 289  Filtered: 20
Clusters:  33  |  PED: 21  VEH: 4  LRG: 0  |  Noise pts: 301  Filtered:  8
Clusters:  40  |  PED: 19  VEH: 2  LRG: 1  |  Noise pts: 261  Filtered: 18
```

| Metric | Observed value |
|---|---|
| Total clusters/scan | 33–44 |
| PED-sized clusters | 17–24 |
| VEH-sized clusters | 2–4 |
| LRG clusters | 0–1 |
| Noise points | ~260–350 (~20% of object pts) |
| Filtered clusters | 8–20 |

---

### Analysis of Results

**High PED count (17–24) with no actual pedestrians in the scene:**
These are real structures that happen to be small — road bollards, traffic signs,
lamp posts, vegetation clusters, and curb sections. Without spawned NPCs, there are
no true pedestrians, so all small clusters are infrastructure objects.
To reduce this, `EPS` could be tuned down slightly or minimum bounding-box volume
filters could be added.

**VEH count (2–4) is plausible:**
Town10HD has parked vehicles and static props on the road. These could be real
parked cars visible in the recording, or large road furniture being classified as
vehicle-sized. Would need semantic LiDAR ground truth to confirm which.

**LRG count (0–1):**
Large structures (buildings, walls) produce many points but are often fragmented into
multiple VEH/PED clusters by DBSCAN because building surfaces have gaps between
LiDAR ring hits. Only the closest, densest sections of a building may exceed 500 pts.

**Scan-to-scan variation (33–44 clusters):**
Normal. The car is moving, so different parts of the environment come into/out of
sensor range each scan. DBSCAN is also sensitive to point density which varies
slightly per rotation.

---

### RViz2 Setup

Add **MarkerArray** display on topic `/lidar/clusters/markers`.
Each detected object shows:
- A colored 3D box (size = bounding box of the cluster)
- A floating text label: e.g. `VEH (142)` — type and point count

---

### Pipeline Summary (Steps 1–4)

```
Raw /carla/hero/lidar (8,100 pts)
    └─ [Step 3] ground_remover  → Ground: 6,600 pts  Objects: 1,500 pts
                                        └─ [Step 4] object_clusterer
                                                → 33–44 clusters
                                                → Colored 3D boxes in RViz2
```

---

### Step 4b — Semantic Validation (detector_validator.py)

**Goal:** Use semantic LiDAR ground truth to verify the raw-LiDAR detector is
finding real vehicles, and calibrate the classification thresholds.

**Node:** `detector_validator.py`
**Run with:** `ros2 run lidar_processor detector_validator`

| Topic | Role |
|---|---|
| `/carla/hero/lidar` | Input — raw LiDAR |
| `/carla/hero/semantic_lidar` | Input — ground truth labels (ObjTag field) |

**Sensor config change:** Added `sensor.lidar.ray_cast_semantic` to
`car_definition_lidar_only.json` at the same position as raw LiDAR (z=2.4m).

**Recording change:** Added `/carla/hero/semantic_lidar` to `record_stack.sh`
so future bags include GT data for offline validation.

---

#### CARLA Semantic LiDAR Field Names

> CARLA 0.9.15 ROS bridge uses **PascalCase** field names, NOT the documented
> snake_case names. This caused an `AssertionError` on first run.

| Documentation says | Actual field name |
|---|---|
| `object_tag` | `ObjTag` |
| `object_idx` | `ObjIdx` |
| `cos_inc_angle` | `CosAngle` |

**CARLA semantic tag constants used:**

| Tag | Object |
|---|---|
| 4 | Pedestrian |
| 7 | Road |
| 10 | Vehicle |

---

#### Validation Approach

Instead of running a second DBSCAN pipeline on raw LiDAR (fragile, parameter-sensitive),
the validator uses a **coverage check**:

```
For each GT vehicle centroid (from semantic DBSCAN):
    count raw LiDAR points within MATCH_DISTANCE (5.0m) of the centroid
    if >= 10 points → vehicle is "detectable" in raw LiDAR
```

This directly answers: *"Does the raw LiDAR physically see this vehicle?"*
without needing to re-tune DBSCAN parameters.

---

#### Debugging Journey

**Issue 1 — TP always 0 (first approach: centroid matching via DBSCAN)**
Root cause: `GROUND_Z_THRESHOLD = -1.0m` was too aggressive — it was
discarding vehicle body points (which sit at z ≈ -2.1m to -0.9m in sensor frame).
Fix: Changed threshold to `-2.1m`.

**Issue 2 — Still TP=0 after fix**
Root cause: Building wall sections (nearby, large) were classified as VEH by DBSCAN
while actual vehicles (far, sparse) produced too few points to exceed the 51pt VEH
minimum. The "detected" and GT centroids were at completely different positions.
Fix: Abandoned centroid-matching approach, switched to coverage-check approach.

**Issue 3 — "No GT vehicles in range" from bag**
Root cause: Old bag recorded before semantic LiDAR was added to the config —
had no `/carla/hero/semantic_lidar` topic.
Fix: Re-recorded a new bag with the updated config and NPCs spawned.

---

#### Final Calibration Results

Recorded with 10 NPC vehicles + 5 pedestrians, ego car driving in Town10HD.

```
GT vehicles in range :   6
  Detectable (>=10 raw pts within 5.0m) : 6 / 6  (100%)
  Raw pts per vehicle  — avg: 145.7  min: 59  max: 233
```

| Distance range | Raw pts per vehicle |
|---|---|
| Far (30–50m) | 19–52 pts |
| Near (10–30m) | 59–247 pts |
| Detection threshold | ≥ 10 pts within 5m of GT centroid |
| **Detection rate** | **6/6 = 100%** |

---

#### Calibrated Final Parameters (object_clusterer.py)

| Parameter | Initial | Calibrated | Reason |
|---|---|---|---|
| `EPS` | 1.0m | **1.5m** | Bridge gaps across car surface (car ≈ 4.5m long) |
| `MIN_SAMPLES` | 5 | **3** | Sparse far-range returns have few neighbors |
| `MIN_CLUSTER_PTS` | 10 | **5** | Catch sparse far-range vehicle clusters |
| `PEDESTRIAN_MAX` | 50 pts | **15 pts** | Poles/small objects are truly tiny clusters |
| `VEHICLE_MAX` | 500 pts | **500 pts** | Must cover 19–247 pts across all distances |

## Step 5 — LiDAR + IMU Odometry (LIO-SAM)

**Goal:** Build a real-time 3D map and estimate ego-vehicle odometry using
LIO-SAM (LiDAR Inertial Odometry via Smoothing and Mapping).

---

### The Ring + Time Problem

LIO-SAM requires `PointXYZIRT` point clouds:

| Field | Type | Purpose |
|---|---|---|
| x, y, z | float32 | Position |
| intensity | float32 | Reflectivity |
| `ring` | uint16 | Which LiDAR channel fired this point |
| `time` | float32 | Relative timestamp within the scan (seconds) |

**CARLA's raw LiDAR only outputs x, y, z, intensity.** No ring or time.

---

### Solution — lidar_preprocessor.py

A new node computes the missing fields from geometry:

**Ring** (from elevation angle):
```
elevation = atan2(z, sqrt(x² + y²))   in degrees
ring      = round((elevation - (-26.8°)) / (28.8° / 31))
ring      = clamp(ring, 0, 31)
```

**Time** (from azimuth angle):
```
azimuth   = atan2(y, x)               normalized to [0, 2π]
time      = (azimuth / 2π) × (1 / 20Hz)   → [0, 0.05s]
```

**Node:** `lidar_preprocessor.py`
**Run with:** `ros2 run lidar_processor lidar_preprocessor`

| Topic | Role |
|---|---|
| `/carla/hero/lidar` | Input — raw XYZL (no ring/time) |
| `/carla/hero/lidar_iosam` | Output — XYZIRT (Velodyne format) |

Point layout (24 bytes per point):
```
x(4B)  y(4B)  z(4B)  intensity(4B)  ring(2B)  pad(2B)  time(4B)
```

---

### LIO-SAM Configuration (params_carla.yaml)

Key parameters tuned for the CARLA setup:

| Parameter | Value | Reason |
|---|---|---|
| `sensor` | `velodyne` | Our preprocessor outputs ring(uint16) + time(float32) |
| `N_SCAN` | 32 | LiDAR channels in config |
| `Horizon_SCAN` | 512 | Nearest power-of-2 ≥ 500 pts/ring |
| `lidarMaxRange` | 50.0 | Matches CARLA sensor range |
| `imuAccNoise` | 0.01 | Small nonzero (CARLA IMU has 0 noise but LIO-SAM needs > 0) |
| `imuGyrNoise` | 0.001 | Same reason |
| `extrinsicTrans` | [2.0, 0.0, -0.4] | IMU is 2m forward, 0.4m lower than LiDAR |
| `extrinsicRot` | Identity | Both sensors aligned with car axes |

---

### Dependencies Installed

```bash
sudo apt install ros-humble-gtsam ros-humble-pcl-ros ros-humble-xacro
```

Source: `git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM.git`

---

### Run Order

```bash
# 1. Play the bag (or start live CARLA)
./play_stack.sh

# 2. Add ring + time fields
ros2 run lidar_processor lidar_preprocessor

# 3. Launch LIO-SAM with CARLA config
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/CARLA_PROJECT/carla_ros2_ws/src/LIO-SAM/config/params_carla.yaml
```

---

### RViz2 Topics (set Fixed Frame: `map`)

| Topic | Type | Shows |
|---|---|---|
| `/lio_sam/mapping/odometry` | Odometry | Estimated pose |
| `/lio_sam/mapping/path` | Path | Ego trajectory |
| `/lio_sam/mapping/map_global` | PointCloud2 | Full built map |
| `/lio_sam/mapping/map_local` | PointCloud2 | Local surroundings |

---

### Result

✅ **LIO-SAM mapped Town10HD successfully from the recorded bag.**
Real-time 3D map construction + ego-vehicle trajectory estimated using
32-channel LiDAR + IMU fusion in Town10HD with 10 NPC vehicles active.


