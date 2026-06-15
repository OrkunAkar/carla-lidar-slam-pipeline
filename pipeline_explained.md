# LiDAR Perception + SLAM Pipeline — End-to-End Explanation

> **Project:** CARLA Autonomous Driving — Final Year Project  
> **Stack:** ROS2 Humble · CARLA 0.9.15 · DBSCAN · RANSAC · LIO-SAM  
> **Map:** Town10HD

---

## Data Flow Overview

```
CARLA Simulator / Bag File
        │
        │  /carla/hero/lidar          (8,000 raw XYZ points @ 20 Hz)
        │  /carla/hero/semantic_lidar (8,000 labelled points @ 20 Hz)
        │  /carla/hero/imu            (400 Hz accelerometer + gyroscope)
        │
        ▼
┌─────────────────┐
│   T1 – Bag      │  Replays recorded sensor data into ROS2 topic bus
└────────┬────────┘
         │ /carla/hero/lidar
         ├──────────────────────────────────────────────────┐
         ▼                                                  ▼
┌─────────────────┐                              ┌──────────────────────┐
│ T2 – Ground     │  RANSAC plane fit            │ T4 – Preprocessor    │
│    Remover      │  splits road from objects    │  adds ring + time    │
└──┬──────────┬───┘                              └──────────┬───────────┘
   │          │                                             │
   │ /ground  │ /objects                                    │ /lidar_iosam
   │          ▼                                             ▼
   │  ┌───────────────┐                         ┌──────────────────────┐
   │  │ T3 – Object   │  DBSCAN clustering      │ T5 – LIO-SAM         │
   │  │  Clusterer    │  + size classification  │  LiDAR-IMU SLAM      │
   │  └───────┬───────┘                         └──────────────────────┘
   │          │ /lidar/clusters/markers                     │
   ▼          ▼                                             ▼
 RViz2   VEH/PED/LRG boxes                         3D Map + Trajectory
```

---

## T1 — Bag Playback

### What it does

Instead of running CARLA live, we replay a pre-recorded `.db3` bag file.
The bag contains all sensor streams frozen in time with original timestamps.

```bash
./play_stack.sh
# internally runs:
ros2 bag play <bag_path> --loop --clock
```

### Why `--clock`?

ROS2 has two clocks:
- **System clock** — real wall-clock time
- **ROS clock** — driven by the `/clock` topic

Without `--clock`, replayed messages carry original timestamps but nodes use the
system clock — causing time skew in synchronised subscribers like
`ApproximateTimeSynchronizer`. With `--clock`, every node reads the same
bag-driven clock.

### What's in the bag

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/carla/hero/lidar` | PointCloud2 | 20 Hz | Raw 32-ch LiDAR (x,y,z,intensity) |
| `/carla/hero/semantic_lidar` | PointCloud2 | 20 Hz | GT labels (x,y,z,ObjTag) |
| `/carla/hero/imu` | Imu | ~20 Hz | Accel + gyro + orientation |
| `/carla/hero/odometry` | Odometry | ~20 Hz | CARLA ground truth pose |
| `/tf` | TFMessage | ~40 Hz | Transform tree |

### Coordinate Frame

Each point is in **sensor frame** — origin at the LiDAR (2.4m above road):

```
Road surface at z_world = 0  →  z_sensor = 0 - 2.4 = -2.4 m
Car roof   at z_world = 1.5  →  z_sensor = 1.5 - 2.4 = -0.9 m
Building   at z_world = 8.0  →  z_sensor = 8.0 - 2.4 = +5.6 m
```

---

## T2 — Ground Remover (RANSAC Plane Fitting)

**Node:** `ground_remover.py`  
**Input:** `/carla/hero/lidar`  
**Output:** `/carla/hero/lidar/ground` + `/carla/hero/lidar/objects`

### Why Not a Simple Z Threshold?

`if z < -1.0: ground` fails when roads slope, the car pitches/rolls, or
raised curbs sit near the threshold. RANSAC finds the actual plane from data.

### RANSAC — Step by Step

```
Repeat N=100 times:
  1. Pick 3 random low-Z points
  2. Compute the plane through them
  3. Count all points within 0.25m of this plane (inliers)
  4. Save if best inlier count so far

→ Return the plane with the most inliers
```

#### Pre-filter: only send road-level candidates to RANSAC

```python
candidate_mask = xyz[:, 2] < -1.0   # only low-Z points
candidates = xyz[candidate_mask]     # ~3,000 pts instead of 8,000
```

#### Fit a plane through 3 random points

Three non-collinear points define a unique plane. Normal via cross product:

```python
p1, p2, p3 = candidates[np.random.choice(n, 3, replace=False)]

v1 = p2 - p1
v2 = p3 - p1
normal = np.cross(v1, v2)
normal /= np.linalg.norm(normal)   # unit normal vector

d = -np.dot(normal, p1)            # plane offset: normal·p + d = 0
```

#### Score: count inliers

```python
distances   = np.abs(candidates @ normal + d)   # distance to plane
inlier_mask = distances < 0.25                  # within 25cm = ground
count       = inlier_mask.sum()
```

#### Typical result

```
Total: 8,082  |  Ground: 6,585 (81.5%)  |  Objects: 1,497 (18.5%)
```

---

## T3 — Object Clusterer (DBSCAN)

**Node:** `object_clusterer.py`  
**Input:** `/carla/hero/lidar/objects` (~1,500 points)  
**Output:** `/lidar/clusters/markers` (3D bounding boxes)

### Why DBSCAN?

We have ~1,500 points from cars, bollards, buildings. We need to group them into
individual objects without knowing how many objects there are in advance.

**DBSCAN parameters:**
- `eps = 1.5m` — two points are "neighbours" if within 1.5m
- `min_samples = 3` — need 3 neighbours to be a "core point"

### How DBSCAN Works

```
For each unvisited point P:
  N = all points within eps=1.5m of P
  if |N| < 3:
    → P is NOISE (label = -1)
  else:
    → P is CORE POINT — start new cluster
    → Recursively add all density-reachable points
```

```python
from sklearn.cluster import DBSCAN

labels = DBSCAN(eps=1.5, min_samples=3).fit_predict(xyz)
# labels: [-1, 0, 0, 1, 1, 1, 2, -1, ...]  (-1 = noise)
```

### Size-Based Classification (Calibrated from Validator)

```python
PEDESTRIAN_MAX = 15    # ≤ 15 pts  → pole/small object  → GREEN
VEHICLE_MAX    = 500   # ≤ 500 pts → vehicle            → BLUE
                       # > 500 pts → large structure     → RED
```

> Calibrated measurement: vehicles produce 59–247 pts at 10–30m,
> and 19–52 pts at 30–50m range.

### Building the Bounding Box

```python
mins   = cluster.min(axis=0)        # corner of box
maxs   = cluster.max(axis=0)
centre = (mins + maxs) / 2.0        # box centre
dims   = np.clip(maxs - mins, 0.1)  # box size (min 10cm to avoid flat boxes)
```

Published as `Marker.CUBE` with text label `VEH (142)` floating above.

---

## T4 — LiDAR Preprocessor (Ring + Time)

**Node:** `lidar_preprocessor.py`  
**Input:** `/carla/hero/lidar` (x, y, z, intensity)  
**Output:** `/carla/hero/lidar_iosam` (x, y, z, intensity, **ring**, **time**)

### Why LIO-SAM Needs These Fields

LIO-SAM projects the cloud into a **32×512 range image**:
- Rows = ring (which beam fired — 0 to 31)
- Columns = horizontal bin (0 to 511)

Without ring, it cannot build this image. Without time, it cannot correct
motion distortion (the car moves while the LiDAR spins).

### Computing Ring from Elevation Angle

The 32 channels fire at fixed elevation angles from -26.8° to +2.0°:

```
ring 0  → -26.8°  (steeply downward — sees road far ahead)
ring 16 → -12.4°  (middle — sees nearby road + low obstacles)
ring 31 → +2.0°   (slightly upward — sees building tops)
```

```python
N_SCAN       = 32
LOWER_FOV    = -26.8        # degrees
CHANNEL_STEP = 28.8 / 31   # = 0.9290° per ring

dist_xy  = np.sqrt(x**2 + y**2)
elev_deg = np.degrees(np.arctan2(z, dist_xy))

ring = np.clip(
    np.round((elev_deg - LOWER_FOV) / CHANNEL_STEP),
    0, 31
).astype(np.uint16)
```

**Example:**
- Point at z=−0.5m, dist=20m → elev = −1.43° → ring = (−1.43 + 26.8) / 0.929 = **27**
- Point at z=−2.4m, dist=1m  → elev = −67.4° → clipped to **ring 0**

### Computing Time from Azimuth

LiDAR does one full 360° rotation in 50ms. A point's azimuth = how far through
the rotation it was captured:

```python
SCAN_PERIOD = 1.0 / 20.0   # 0.05 seconds

azimuth  = np.arctan2(y, x)                    # -π to +π
azimuth  = (azimuth + 2*np.pi) % (2*np.pi)    # normalise to [0, 2π]

time_pts = (azimuth / (2*np.pi)) * SCAN_PERIOD # [0.000, 0.050) seconds
```

**Why this matters for motion distortion:**
If the car moves at 10 m/s, in 50ms it travels 0.5m. Points at opposite sides of
the scan were measured at different positions — the time field lets LIO-SAM
compensate using IMU integration.

### Vectorised Packing (Performance)

```python
dtype = np.dtype([
    ('x',         np.float32),   # 4B
    ('y',         np.float32),   # 4B
    ('z',         np.float32),   # 4B
    ('intensity', np.float32),   # 4B
    ('ring',      np.uint16),    # 2B
    ('_pad',      np.uint16),    # 2B  ← alignment padding
    ('time',      np.float32),   # 4B
])                               # = 24B total

arr = np.empty(n, dtype=dtype)
arr['x'] = x;  arr['y'] = y;  arr['z'] = z
arr['intensity'] = intensity
arr['ring'] = ring;  arr['time'] = time_pts
out.data = arr.tobytes()         # C-speed — <1ms for 16,000 points
```

---

## T5 — LIO-SAM (LiDAR Inertial Odometry via Smoothing and Mapping)

**Input:** `/carla/hero/lidar_iosam` + `/carla/hero/imu`  
**Output:** 3D map + ego trajectory

### The Core Idea

LiDAR gives geometry (where things are), IMU gives dynamics (how fast we move).
Neither alone is enough:
- LiDAR alone: 20Hz, no intra-scan motion compensation, drifts without loop closure
- IMU alone: 400Hz, but drift accumulates quadratically — 10cm/s² error → 10m in 10s

### Module 1 — IMU Pre-Integration

Between each pair of LiDAR scans (50ms), IMU fires ~10 times.
These readings are integrated to predict the next pose:

```
Δp = ∫∫ a dt²    (position change from accelerometer)
Δv = ∫  a dt     (velocity change)
Δq = ∫  ω dt     (rotation from gyroscope as quaternion)
```

This pose prediction seeds the LiDAR matching step.

### Module 2 — Feature Extraction (Range Image → 300 features)

The 32×512 range image is analysed for:

**Edge features** (sharp corners — lamp posts, building edges):
```
curvature = |Σ(range_i − range_centre)| for neighbours i
if curvature > 1.0 → edge feature
```

**Surface features** (flat areas — road, building walls):
```
if curvature < 0.1 → surface feature
```

~300 features per scan vs 16,000 raw points = 50× speedup for matching.

### Module 3 — Scan-to-Map ICP Matching

Each feature is matched to the accumulated map:
- **Edge feature** → minimise distance to nearest LINE in map
- **Surface feature** → minimise distance to nearest PLANE in map

Result: 6-DOF pose (x, y, z, roll, pitch, yaw) for the current scan.

### Module 4 — GTSAM Factor Graph Back-End

All pose estimates are nodes in a probabilistic factor graph:

```
[Prior] ─ Pose₀ ─[IMU]─ Pose₁ ─[IMU]─ Pose₂ ─ ... ─ PoseN
              ↕LiDAR        ↕LiDAR         ↕LiDAR
         [Loop closure: PoseN ↔ Pose₅ if revisiting same place]
```

GTSAM jointly optimises ALL poses to maximise consistency — when a loop closure
is added, all historical poses are corrected simultaneously.

### CARLA Configuration

```yaml
sensor: velodyne       # our preprocessor outputs uint16 ring + float32 time
N_SCAN: 32
Horizon_SCAN: 512      # nearest 2^n ≥ 500 pts/ring
lidarMaxRange: 50.0    # matches CARLA sensor config
extrinsicTrans: [2.0, 0.0, -0.4]   # IMU is 2m fwd, 0.4m lower than LiDAR
extrinsicRot: [1,0,0, 0,1,0, 0,0,1]  # identity — same orientation
```

---

## Full Pipeline in Numbers

| Stage | Input | Output | Rate |
|---|---|---|---|
| Bag | — | 8,000 raw pts | 20 Hz |
| Ground Remover | 8,000 pts | 6,600 ground + 1,400 objects | 20 Hz |
| Object Clusterer | 1,400 pts | 33–44 clusters, 2–4 VEH boxes | 20 Hz |
| Preprocessor | 8,000 pts | 8,000 pts + ring + time | 20 Hz |
| LIO-SAM | 8,000 XYZIRT + IMU | pose + 3D map | 20 Hz + 400 Hz |
