# Section 4: Sensor Setup and Data Understanding

---

## 4. Sensor Setup and Data Understanding

### 4.1 LiDAR Configuration

The 32-channel LiDAR sensor was configured with the following parameters in CARLA's sensor definition file:

| Parameter | Value |
|---|---|
| Channels (rings) | 32 |
| Range | 50 m |
| Points per second | 320,000 |
| Rotation frequency | 20 Hz |
| Upper FOV | +2.0° |
| Lower FOV | −26.8° |
| Sensor height (above road) | 2.4 m |

Each scan covers a full 360° horizontal field of view, with 32 laser beams spread uniformly between −26.8° and +2.0° in elevation. The theoretical point count per scan is:

```
Points/scan = Points/sec ÷ Rotation frequency = 320,000 ÷ 20 = 16,000 pts
```

---

### 4.2 Raw Data Analysis

A diagnostic node (`lidar_analyzer.py`) was written to characterise the actual sensor output from recorded bag data. The results revealed a systematic discrepancy from the theoretical point count:

| Statistic | Value |
|---|---|
| Theoretical points/scan | 16,000 |
| Observed points/scan | ~8,000 |
| Reduction factor | ~50% |
| Z range (sensor frame) | −2.4 m to +1.7 m |
| Typical ground return Z | −2.3 m to −2.0 m |
| Typical vehicle return Z | −1.5 m to +0.5 m |

The 50% reduction in point count is caused by **self-occlusion** from the vehicle's body: the lower half of the LiDAR's field of view (approximately rings 0–15, pointing downward at more than −13°) is blocked by the car roof, bonnet, and boot. This finding directly influenced the choice of ground removal threshold and the DBSCAN point count thresholds used in later stages.

> **📌 Figure to add:**
> **Figure 8** — A raw RViz2 screenshot of the point cloud from the sensor's perspective, showing the "hollow" centre where the car body blocks the lower rings. Take this screenshot while the bag is playing with Fixed Frame set to `hero/lidar`.

---

### 4.3 PointCloud2 Binary Format

The ROS2 `sensor_msgs/PointCloud2` message stores point data as a flat byte array. Each point in the raw CARLA LiDAR output occupies **16 bytes**:

| Field | Type | Bytes | Offset |
|---|---|---|---|
| x | float32 | 4 | 0 |
| y | float32 | 4 | 4 |
| z | float32 | 4 | 8 |
| intensity | float32 | 4 | 12 |

Understanding this binary layout was essential for writing the `lidar_preprocessor` node (Section 8), which must parse and repack each point with additional fields required by LIO-SAM.

---

### 4.4 Coordinate Frame Convention

All point coordinates are expressed in the **sensor (LiDAR) frame**:
- **+x** → forward (vehicle heading direction)
- **+y** → left
- **+z** → upward

Since the sensor is 2.4 m above the road surface, ground returns appear at z ≈ −2.4 m. This value serves as the starting point for the ground removal pre-filter described in Section 5.

---

*Word count: ~370 words*

> **📌 Code snippet to add (optional):**
> Show 3–4 lines from `lidar_analyzer.py` that compute the Z range statistics — demonstrates practical implementation of the diagnostic step.
