# Section 7: Semantic Ground Truth Validation

---

## 7. Semantic Ground Truth Validation

### 7.1 Purpose

DBSCAN clustering produces unlabelled geometry-based detections with no inherent accuracy metric. To quantify detection performance, a validation node (`detector_validator.py`) was developed to compare raw LiDAR coverage against ground-truth object positions provided by CARLA's **semantic LiDAR** sensor — a special sensor that annotates each point with the class and instance ID of the object it hit.

---

### 7.2 Ground Truth Source

CARLA's semantic LiDAR publishes `sensor_msgs/PointCloud2` messages with additional fields per point:

| Field | Type | Content |
|---|---|---|
| `ObjTag` | uint32 | Object class (10 = vehicle, 4 = pedestrian, 7 = road) |
| `ObjIdx` | uint32 | Unique instance ID per object |

> **⚠️ Implementation note:** CARLA 0.9.15 uses PascalCase field names (`ObjTag`, `ObjIdx`) — not the snake_case (`object_tag`, `object_idx`) documented in older versions. This caused a silent parsing failure that produced empty validation results until identified and corrected.

---

### 7.3 Validation Methodology

Rather than attempting to match DBSCAN clusters one-to-one with semantic objects (fragile, sensitive to cluster fragmentation), a **coverage-based** approach was used:

1. Extract all semantic LiDAR points with `ObjTag = 10` (vehicles)
2. Group by `ObjIdx` to get per-vehicle point sets
3. For each vehicle: compute its centroid from semantic points
4. Count raw LiDAR points within **5 m** of that centroid
5. If count > 0 → vehicle is **detected**; otherwise → **missed**

This method is robust to DBSCAN parameter variation — it checks whether the sensor physically sees the vehicle, independent of clustering quality.

---

### 7.4 Results

| Metric | Value |
|---|---|
| Vehicles in scene | 6 |
| Vehicles detected (coverage > 0) | **6 / 6 (100%)** |
| Average raw points per vehicle | 145.7 pts |
| Minimum raw points (farthest vehicle) | 59 pts |
| Validation rate | 20 Hz |

All six in-range vehicles were covered by raw LiDAR returns, confirming that the ground removal stage does not discard vehicle body points and that DBSCAN operates on complete object representations.

> **📌 Figure to add:**
> **Figure 12** — A diagram or annotated RViz2 screenshot showing the 5m coverage radius circles drawn around each GT vehicle centroid, with raw LiDAR points visible inside them. Alternatively, a simple bar chart showing "points per vehicle" for all 6 vehicles works well as a compact figure.

---

### 7.5 Limitations

The semantic validator confirms **raw LiDAR coverage** but does not verify that each vehicle was correctly clustered as a single entity. A vehicle split into two DBSCAN clusters would still register as detected. Full cluster-matching evaluation (precision/recall at the cluster level) is identified as future work.

---

*Word count: ~370 words*
