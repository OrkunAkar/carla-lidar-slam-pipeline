# Section 5: Ground Plane Removal

---

## 5. Ground Plane Removal

### 5.1 Motivation

Before objects can be detected, the road surface must be removed from the point cloud. Ground points constitute the majority of each scan (~81%) and would otherwise dominate clustering, causing road surface regions to be merged with nearby obstacles or forming large spurious clusters.

---

### 5.2 Implementation

Ground removal is implemented in `ground_remover.py` as a ROS2 node subscribing to `/carla/hero/lidar` and publishing two output topics: `/lidar/ground` and `/lidar/objects`.

**Pre-filter:** Only points with z < −1.0 m (in sensor frame) are considered as ground candidates. This immediately discards vehicle bodies, pedestrians, and infrastructure, reducing the RANSAC search space from ~8,000 to ~2,000 points and improving runtime.

**RANSAC plane fitting:**

```python
for _ in range(N_ITERATIONS):          # N = 100
    sample = random.sample(candidates, 3)
    plane = fit_plane(sample)           # cross product of two edge vectors
    inliers = [p for p in candidates
               if abs(plane.distance(p)) < THRESHOLD]   # τ = 0.25 m
    if len(inliers) > best_count:
        best_plane, best_count = plane, len(inliers)
```

**Classification:** All 8,000 points in the full scan are then tested against the best-fit plane:
- Distance ≤ 0.25 m → **ground**
- Distance > 0.25 m → **object**

> **📌 Code snippet to add:**
> Show the 6-line `fit_plane()` function from `ground_remover.py` — the cross-product plane normal calculation. Short, readable, and demonstrates the maths directly.

---

### 5.3 Key Parameters

| Parameter | Value | Rationale |
|---|---|---|
| Pre-filter threshold | z < −1.0 m | Keeps only road-level candidates |
| RANSAC iterations | 100 | P(success) ≈ 1.000 at 80% inlier ratio |
| Inlier distance (τ) | 0.25 m | Tolerates road camber and sensor noise |

---

### 5.4 Results

Across 1,660 scans in the test recording, RANSAC ground removal consistently classified approximately **81.5% of points as ground** (≈6,560 pts) and **18.5% as objects** (≈1,520 pts), with a standard deviation of less than 1% across scans — confirming stable, repeatable segmentation.

| Metric | Value |
|---|---|
| Average ground points/scan | ~6,560 (81.5%) |
| Average object points/scan | ~1,520 (18.5%) |
| Scan-to-scan variation | < 1% |
| Processing rate | 20 Hz (real-time) |

> **📌 Figure to add:**
> **Figure 9** — Two side-by-side RViz2 screenshots: left showing the raw point cloud, right showing ground points in green and object points in red/orange after RANSAC. This is the single most visually impactful figure in the paper — definitely include it.

---

*Word count: ~350 words*
