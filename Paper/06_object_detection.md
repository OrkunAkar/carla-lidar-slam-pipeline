# Section 6: Object Detection and Classification

---

## 6. Object Detection and Classification

### 6.1 Overview

After ground removal, the remaining ~1,520 object points per scan are passed to `object_clusterer.py`, which groups them into individual objects using DBSCAN and classifies each cluster by its point count. The node subscribes to `/lidar/objects` and publishes 3D bounding box markers to `/clusters/markers` for RViz2 visualisation.

---

### 6.2 DBSCAN Configuration

DBSCAN was applied in 3D (x, y, z coordinates) using scikit-learn:

```python
from sklearn.cluster import DBSCAN

labels = DBSCAN(eps=1.5, min_samples=3).fit_predict(object_points)
```

The parameters were determined empirically through iterative testing with the semantic validator (Section 7):

| Parameter | Initial | Final | Reason for change |
|---|---|---|---|
| ε (eps) | 1.0 m | **1.5 m** | Prevented vehicle fragmentation at range > 30m |
| min_samples | 5 | **3** | Far vehicles have only 2–4 intra-cluster neighbours |

> **📌 Figure to add:**
> **Figure 10** — RViz2 screenshot showing DBSCAN bounding boxes overlaid on the object point cloud. The bounding boxes should be coloured by class (blue = PED, orange = VEH, red = LRG). Run the bag + `ground_remover` + `object_clusterer` together, add `/clusters/markers` as a MarkerArray display.

---

### 6.3 Size-Based Classification

Each cluster is classified solely by its point count, which correlates with object size and distance from the sensor:

| Class | Point Range | Colour | Typical object |
|---|---|---|---|
| PED | 5 – 15 pts | Blue | Pedestrian, thin pole |
| VEH | 16 – 500 pts | Orange | Car, van, truck |
| LRG | > 500 pts | Red | Building wall, large structure |
| Noise | < 5 pts | (discarded) | Isolated returns |

A **3D axis-aligned bounding box (AABB)** is computed for each cluster from the min/max coordinates along each axis:

```
width  = x_max − x_min
depth  = y_max − y_min
height = z_max − z_min
```

**Limitation:** Geometry-only classification cannot distinguish a thin road-sign pole (PED-sized) from an actual pedestrian. Semantic labels or shape descriptors would be required for higher accuracy — identified as future work.

---

### 6.4 Results

| Metric | Value |
|---|---|
| Average clusters per scan | 33 – 44 |
| VEH-class detections per scan | 2 – 4 |
| Maximum detection range | ~50 m |
| Processing rate | 20 Hz |
| Min points for detection | 5 pts |

> **📌 Figure to add:**
> **Figure 11** — A close-up RViz2 screenshot showing a VEH-class bounding box around a vehicle cluster at ~15m range, with the point count visible. You can add a `Text` marker or just annotate the screenshot image afterwards in any image editor.

---

*Word count: ~340 words*
