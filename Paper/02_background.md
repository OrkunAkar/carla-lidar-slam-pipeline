# Section 2: Background

---

## 2. Background

This section provides the technical context necessary to understand the design decisions made in this project. It covers the operating principles of LiDAR sensors, the algorithms used for point cloud segmentation and clustering, the fundamentals of SLAM, and the simulation and middleware tools employed.

---

### 2.1 LiDAR Sensors

A Light Detection and Ranging (LiDAR) sensor emits short pulses of laser light and measures the time elapsed before each pulse returns after reflecting off a surface. Given the known speed of light (*c* = 3×10⁸ m/s), the distance to the reflecting surface is calculated as:

```
d = (c × t) / 2
```

where *t* is the round-trip travel time of the pulse. Modern automotive LiDAR units use multiple laser emitters arranged at fixed vertical angles — called **channels** or **rings** — that rotate continuously to produce a full 360° scan. A 32-channel unit, such as the one simulated in this project, fires 32 laser beams simultaneously, each at a distinct elevation angle ranging from −26.8° to +2.0°. At 20 rotations per second, this produces a point cloud of approximately 16,000 points per scan in open environments (reduced in practice by self-occlusion from the vehicle body).

Each point in the resulting cloud is described by its Cartesian coordinates (x, y, z) in the sensor frame and an intensity value proportional to the reflectivity of the surface at that point.

> **📌 Figure to add:**
> **Figure 3** — Diagram of a rotating multi-channel LiDAR showing the 32 elevation angles (rings) as horizontal scan lines at different vertical angles. Can be a simple hand-drawn or found from a LiDAR manufacturer diagram (Velodyne HDL-32E datasheet is publicly available).

---

### 2.2 Point Cloud Coordinate Frame

Points measured by the LiDAR are expressed in the **sensor frame**: x points forward, y points left, z points upward. In this project, the LiDAR is mounted at the rooftop of the ego vehicle, 2.4 m above the road surface. Ground returns therefore appear at approximately z = −2.4 m in sensor coordinates, while vehicle rooftops appear near z = 0 to +1 m.

---

### 2.3 Random Sample Consensus (RANSAC)

RANSAC is a robust model-fitting algorithm introduced by Fischler and Bolles (1981) [5] designed to fit a mathematical model to data that contains a large fraction of outliers. For ground plane removal, the model is a 3D plane defined by:

```
ax + by + cz + d = 0
```

where (a, b, c) is the unit normal vector of the plane and d is the distance from the origin.

**Algorithm:**
1. Randomly sample 3 points from the candidate set
2. Fit a plane through the 3 points (using the cross product of two edge vectors)
3. Count **inliers**: all points within distance threshold τ of the plane
4. Repeat for *N* iterations; keep the plane with the most inliers
5. Refit the plane using all inliers for improved accuracy

**Convergence probability:** The probability that at least one of *N* iterations produces a sample containing only inliers is:

```
P = 1 − (1 − w^s)^N
```

where *w* is the inlier ratio and *s* is the sample size (3 for plane fitting). With *w* = 0.80, *s* = 3, *N* = 100: P ≈ 1.000.

> **📌 Formula to display as a numbered equation in your paper:**
> Display the plane equation and P formula as proper numbered equations (Eq. 1, Eq. 2).

---

### 2.4 DBSCAN Clustering

Density-Based Spatial Clustering of Applications with Noise (DBSCAN) is a clustering algorithm proposed by Ester et al. (1996) [6] that groups points based on local density rather than assuming a fixed number of clusters. It is well-suited to point cloud data where the number of objects is unknown and objects have irregular shapes.

**Key parameters:**
- **ε (epsilon):** the neighbourhood radius — two points are neighbours if their Euclidean distance is ≤ ε
- **MinPts:** the minimum number of neighbours required for a point to be classified as a **core point**

**Definitions:**
- A point *p* is a **core point** if at least MinPts points lie within distance ε of *p*
- A point *q* is **directly density-reachable** from *p* if *q* is within ε of core point *p*
- A point is classified as **noise** if it is not density-reachable from any core point

**Algorithm:**
1. For each unvisited point *p*, retrieve all points within ε (the ε-neighbourhood)
2. If fewer than MinPts neighbours → mark *p* as noise
3. Otherwise → start a new cluster, recursively add all density-reachable points

DBSCAN naturally handles noise (unclassified points) and does not require the number of clusters to be specified in advance — a significant advantage over k-means in autonomous driving applications.

> **📌 Figure to add:**
> **Figure 4** — A 2D illustration of DBSCAN showing core points (large circles), border points, and noise points with ε-radius circles drawn. This is a standard textbook figure; find a clean version or recreate it.

---

### 2.5 Simultaneous Localisation and Mapping (SLAM)

SLAM is the computational problem of constructing a map of an unknown environment while simultaneously tracking the agent's location within that map. In a LiDAR-based SLAM system, the vehicle's pose (6-DOF: position x, y, z and orientation roll, pitch, yaw) is estimated by finding the rigid-body transformation between consecutive point cloud scans:

```
T* = argmin_T  Σᵢ ‖q_i − T × p_i‖²
```

where {p_i} are points from the current scan and {q_i} are their corresponding points in the map.

**The core challenge** is that purely sequential scan-matching (odometry) accumulates drift over time — small errors compound with each scan pair. Global consistency is maintained through **loop closure**: when the vehicle revisits a previously mapped area, a corrective constraint is added to a factor graph and the full trajectory is re-optimised.

To evaluate this drift quantitatively, the estimated trajectory can be compared against a high-accuracy reference trajectory. This is typically done using the **Absolute Trajectory Error (ATE)**, which measures the absolute distance between the estimated and ground-truth poses after aligning their coordinate frames.

---

### 2.6 LIO-SAM: Tightly-Coupled LiDAR-Inertial Odometry

LIO-SAM (Shan et al., 2020) [4] is an open-source SLAM framework that tightly couples LiDAR point clouds with IMU (Inertial Measurement Unit) data in a smoothing-and-mapping factor graph. It addresses two key limitations of LiDAR-only SLAM:

1. **Motion distortion correction:** A rotating LiDAR captures points over 50ms while the vehicle moves. Without correction, the resulting scan is geometrically inconsistent. LIO-SAM uses IMU integration to de-skew each scan by transforming all points to the pose at the scan's start time.

2. **Scan matching initialisation:** Rather than searching blindly for the correct pose alignment, LIO-SAM uses IMU-propagated poses as an accurate initial estimate for ICP (Iterative Closest Point) optimisation, dramatically reducing convergence time.

The system processes point clouds as **range images** (structured 2D grids, one row per LiDAR ring) and extracts two feature types: **edge features** (sharp depth discontinuities, high curvature) and **surface features** (locally flat regions, low curvature). These ~300 features per scan are matched against an accumulated map using point-to-edge and point-to-plane distance metrics. All pose estimates are maintained in a **GTSAM factor graph** and optimised incrementally using iSAM2.

> **📌 Figure to add:**
> **Figure 5** — The LIO-SAM architecture diagram (4 modules: Image Projection → Feature Extraction → Scan Matching → Map Optimisation). A simplified version of Figure 2 from the original LIO-SAM paper (Shan et al., 2020) — cite it and redraw it.

---

### 2.7 CARLA Autonomous Driving Simulator

CARLA (Car Learning to Act) [2] is an open-source, Unreal Engine-based simulator designed specifically for autonomous driving research. It provides:
- Realistic urban maps (Town01–Town10) with buildings, roads, traffic signs, and pedestrian paths
- A configurable sensor suite including cameras, LiDAR, radar, GNSS, and IMU
- A Python and ROS2 API for programmatic control of vehicles and sensors
- Non-Player Character (NPC) vehicles and pedestrians with realistic behaviour

A critical feature of CARLA for algorithm development is its ability to output **exact, noise-free ground-truth state information** (ego-vehicle velocity, orientation, and absolute 3D position). This provides a perfect reference trajectory (ground-truth odometry) for benchmarking the localization accuracy of SLAM frameworks without the sensor noise or dropouts inherent in real-world GNSS/INS reference systems.

In this project, **Town10HD** (a high-detail urban district with dense building coverage) was used as the test environment.

---

### 2.8 Robot Operating System 2 (ROS2)

ROS2 [3] is an open-source middleware framework for robotics and autonomous systems. It provides a publish-subscribe messaging system in which independent **nodes** (processes) communicate by publishing and subscribing to named **topics**. In this project, each pipeline stage is implemented as a separate ROS2 node, enabling modular development and real-time data flow between stages at configurable message rates.

| ROS2 Message Type | Used for |
|---|---|
| `sensor_msgs/PointCloud2` | LiDAR point clouds |
| `sensor_msgs/Imu` | IMU acceleration and angular velocity |
| `nav_msgs/Odometry` | Ground-truth vehicle pose |
| `visualization_msgs/MarkerArray` | 3D bounding box visualisation |

---

*Word count: ~900 words*

> **📌 Notes for this section:**
> - Add equation numbers (Eq. 1, 2, 3…) for the plane equation, convergence probability, and SLAM cost function
> - The DBSCAN 2D illustration (Figure 4) is easy to draw by hand and photograph, or recreate in any diagram tool
> - Cite all 4 references [2][3][4][5][6] with their full bibliographic details in your References section
