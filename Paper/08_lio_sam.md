# Section 8: Tightly-Coupled SLAM Integration (LIO-SAM)

---

## 8. Tightly-Coupled SLAM Integration (LIO-SAM)

### 8.1 Integration Challenge

Although LIO-SAM is designed to be sensor-agnostic, it assumes that incoming point clouds conform strictly to the Velodyne XYZIRT format. CARLA's ROS2 bridge, however, only publishes standard XYZI (X, Y, Z, Intensity) fields. Integrating LIO-SAM directly with CARLA's raw output causes initialization failures due to two missing metadata fields:
1. **Ring ID (`ring`):** The vertical channel number (0 to 31) that produced the point. LIO-SAM uses this to organize points into a structured range image for fast column-by-column curvature calculations.
2. **Time Offset (`time`):** The relative timestamp (in seconds) of each point relative to the start of the scan. LIO-SAM uses this to correct motion skewing by integrating IMU measurements at the exact instant each point was fired.

---

### 8.2 Mathematical Derivations for Missing Fields

To bridge this compatibility gap without altering the LIO-SAM source code, a custom `lidar_preprocessor.py` node was developed to calculate these fields geometrically in real time.

#### 1. Ring Channel ID Calculation
The vertical elevation angle $\theta$ of any point $p = (x, y, z)$ in the sensor frame is defined by:
$$\theta = \arctan2\left(z, \sqrt{x^2 + y^2}\right)$$

Given the lower elevation limit of the LiDAR $\theta_{min} = -26.8^\circ$ ($-0.4678\text{ rad}$) and the vertical step resolution $\Delta\theta = 0.929^\circ$ ($0.0162\text{ rad}$), the corresponding ring index $R \in [0, 31]$ is derived as:
$$R = \text{round}\left( \frac{\theta - \theta_{min}}{\Delta\theta} \right)$$

#### 2. Intra-Scan Time Offset Calculation
The sensor rotates clockwise around the Z-axis, with a full rotation taking $T_{scan} = 0.05\text{ s}$ (20 Hz). The azimuth angle $\phi \in [-\pi, \pi]$ of a point is calculated as:
$$\phi = \arctan2(y, x)$$

Normalizing $\phi$ to the interval $[0, 2\pi]$ yields the relative phase of rotation. The relative time offset $t_{offset} \in [0, 0.05]$ is therefore:
$$t_{offset} = \left( \frac{\phi_{normalized}}{2\pi} \right) \times T_{scan}$$

---

### 8.3 Vectorized Python Implementation

To prevent this preprocessing step from introducing latency, a vectorized NumPy implementation was used instead of a standard iterative Python loop. The math operations are performed over the entire point matrix simultaneously:

```python
# Extract coordinates as vectors
x, y, z = points[:, 0], points[:, 1], points[:, 2]

# Compute rings
elevations = np.arctan2(z, np.sqrt(x**2 + y**2))
rings = np.round((elevations - ELEV_MIN_RAD) / ELEV_STEP_RAD).astype(np.uint16)
rings = np.clip(rings, 0, N_SCAN - 1)

# Compute time offsets
azimuths = np.arctan2(y, x)
azimuths_normalized = np.mod(azimuths + np.pi, 2 * np.pi)
times = (azimuths_normalized / (2 * np.pi)) * SCAN_PERIOD
```

This vectorization enables the processing of 16,000 points in **under 1.0 ms**, maintaining a negligible footprint on the real-time ROS2 executor.

---

### 8.4 LIO-SAM Parameters

Key parameters configured in `params_carla.yaml` to match the simulated vehicle setup:

| Parameter | Value | Description |
|---|---|---|
| `sensor` | `velodyne` | Activates XYZIRT channel decoding |
| `N_SCAN` | 32 | Configures vertical ring count |
| `Horizon_SCAN` | 512 | Nearest $2^n \ge 500$ points per ring |
| `extrinsicTrans` | `[2.0, 0.0, -0.4]` | Spatial translation vector from IMU to LiDAR |
| `extrinsicRot` | Identity matrix | Orientation alignment between IMU and LiDAR |

---

### 8.5 Generated Map Representation

The output of LIO-SAM consists of a globally optimized factor graph, real-time keyframe trajectories, and point clouds. The system successfully mapped a $214\text{ m} \times 219\text{ m}$ area of Town10HD.

> **📌 Figure to add:**
> **Figure 13** — Bird's-Eye View (BEV) height-colored rendering of the generated `GlobalMap.pcd` with the trajectory overlay (red line). Save the output of `bev_map.py` (which produces a beautiful top-down layout) to the paper folder.

---

*Word count: ~530 words*
