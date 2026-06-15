# Section 10: Discussion

---

## 10. Discussion

### 10.1 Key Findings and Successes

The modular design of the proposed pipeline succeeded in meeting the real-time requirements of the perception and localization stack. The primary accomplishments are:

1. **Format Compatibility Solution:** The vectorized `lidar_preprocessor.py` node bridged the gap between CARLA's raw XYZI output and LIO-SAM's XYZIRT requirements. Performing mathematical vertical elevation and horizontal phase calculations at runtime without latency (<1.0 ms) proves that Python code, when vectorized with NumPy, can meet real-time robotic loop requirements (20 Hz).
2. **Localization Performance:** In the absence of collision anomalies, LIO-SAM achieved an **ATE RMSE of 2.54 m (0.42% relative error)** over a 597.4 m path. This result places the simulated implementation on par with real-world state-of-the-art results (such as KITTI dataset benchmarks), confirming the validity of the parameters configured in `params_carla.yaml`.
3. **Robustness to Dynamic Traffic:** LIO-SAM showed high robustness to dynamic traffic scenes where multiple NPC vehicles moved within the LiDAR's field of view. The feature extraction mechanism successfully ignored moving points by selecting distinct high-curvature edges and low-curvature flat surfaces, which are naturally dominated by static buildings and road structures.

---

### 10.2 Pipeline Limitations

Despite its successes, several structural limitations were identified during development and testing:

1. **Vulnerability to Extreme Occlusions and Collisions:** As shown by the first experiment, physical collisions (such as the NPC spawning anomaly) saturate the IMU and block the LiDAR's view. Tightly-coupled systems lack recovery routines when both sensor streams are simultaneously corrupted.
2. **Pedestrian Classification Ambiguity:** The geometry-only classifier (DBSCAN + point count bounding box sizing) cannot distinguish between small static infrastructure elements (traffic cones, poles, signs) and dynamic pedestrians due to similar size characteristics.
3. **Pure Dead-Reckoning Drift:** Since the system operates without global positioning corrections (GPS/GNSS), the trajectory estimation relies entirely on local relative constraints. Although drift was low (0.42%), it will continue to accumulate over longer distances.

---

### 10.3 Real-World Transferability

Deploying this pipeline onto a physical autonomous vehicle would require addressing several simulation gaps:

- **IMU Noise Modeling:** CARLA's simulated IMU operates with near-ideal parameters. In a real vehicle, thermal drift, bias instability, and engine vibration introduce high-frequency noise that requires careful sensor calibration and online bias estimation.
- **Sensor Synchronization:** In simulation, messages are generated with perfect digital timestamps. In a physical stack, hardware synchronization (e.g., PTP/PPS triggering) is required to align the IMU and LiDAR capture times.
- **LiDAR Reflection Characteristics:** Real-world LiDAR point clouds suffer from dropouts due to absorbative surfaces (black paint, wet roads) and false reflections, requiring more robust filtering than simple plane-fitting.

---

*Word count: ~440 words*
