# Section 11: Conclusion and Future Work

---

## 11. Conclusion and Future Work

### 11.1 Conclusion

This project successfully designed, implemented, and validated a complete modular LiDAR-based perception and localisation pipeline for autonomous vehicles within the CARLA simulation environment. Using ROS2 Humble as the communication middleware, the pipeline processed data from a rooftop-mounted 32-channel rotating LiDAR and IMU at 20 Hz in real time. 

The custom data analysis node (`lidar_analyzer.py`) revealed a 50% point count reduction caused by self-occlusion from the ego-vehicle body, which informed all downstream stage designs. A RANSAC-based ground plane removal algorithm successfully isolated the road surface, maintaining a stable 81.5% classification rate scan-to-scan. Above-ground points were clustered using DBSCAN with calibrated parameters ($\epsilon=1.5\text{ m}$, $\text{MinPts}=3$), and classified into size-based categories (PED, VEH, LRG). This perception layer was quantitatively validated against CARLA's semantic ground truth, achieving a 100% vehicle detection rate across all targets.

For localisation, the LIO-SAM framework was integrated using a custom format-bridging preprocessor. In static testing, this setup achieved a trajectory error of **2.54 m ATE RMSE (0.42% relative error)** over a 597.4 m route, demonstrating performance comparable to real-world datasets. A controlled comparison in a dynamic environment highlighted that while the pipeline is robust to moving traffic, a localized spawning collision corrupted state estimation due to IMU saturation and close-range occlusion.

In summary, this work provides a functional, modular, and quantitatively validated foundation for autonomous driving research, proving that real-time sensor processing and state estimation can be integrated using open-source tools.

---

### 11.2 Future Work

To address the limitations identified in the current implementation, several future research directions are proposed:

1. **Semantic Segment-Driven Clustering:** Integrating semantic segmentation models (such as SalsaNext or PointNet++) into the perception front-end would allow the system to filter out dynamic objects before scan-matching, improving localization robustness.
2. **GNSS/GPS Fusion:** Incorporating absolute positioning updates (via a Kalman Filter or factor graph GNSS factors) would bound the dead-reckoning drift of LIO-SAM, enabling long-term localization consistency over extended routes.
3. **Collision Recovery and Re-localisation:** Developing automated recovery routines — such as detecting IMU spikes and falling back to Monte Carlo Localisation (MCL) or global map-based registration — would prevent catastrophic failures during high-g collisions.
4. **Physical Platform Deployment:** Testing the pipeline on a real-world vehicle platform with physical hardware sensors to evaluate the impact of real sensor noise, uneven terrain, and environmental variability.

---

*Word count: ~390 words*
