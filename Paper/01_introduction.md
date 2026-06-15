# Section 1: Introduction

---

## 1. Introduction

The development of autonomous vehicles represents one of the most significant technological challenges of the modern era. Enabling a machine to navigate complex, dynamic urban environments safely requires the ability to solve two fundamental problems simultaneously: understanding what is in the surrounding environment, and knowing precisely where within that environment the vehicle is located. Solving these problems in real time, without relying on external infrastructure such as GPS — which suffers from multipath interference in urban canyons and is unavailable in tunnel or underground settings — is a core requirement of any production-ready autonomous driving system [1].

Light Detection and Ranging (LiDAR) sensors have emerged as the primary modality for autonomous vehicle perception. Unlike cameras, which are sensitive to lighting conditions, and radar, which provides limited spatial resolution, LiDAR directly measures the 3D geometry of the environment by emitting laser pulses and recording the time of flight of returning reflections. Modern rotating LiDAR units produce dense 360° point clouds at rates of 10–20 Hz, capturing detailed structural information about roads, vehicles, pedestrians, and infrastructure at ranges of up to 100 metres. However, raw point cloud data presents significant challenges: it must be filtered to isolate objects of interest from irrelevant surface returns, segmented to identify individual entities, and fused with other sensor modalities to estimate vehicle motion without drift.

This project addresses these challenges by designing, implementing, and evaluating a complete five-stage LiDAR processing and localisation pipeline. The pipeline was developed using the CARLA autonomous driving simulator [2] — a high-fidelity, open-source platform capable of rendering realistic urban environments with configurable sensor suites — and the Robot Operating System 2 (ROS2) Humble framework [3] for real-time inter-node communication. The five stages progress from raw sensor characterisation through ground removal, object clustering, semantic validation, and finally tightly-coupled LiDAR-Inertial Simultaneous Localisation and Mapping (SLAM) using the LIO-SAM framework [4]. A primary technical contribution of this work is a format-bridging preprocessing node that enables LIO-SAM to consume CARLA's raw LiDAR output without modification of the upstream SLAM framework — a non-trivial integration challenge arising from incompatible point cloud field specifications between the two systems.

The performance of the complete system was quantitatively evaluated using the Absolute Trajectory Error (ATE) metric, comparing estimated vehicle trajectories against CARLA's ground-truth odometry. A controlled experiment comparing scenarios with and without dynamic non-player character (NPC) vehicles provides novel empirical evidence of the impact of moving objects on LiDAR-based scan matching, yielding a measurable and reproducible degradation factor that has practical implications for the design of real-world SLAM pipelines.

The remainder of this paper is organised as follows. Section 2 provides the necessary background on LiDAR sensors, point cloud processing algorithms, and SLAM. Section 3 describes the overall system architecture and software stack. Sections 4 through 8 present each of the five pipeline stages in detail, including mathematical formulations, implementation decisions, and results. Section 9 presents the trajectory accuracy evaluation and the NPC impact experiment. Section 10 discusses key findings and limitations. Section 11 concludes the paper and outlines directions for future work.

---

> **📌 Figure to add here:**
> **Figure 1** — A screenshot of the CARLA Town10HD environment showing the ego vehicle with the LiDAR point cloud visible in RViz2. Caption: *"The CARLA simulation environment (Town10HD) used for all experiments, showing the ego vehicle (hero) with 32-channel LiDAR point cloud overlay in RViz2."*
>
> **Figure 2** — The pipeline flow diagram (5 boxes connected by arrows: Raw LiDAR → Ground Removal → Object Clustering → Semantic Validation → LIO-SAM). Caption: *"Overview of the proposed five-stage LiDAR perception and mapping pipeline."*

---

*Word count: ~500 words*
