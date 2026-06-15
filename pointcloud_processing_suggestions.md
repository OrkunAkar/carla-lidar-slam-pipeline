# 🟦 Point Cloud Processing — Suggestions & Guide
> Based on the sensor suite in `car_definition_file.json`

---

## Your LiDAR Setup

Two LiDAR sensors are already configured on the vehicle roof (z = 2.4m):

| Sensor | ROS2 Topic | Data |
|---|---|---|
| Ray-cast LiDAR | `/carla/hero/lidar` | `PointCloud2` — X, Y, Z, Intensity |
| Semantic LiDAR | `/carla/hero/semantic_lidar` | `PointCloud2` — X, Y, Z, object tag, actor ID |

**Specs:** 32 channels · 50m range · 320,000 pts/sec · 20 Hz rotation

---

## What You Can Do With the Point Cloud

### 1. 🗺️ Mapping & SLAM
Build a 3D map of the environment as the car drives. The most natural use for a 32-channel LiDAR.

**Recommended tools:**
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) — 2D SLAM using a projected laser scan, works out of the box with ROS2 Humble
- [`cartographer_ros`](https://google-cartographer-ros.readthedocs.io) — 2D/3D SLAM by Google
- [`LIO-SAM`](https://github.com/TixiaoShan/LIO-SAM) — tightly coupled LiDAR + IMU odometry *(you already have the IMU sensor!)*
- [`LeGO-LOAM`](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) — lightweight LiDAR odometry & mapping

**Inputs needed:** `/carla/hero/lidar` + `/carla/hero/imu` — both already configured.

---

### 2. 🚗 3D Object Detection
Detect vehicles, pedestrians, and obstacles directly from point cloud clusters.

**Approach A — Classical pipeline (no ML required):**
```
Raw PointCloud2
    → Ground plane removal   (RANSAC plane fitting)
    → Euclidean cluster extraction
    → Bounding box fitting around each cluster
    → Size-based classification (car / pedestrian / cyclist)
```

**Approach B — Use the Semantic LiDAR (easiest in CARLA):**
`/carla/hero/semantic_lidar` already labels every point with its object class.
You can skip detection entirely and jump straight to object tracking.

Semantic LiDAR point fields: `x, y, z, cos_inc_angle, object_idx, object_tag`

| `object_tag` value | Class |
|---|---|
| 10 | Vehicle |
| 4  | Pedestrian |
| 7  | Vegetation |
| 8  | Building |
| 6  | Road |

**ROS2 packages:** `pcl_ros`, `pcl_conversions`

---

### 3. 📍 LiDAR-Based Localization
Localize the vehicle against a pre-built map using scan matching.

**Tools:**
- `ndt_scan_matcher` from [Autoware Universe](https://github.com/autowarefoundation/autoware.universe) — NDT-based
- `icp_localization` — Iterative Closest Point matching against a saved map

---

### 4. 🛑 LiDAR Obstacle Detection (Phase 3 upgrade)
Your current AEB system uses cameras. LiDAR-based detection would work at longer range,
in darkness, and without depending on semantic segmentation colors.

**Pipeline:**
```
PointCloud2
    → Crop to forward ROI (x: 0–30m, y: ±3m lane width)
    → Remove ground plane
    → Euclidean clustering
    → If cluster centroid within stopping distance → publish brake command
```
This could run alongside or replace the current `collision_avoidance_system` node.

---

### 5. 📊 Recording & Offline Analysis
Record raw point cloud data for offline processing or ML dataset creation:

```bash
# Record LiDAR + TF (IMPORTANT: always include /tf and /tf_static for RViz2 playback)
ros2 bag record /carla/hero/lidar /carla/hero/semantic_lidar /carla/hero/odometry /tf /tf_static

# Play back later
ros2 bag play <bag_folder>
```

For offline processing, convert to PCD files or use Open3D / PyTorch3D.

---

## Starter Python Node — Read Point Cloud in ROS2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.callback,
            10)

    def callback(self, msg):
        # Convert PointCloud2 → numpy array (N x 4): x, y, z, intensity
        points = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)))

        self.get_logger().info(f'Total points: {len(points)}')

        # Filter: front of car (x > 0) within 20m
        dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        front_points = points[(points[:, 0] > 0) & (dist < 20)]
        self.get_logger().info(f'Front 20m: {len(front_points)} points')

def main():
    rclpy.init()
    rclpy.spin(LidarProcessor())

if __name__ == '__main__':
    main()
```

### For Semantic LiDAR:
```python
points = np.array(list(pc2.read_points(
    msg, field_names=('x', 'y', 'z', 'object_tag'), skip_nans=True)))

vehicles    = points[points[:, 3] == 10]
pedestrians = points[points[:, 3] == 4]
```

---

## Suggested Phase 3 Roadmap

| Step | Feature | Tools |
|---|---|---|
| 3.1 | LiDAR obstacle detection (replace/augment camera AEB) | `pcl_ros`, custom node |
| 3.2 | SLAM — build map while driving | `slam_toolbox` or `cartographer` |
| 3.3 | LiDAR + IMU odometry | `LIO-SAM` |
| 3.4 | Semantic object tracking | Semantic LiDAR + cluster tracker |
| 3.5 | Traffic light compliance | `sensor.pseudo.traffic_lights` (already in config) |

---

## Dependencies to Install

```bash
# PCL ROS2 bindings
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions

# sensor_msgs_py (for reading PointCloud2 in Python)
sudo apt install ros-humble-sensor-msgs-py

# slam_toolbox (if going SLAM route)
sudo apt install ros-humble-slam-toolbox
```
