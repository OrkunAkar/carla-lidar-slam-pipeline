# 🚗 CARLA Self-Driving Car Project — Full Analysis

## Overview

This is a **final-year computer engineering project** implementing a self-driving car software stack using **CARLA 0.9.15** and **ROS 2 Humble** on Ubuntu 22.04. The architecture is split across two ROS 2 workspaces — one for the upstream CARLA-ROS2 bridge and one for custom application logic.

---

## 📁 Project Directory Structure

```
CARLA_PROJECT/
├── CARLA_0.9.15/              # CARLA simulator binary (installed)
├── CARLA_0.9.15.tar.gz        # Source archive (~8.4 GB)
├── proc1.md                   # Quick-start terminal cheat sheet
├── ros2_bridge_ws/            # Upstream CARLA ROS2 bridge workspace
│   └── ros-bridge/src/        # 24 bridge packages (built + installed)
└── carla_ros2_ws/             # Custom application workspace
    └── src/Self-Driving-Car-CARLA-ROS2/
        ├── car_definition_file.json   # Vehicle + sensor configuration
        └── src/
            ├── navigation/            # Navigation HMI node
            ├── collision_avoidance_system/  # AEB / collision module
            └── task_launch/           # Launch files
```

---

## 🧱 Architecture: Two Workspaces

### 1. `ros2_bridge_ws` — CARLA ROS2 Bridge (Upstream)
Forked from [`ttgamage/carla-ros-bridge`](https://github.com/ttgamage/carla-ros-bridge). Contains **24 ROS2 packages** providing the CARLA↔ROS2 interface layer:

| Package | Role |
|---|---|
| `carla_ros_bridge` | Core bridge: spawns actors, translates CARLA→ROS2 topics |
| `carla_waypoint_publisher` | A* route planner from current pose to goal |
| `carla_ad_agent` / `local_planner` | PID/speed controller following waypoints |
| `carla_msgs` / `carla_ackermann_msgs` | Custom ROS2 message types |
| `carla_spawn_objects` | Spawns sensors/actors via JSON definition |
| `carla_manual_control` | Manual `pygame` driving node |
| `pcl_recorder` | Records point cloud data |
| `rviz_carla_plugin` | Custom RViz2 panels |
| `rqt_carla_control` | RQT GUI control panel |

### 2. `carla_ros2_ws` — Custom Self-Driving Stack
The student-written application layer. Contains **3 custom packages**:

---

## 📦 Custom Packages Deep Dive

### `navigation` — Navigation HMI Node

**File:** `navigation_hmi.py` (119 lines)

**Node name:** `Carla_navigation_hmi`

**What it does:**
- Connects directly to the CARLA Python API at startup (`localhost:2000`)
- Generates all road-network waypoints at 0.7 m intervals
- Publishes them as a `MarkerArray` to `/carla_road_network` every 1 second → visible as green dots in RViz2
- Listens for `/goal_pose` from RViz2 "2D Goal Pose" tool
- On receiving a goal: optionally publishes a **speed command** (`20.0 km/h` or `0.0`) to `/carla/hero/speed_command`, then forwards the goal to `/carla/hero/goal_pose` for the waypoint publisher

**Key design decision:** `enable_spd_cmd` parameter (default `true`) — in Phase 2 this is set to `false` because the collision avoidance system takes over speed control.

**Coordinate quirk:** Y and Z axes are negated/inverted when converting CARLA world coordinates to RViz2 marker positions (CARLA uses a left-handed coordinate system vs. ROS right-handed).

---

### `collision_avoidance_system` — AEB Module

**Files:** `collision_monitor_control.py` (151 lines) + `sarround_monitor.py` (255 lines)

**Node name:** `collision_monitor_control`

**What it does:**
Implements an **Automatic Emergency Braking (AEB)** system using three front-facing camera streams:

| Sensor Input | ROS2 Topic |
|---|---|
| RGB camera (1280×768) | `/carla/hero/rgb_front/image` |
| Semantic segmentation camera | `/carla/hero/semantic_segmentation_front_normal/image` |
| Depth camera | `/carla/hero/depth_front_normal/image` |
| Vehicle speed | `/carla/hero/vehicle_status` |

**Algorithm (in `SarroundMonitor`):**

1. **Object masking:** Uses semantic segmentation pixel colors to isolate 7 dangerous object types: `car`, `truck`, `bus`, `motorcycle`, `bicycle`, `rider`, `pedestrian`

2. **Dynamic danger zone:** A trapezoid-shaped region in front of the vehicle. The zone size **scales with vehicle speed** (10% at low speed → 100% at `speed_limit - 10 km/h` ≈ 50 km/h)

3. **Stopping distance prediction:** Physics-based formula:
   ```
   stopping_distance = (v² × 0.004286) + (v × 0.3)  [min: 3m]
   ```

4. **Depth intersection:** Objects within the predicted stopping distance that fall inside the danger zone trigger an **AEB alert**

5. **Speed control output** (published every 100ms):
   - Object in danger zone → `speed - 1.0 km/h` (gradual slowdown)
   - Object within stopping distance in zone → `0.0 km/h` (full stop / AEB)
   - Clear → `speed + 1.0 km/h` (resume up to `speed_limit = 60 km/h`)

6. **Visual HUD:** Draws the detection zone, object outlines, and AEB status overlay on the RGB feed via `cv2.imshow()`

---

### `task_launch` — Launch Files

Two launch files:

**`multi_node_launch.py`** — Launches 3 nodes together:
- `carla_ad_agent/local_planner`
- `navigation/navigation_hmi` (with `enable_spd_cmd:=false`)
- `collision_avoidance_system/collision_monitor_control`

All under the namespace `SELF_DRIVE_stack`.

**`master_launch.py`** — The top-level one-shot launcher:
1. `carla_ros_bridge_with_example_ego_vehicle.launch.py` (with custom `car_definition_file.json`)
2. `carla_waypoint_publisher.launch.py`
3. `multi_node_launch.py`

---

## 🚗 Vehicle & Sensor Configuration (`car_definition_file.json`)

**Ego vehicle:** `vehicle.dodge.charger_police_2020` (role: `hero`)

| Sensor | ID | Position | Purpose |
|---|---|---|---|
| RGB camera | `rgb_front` | +2m front, +2m up | AEB visual input |
| Semantic seg | `semantic_segmentation_front_normal` | same | AEB object classification |
| Depth camera | `depth_front_normal` | same | AEB distance measurement |
| RGB (follow-cam) | `rgb_view` | -4.5m rear, +2.8m up, -20° pitch | Third-person view |
| LiDAR (32ch) | `lidar` | roof (+2.4m) | RViz2 point cloud |
| Semantic LiDAR | `semantic_lidar` | roof (+2.4m) | Labeled point cloud |
| Radar | `radar_front` | front | Range sensing |
| GNSS | `gnss` | front | GPS positioning |
| IMU | `imu` | front | Inertial data |
| Collision sensor | `collision` | origin | Event detection |
| Lane invasion | `lane_invasion` | origin | Lane departure |
| Odometry (pseudo) | `odometry` | — | ROS odom topic |
| TF (pseudo) | `tf` | — | Transform tree |

> **LiDAR spec:** 32 channels, 50m range, 320,000 pts/sec, 20 Hz rotation — a strong 3D-LiDAR config.

---

## 🔄 Data / Control Flow

```
CARLA Simulator (port 2000)
         │
         ▼
  carla_ros_bridge  ──────────────────────────────────┐
  (spawns ego vehicle,                                 │
   publishes sensor topics)                            │
         │                                             │
    ┌────┴────────────────┐                            │
    │                     │                            │
    ▼                     ▼                            ▼
carla_waypoint_     collision_avoidance_        navigation_hmi
publisher           system                      (HMI + road map)
(A* route           (AEB via RGB/seg/depth)     │
planning)           │                           │ /carla/hero/goal_pose
    │               │ speed_command (0-60 km/h) │
    ▼               ▼                           ▼
local_planner ──── /carla/hero/speed_command ◄──┘
(PID controller)
    │
    ▼
CARLA vehicle control (throttle/steer/brake)
```

---

## 🚦 Development Phases

### ✅ Phase 1 — Basic Route Following
- CARLA bridge → ego vehicle spawned
- Waypoint publisher computes A* route to goal
- `local_planner` PID controller follows path
- Navigation HMI provides goal selection via RViz2
- Speed: fixed 20 km/h from HMI on goal selection

### ✅ Phase 2 — Collision Avoidance (AEB)
- Custom `car_definition_file.json` equips extra cameras (seg + depth)
- `collision_monitor_control` node added
- Speed control **transferred from HMI to AEB system**
- HMI now runs with `enable_spd_cmd:=false`
- AEB dynamically adjusts speed based on frontal object detection

---

## ⚠️ Observations & Potential Issues

| # | Issue | Severity | Details |
|---|---|---|---|
| 1 | **Typo in filename** | Low | `sarround_monitor.py` — should be `surround_monitor.py`. Also `set_mone_zone()` → `set_monitor_zone()` |
| 2 | **Speed race condition** | Medium | Both `navigation_hmi` and `collision_avoidance_system` publish to `/carla/hero/speed_command`. Race condition if `enable_spd_cmd` is not properly set to `false` in Phase 2. |
| 3 | **Speed increment is absolute** | Medium | AEB adjusts speed by ±1.0 (absolute km/h) regardless of actual speed, applied at 10 Hz. This means max acceleration = 10 km/h/s which is fine, but abrupt. |
| 4 | **Depth image interpretation** | Low | The depth decoding comment in `get_near_obj()` is partially commented out. The current implementation passes the raw `imgdep` array directly and compares with `distance`, which only works correctly if the depth image is already in meters (as CARLA's depth sensor with `passthrough` encoding is). |
| 5 | **`carla_marker_to_ros_marker` imports `Marker` inside the function** | Low | Redundant — `Marker` is already imported at module level at line 3. |
| 6 | **Node namespace mismatch** | Low | `multi_node_launch.py` puts all nodes under `SELF_DRIVE_stack` namespace, but topics like `/carla/hero/speed_command` are published without that namespace — this is fine since the publishers/subscribers use absolute topic paths. |
| 7 | **No Phase-3 yet** | Info | LiDAR, GNSS, IMU, Radar, and Semantic LiDAR are all already configured in `car_definition_file.json` — suggesting a future SLAM/localization phase is planned. |
| 8 | **Waypoint density** | Low | `generate_waypoints(0.7)` creates very dense markers (every 70 cm across the entire map). Publishing a full `MarkerArray` every 1 second in RViz2 could be slow on large maps. |

---

## 🧩 ROS2 Topic Summary

| Topic | Type | Direction |
|---|---|---|
| `/carla_road_network` | `MarkerArray` | navigation_hmi → RViz2 |
| `/carla/hero/goal_pose` | `PoseStamped` | navigation_hmi → waypoint_publisher |
| `/carla/hero/speed_command` | `Float64` | HMI / AEB → local_planner |
| `/carla/hero/vehicle_status` | `CarlaEgoVehicleStatus` | bridge → AEB |
| `/carla/hero/rgb_front/image` | `Image` | bridge → AEB |
| `/carla/hero/semantic_segmentation_front_normal/image` | `Image` | bridge → AEB |
| `/carla/hero/depth_front_normal/image` | `Image` | bridge → AEB |
| `/carla/hero/lidar` | `PointCloud2` | bridge → RViz2 |
| `/goal_pose` | `PoseStamped` | RViz2 → navigation_hmi |
| `/initialpose` | `PoseWithCovarianceStamped` | RViz2 → navigation_hmi (logged only) |

---

## 🔭 What's Next (Suggested Phase 3)

Based on the rich sensor suite already configured, likely next steps:
- **SLAM** using LiDAR point clouds (e.g., with `slam_toolbox` or `cartographer`)
- **Sensor fusion** combining LiDAR + GNSS + IMU
- **Traffic light / sign compliance** using `sensor.pseudo.traffic_lights`
- **Object list integration** using `sensor.pseudo.objects`
- **Lane keeping** using the semantic segmentation camera
