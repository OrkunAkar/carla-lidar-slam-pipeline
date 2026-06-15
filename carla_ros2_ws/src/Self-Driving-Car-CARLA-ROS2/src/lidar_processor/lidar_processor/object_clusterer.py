import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN


class ObjectClusterer(Node):
    """
    Groups object points into individual clusters and draws 3D bounding boxes.

    Subscribes:  /carla/hero/lidar/objects    (output of ground_remover)
    Publishes:   /lidar/clusters/markers      (MarkerArray — 3D boxes in RViz2)
    """

    # ── DBSCAN parameters ─────────────────────────────────────────────────────
    # Calibrated from detector_validator with driving ego + NPC traffic
    EPS         = 1.5   # bridges gaps across car surface (car ≈ 4.5m long)
    MIN_SAMPLES = 3     # sparse returns at range need lower threshold

    # ── Cluster size filters ──────────────────────────────────────────────────
    MIN_CLUSTER_PTS  = 5     # catch small far-range vehicle returns
    MAX_CLUSTER_PTS  = 3000  # larger = entire building wall, discard

    # ── Size classification thresholds (points) ───────────────────────────────
    # Calibrated measurements:
    #   Far vehicles  (30-50m) : 19 -  52 pts
    #   Near vehicles (10-30m) : 59 - 247 pts
    #   → VEH range must cover 15-500 to handle all distances
    PEDESTRIAN_MAX = 15    # ≤ 15 pts  → pole/small object → green
    VEHICLE_MAX    = 500   # ≤ 500 pts → vehicle (any range) → blue
                           # > 500 pts → large structure      → red

    def __init__(self):
        super().__init__('object_clusterer')

        self.sub = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar/objects',
            self.callback,
            10)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/lidar/clusters/markers', 10)

        self.get_logger().info(
            'Object Clusterer started.\n'
            '  Input : /carla/hero/lidar/objects\n'
            '  Output: /lidar/clusters/markers\n'
            '  Add MarkerArray in RViz2 to visualize bounding boxes.')

    # ── Colour helper ─────────────────────────────────────────────────────────
    def classify_color(self, n_pts):
        """Return RGBA color based on cluster size."""
        if n_pts <= self.PEDESTRIAN_MAX:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)   # green — pedestrian
        elif n_pts <= self.VEHICLE_MAX:
            return ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.6)   # blue  — vehicle
        else:
            return ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.5)   # red   — large

    def classify_label(self, n_pts):
        if n_pts <= self.PEDESTRIAN_MAX:
            return 'PED'
        elif n_pts <= self.VEHICLE_MAX:
            return 'VEH'
        else:
            return 'LRG'

    # ── Build a bounding-box Marker ───────────────────────────────────────────
    def make_box_marker(self, cluster_pts, marker_id, header):
        mins = cluster_pts.min(axis=0)
        maxs = cluster_pts.max(axis=0)
        centre = (mins + maxs) / 2.0
        dims   = maxs - mins

        # Avoid zero-size boxes
        dims = np.clip(dims, 0.1, None)

        m = Marker()
        m.header = header
        m.ns     = 'clusters'
        m.id     = marker_id
        m.type   = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = float(centre[0])
        m.pose.position.y = float(centre[1])
        m.pose.position.z = float(centre[2])
        m.pose.orientation.w = 1.0

        m.scale = Vector3(x=float(dims[0]), y=float(dims[1]), z=float(dims[2]))
        m.color = self.classify_color(len(cluster_pts))
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200_000_000   # 200ms — auto-disappears if no new scan
        return m

    # ── Build a text label Marker ──────────────────────────────────────────────
    def make_text_marker(self, cluster_pts, marker_id, header):
        maxs = cluster_pts.max(axis=0)

        m = Marker()
        m.header = header
        m.ns     = 'labels'
        m.id     = marker_id
        m.type   = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = float(cluster_pts[:, 0].mean())
        m.pose.position.y = float(cluster_pts[:, 1].mean())
        m.pose.position.z = float(maxs[2]) + 0.5   # float above the box
        m.pose.orientation.w = 1.0

        m.scale.z = 0.8   # text height in metres
        m.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text    = f'{self.classify_label(len(cluster_pts))} ({len(cluster_pts)})'
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200_000_000
        return m

    # ── Main callback ─────────────────────────────────────────────────────────
    def callback(self, msg):
        # Read object points from ground_remover output
        raw = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))

        if raw.size == 0:
            return

        xyz = np.column_stack([
            raw['x'].astype(np.float32),
            raw['y'].astype(np.float32),
            raw['z'].astype(np.float32)
        ])

        # ── DBSCAN clustering ─────────────────────────────────────────────────
        labels = DBSCAN(eps=self.EPS, min_samples=self.MIN_SAMPLES).fit_predict(xyz)
        # labels: -1 = noise, 0..N = cluster IDs

        unique_labels = set(labels) - {-1}
        markers = MarkerArray()
        marker_id = 0

        counts = {'ped': 0, 'veh': 0, 'lrg': 0, 'noise': 0, 'filtered': 0}
        counts['noise'] = int((labels == -1).sum())

        for label in unique_labels:
            cluster = xyz[labels == label]
            n = len(cluster)

            # Size filter
            if n < self.MIN_CLUSTER_PTS:
                counts['filtered'] += 1
                continue
            if n > self.MAX_CLUSTER_PTS:
                counts['filtered'] += 1
                continue

            # Count by type
            if n <= self.PEDESTRIAN_MAX:
                counts['ped'] += 1
            elif n <= self.VEHICLE_MAX:
                counts['veh'] += 1
            else:
                counts['lrg'] += 1

            # Build markers
            markers.markers.append(self.make_box_marker(cluster, marker_id, msg.header))
            marker_id += 1
            markers.markers.append(self.make_text_marker(cluster, marker_id, msg.header))
            marker_id += 1

        self.pub_markers.publish(markers)

        total_clusters = len(unique_labels)
        self.get_logger().info(
            f'Clusters: {total_clusters:>3}  |  '
            f'PED: {counts["ped"]}  VEH: {counts["veh"]}  LRG: {counts["lrg"]}  |  '
            f'Noise pts: {counts["noise"]}  Filtered: {counts["filtered"]}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectClusterer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
