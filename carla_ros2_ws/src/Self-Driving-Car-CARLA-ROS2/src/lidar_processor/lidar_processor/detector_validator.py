import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import message_filters
import numpy as np
from sklearn.cluster import DBSCAN


# ── CARLA semantic tag constants ───────────────────────────────────────────────
TAG_VEHICLE    = 10
TAG_PEDESTRIAN = 4
TAG_ROAD       = 7    # Roads in CARLA semantic palette


class DetectorValidator(Node):
    """
    Validates raw-LiDAR object detection against semantic LiDAR ground truth.

    For each scan it:
      1. Extracts ground-truth vehicle positions from semantic LiDAR (tag == 10)
      2. Runs ground removal + DBSCAN on raw LiDAR → detected clusters
      3. Matches detected VEH clusters against GT vehicles (centroid distance < 3m)
      4. Reports: TP, FP, FN, Precision, Recall per scan

    Subscribes:
      /carla/hero/lidar           (raw — X,Y,Z,Intensity)
      /carla/hero/semantic_lidar  (ground truth — X,Y,Z,object_tag)
    """

    # ── Ground removal (simple Z threshold — fast for validation) ─────────────
    # LiDAR is at z=2.4m. Road surface = z≈-2.4m in sensor frame.
    # Vehicle body spans roughly z=-2.1m (roof) to z=-2.35m (wheels).
    # Threshold must be BELOW roof height to preserve vehicle points.
    GROUND_Z_THRESHOLD = -2.1    # keep everything above road surface

    # ── DBSCAN parameters ─────────────────────────────────────────────────────
    EPS          = 1.5   # increased: bridges gaps across car surface (car≈4.5m long)
    MIN_SAMPLES  = 3     # lowered: cars at 40m have very few points per neighbourhood
    MIN_PTS      = 5
    MAX_PTS      = 3000
    VEH_MAX_PTS  = 800   # raised: dense/close vehicles can exceed 500 pts

    # ── Matching threshold ────────────────────────────────────────────────────
    MATCH_DISTANCE = 5.0   # metres — raised for centroid offset tolerance

    def __init__(self):
        super().__init__('detector_validator')

        raw_sub = message_filters.Subscriber(
            self, PointCloud2, '/carla/hero/lidar')
        sem_sub = message_filters.Subscriber(
            self, PointCloud2, '/carla/hero/semantic_lidar')

        # Synchronise the two streams (allow up to 0.15s time difference)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [raw_sub, sem_sub], queue_size=10, slop=0.15)
        self.ts.registerCallback(self.callback)

        self.get_logger().info(
            'Detector Validator started — syncing raw + semantic LiDAR...\n'
            '  Make sure CARLA is running with NPC vehicles spawned.')

    # ── Read raw LiDAR ────────────────────────────────────────────────────────
    def read_xyz(self, msg):
        raw = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.column_stack([
            raw['x'].astype(np.float32),
            raw['y'].astype(np.float32),
            raw['z'].astype(np.float32)])

    # ── Read semantic LiDAR ───────────────────────────────────────────────────
    def read_semantic(self, msg):
        raw = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'ObjTag'), skip_nans=True)))
        if raw.size == 0:
            return np.empty((0, 4), dtype=np.float32)
        return np.column_stack([
            raw['x'].astype(np.float32),
            raw['y'].astype(np.float32),
            raw['z'].astype(np.float32),
            raw['ObjTag'].astype(np.float32)])

    # ── Cluster XYZ points → list of cluster centroids ────────────────────────
    def cluster_centroids(self, xyz, min_pts, max_pts, size_min=0, size_max=99999):
        """Run DBSCAN and return centroids of clusters in the given size range."""
        if len(xyz) < self.MIN_SAMPLES:
            return []
        labels = DBSCAN(eps=self.EPS, min_samples=self.MIN_SAMPLES).fit_predict(xyz)
        centroids = []
        for lbl in set(labels) - {-1}:
            cluster = xyz[labels == lbl]
            n = len(cluster)
            if min_pts <= n <= max_pts and size_min <= n <= size_max:
                centroids.append(cluster.mean(axis=0))
        return centroids

    # ── Match detections to ground truth ─────────────────────────────────────
    def match(self, detections, ground_truth):
        """
        Greedy matching by centroid distance.
        Returns (tp, fp, fn).
        """
        if not ground_truth:
            return 0, len(detections), 0
        if not detections:
            return 0, 0, len(ground_truth)

        gt_arr  = np.array(ground_truth)[:, :2]   # XY only
        det_arr = np.array(detections)[:, :2]

        matched_gt  = set()
        matched_det = set()

        for di, det in enumerate(det_arr):
            dists = np.linalg.norm(gt_arr - det, axis=1)
            nearest_idx = int(np.argmin(dists))
            if dists[nearest_idx] < self.MATCH_DISTANCE and nearest_idx not in matched_gt:
                matched_gt.add(nearest_idx)
                matched_det.add(di)

        tp = len(matched_gt)
        fp = len(detections) - tp
        fn = len(ground_truth) - tp
        return tp, fp, fn

    # ── Main callback ─────────────────────────────────────────────────────────
    def callback(self, raw_msg, sem_msg):
        # 1. Read raw LiDAR → remove ground
        xyz = self.read_xyz(raw_msg)
        if len(xyz) == 0:
            return
        obj_xyz = xyz[xyz[:, 2] >= self.GROUND_Z_THRESHOLD]

        # 2. Read semantic LiDAR → GT vehicle centroids (clean, labeled)
        sem = self.read_semantic(sem_msg)
        veh_pts_sem = sem[sem[:, 3] == TAG_VEHICLE, :3] if len(sem) > 0 else np.empty((0, 3))
        gt_vehicles = self.cluster_centroids(veh_pts_sem, min_pts=5, max_pts=99999)

        if not gt_vehicles:
            self.get_logger().info('No GT vehicles in range this scan.')
            return

        # Convert GT vehicle centroids to array for vectorised ops
        gt_arr = np.array(gt_vehicles)

        # ── Coordinate diagnostic (runs once, then silences itself) ───────────
        if not hasattr(self, '_diag_done'):
            self._diag_done = True
            raw_sample = obj_xyz[0] if len(obj_xyz) > 0 else None
            sem_sample = veh_pts_sem[0] if len(veh_pts_sem) > 0 else None
            gt_sample  = gt_arr[0] if len(gt_arr) > 0 else None
            self.get_logger().warn(
                f'COORD DIAGNOSTIC:\n'
                f'  raw frame_id      : {raw_msg.header.frame_id}\n'
                f'  sem frame_id      : {sem_msg.header.frame_id}\n'
                f'  raw sample XYZ    : {raw_sample}\n'
                f'  sem vehicle XYZ   : {sem_sample}\n'
                f'  GT centroid XYZ   : {gt_sample}\n'
                f'  raw obj_xyz range X: [{obj_xyz[:,0].min():.1f}, {obj_xyz[:,0].max():.1f}]\n'
                f'  raw obj_xyz range Y: [{obj_xyz[:,1].min():.1f}, {obj_xyz[:,1].max():.1f}]\n'
                f'  sem veh range X   : [{veh_pts_sem[:,0].min():.1f}, {veh_pts_sem[:,0].max():.1f}]\n'
                f'  sem veh range Y   : [{veh_pts_sem[:,1].min():.1f}, {veh_pts_sem[:,1].max():.1f}]')

        # 3. For each GT vehicle centroid, check raw LiDAR point coverage
        #    "Is this vehicle visible in the raw LiDAR?"
        raw_xy = obj_xyz[:, :2]

        detectable   = 0
        undetectable = 0
        coverage_list = []

        for centroid in gt_arr:
            dists    = np.linalg.norm(raw_xy - centroid[:2], axis=1)
            n_nearby = int((dists < self.MATCH_DISTANCE).sum())
            coverage_list.append(n_nearby)
            if n_nearby >= 10:
                detectable += 1
            else:
                undetectable += 1

        coverage_pct = 100.0 * detectable / len(gt_vehicles)
        avg_pts      = float(np.mean(coverage_list))
        min_pts_v    = int(np.min(coverage_list))
        max_pts_v    = int(np.max(coverage_list))

        self.get_logger().info(
            f'GT vehicles in range : {len(gt_vehicles):>3}\n'
            f'  Detectable (>=10 raw pts within {self.MATCH_DISTANCE}m) : '
            f'{detectable} / {len(gt_vehicles)}  ({coverage_pct:.0f}%)\n'
            f'  Raw pts per vehicle  — avg: {avg_pts:.1f}  '
            f'min: {min_pts_v}  max: {max_pts_v}\n'
            f'  Undetectable (too sparse)           : {undetectable}')




def main(args=None):
    rclpy.init(args=args)
    node = DetectorValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
