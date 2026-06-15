import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class GroundRemover(Node):
    """
    Separates ground points from object points using RANSAC plane fitting.

    Subscribes:  /carla/hero/lidar          (raw PointCloud2)
    Publishes:   /carla/hero/lidar/ground   (road surface points)
                 /carla/hero/lidar/objects  (everything above ground)
    """

    # ── RANSAC parameters ─────────────────────────────────────────────────────
    RANSAC_ITERATIONS  = 100   # more = more accurate, more CPU
    DISTANCE_THRESHOLD = 0.25  # meters — points within this distance = ground
    MIN_GROUND_RATIO   = 0.30  # sanity check: reject plane if < 30% inliers

    def __init__(self):
        super().__init__('ground_remover')

        self.sub = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.callback,
            10)

        self.pub_ground = self.create_publisher(
            PointCloud2, '/carla/hero/lidar/ground', 10)

        self.pub_objects = self.create_publisher(
            PointCloud2, '/carla/hero/lidar/objects', 10)

        self.get_logger().info(
            'Ground Remover started.\n'
            '  Input : /carla/hero/lidar\n'
            '  Output: /carla/hero/lidar/ground\n'
            '          /carla/hero/lidar/objects')

    # ── RANSAC plane fitting ───────────────────────────────────────────────────
    def ransac_plane(self, pts):
        """
        Fit a plane to pts (N x 3) using RANSAC.
        Returns a boolean mask: True = inlier (ground), False = outlier (object).
        """
        n = len(pts)
        best_mask  = np.zeros(n, dtype=bool)
        best_count = 0

        for _ in range(self.RANSAC_ITERATIONS):
            # 1. Pick 3 random points
            idx = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = pts[idx]

            # 2. Compute plane normal via cross product of two edge vectors
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-6:
                continue          # degenerate — 3 collinear points, skip
            normal /= norm_len

            # 3. Plane equation: normal · p + d = 0
            d = -np.dot(normal, p1)

            # 4. Distance of every point to this plane
            distances = np.abs(pts @ normal + d)
            inlier_mask = distances < self.DISTANCE_THRESHOLD
            count = inlier_mask.sum()

            if count > best_count:
                best_count = count
                best_mask  = inlier_mask

        # Sanity check — reject if plane captured too few points
        if best_count / n < self.MIN_GROUND_RATIO:
            self.get_logger().warn(
                f'RANSAC found only {best_count}/{n} inliers — ground plane uncertain.')

        return best_mask

    # ── Main callback ─────────────────────────────────────────────────────────
    def callback(self, msg):
        # Read structured array from PointCloud2
        raw = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))

        if raw.size == 0:
            return

        # Stack into (N x 3) float array
        xyz = np.column_stack([
            raw['x'].astype(np.float32),
            raw['y'].astype(np.float32),
            raw['z'].astype(np.float32)
        ])

        # ── Pre-filter for RANSAC candidates ─────────────────────────────────
        # Only send likely-ground points to RANSAC (z < -1.0m in sensor frame).
        # This speeds up RANSAC dramatically — it only sees road-level points,
        # not walls, buildings, or car roofs.
        candidate_mask = xyz[:, 2] < -1.0
        candidates = xyz[candidate_mask]

        # ── Run RANSAC ────────────────────────────────────────────────────────
        ground_mask = np.zeros(len(xyz), dtype=bool)
        if len(candidates) >= 3:
            ground_in_candidates       = self.ransac_plane(candidates)
            ground_mask[candidate_mask] = ground_in_candidates

        object_mask = ~ground_mask

        # ── Log results ───────────────────────────────────────────────────────
        total   = len(xyz)
        n_gnd   = ground_mask.sum()
        n_obj   = object_mask.sum()
        self.get_logger().info(
            f'Total: {total:,}  |  '
            f'Ground: {n_gnd:,} ({100*n_gnd/total:.1f}%)  |  '
            f'Objects: {n_obj:,} ({100*n_obj/total:.1f}%)')

        # ── Publish separated clouds ──────────────────────────────────────────
        ground_pts = [tuple(p) for p in xyz[ground_mask]]
        object_pts = [tuple(p) for p in xyz[object_mask]]

        self.pub_ground.publish(
            pc2.create_cloud_xyz32(msg.header, ground_pts))
        self.pub_objects.publish(
            pc2.create_cloud_xyz32(msg.header, object_pts))


def main(args=None):
    rclpy.init(args=args)
    node = GroundRemover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
