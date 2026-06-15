import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time


class LidarAnalyzer(Node):
    def __init__(self):
        super().__init__('lidar_analyzer')

        self.scan_count = 0
        self.last_time = time.time()

        self.sub = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.callback,
            10)

        self.get_logger().info('LiDAR Analyzer started — waiting for scans on /carla/hero/lidar ...')

    def callback(self, msg):
        # ── Convert PointCloud2 → structured numpy array ─────────────────────
        # ROS2 Humble returns a structured array with named fields
        points = np.array(list(pc2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'intensity'),
            skip_nans=True)))

        if points.size == 0:
            self.get_logger().warn('Received empty point cloud!')
            return

        x         = points['x'].astype(np.float32)
        y         = points['y'].astype(np.float32)
        z         = points['z'].astype(np.float32)
        intensity = points['intensity'].astype(np.float32)

        # ── Distance from sensor (horizontal) ─────────────────────────────────
        dist = np.sqrt(x**2 + y**2)

        # ── Zones ─────────────────────────────────────────────────────────────
        front_mask  = (x > 0) & (dist < 20)   # forward 20m cone
        ground_mask = (z < -1.5)               # ground is BELOW sensor (~-2.4m), not above

        # ── FPS estimate ──────────────────────────────────────────────────────
        self.scan_count += 1
        now = time.time()
        elapsed = now - self.last_time
        if elapsed >= 5.0:
            fps = self.scan_count / elapsed
            self.get_logger().info(f'Scan rate: {fps:.1f} Hz  ({self.scan_count} scans in {elapsed:.1f}s)')
            self.scan_count = 0
            self.last_time = now

        # ── Print stats every scan ────────────────────────────────────────────
        sep = '─' * 52
        self.get_logger().info(f'\n{sep}')
        self.get_logger().info(f'  Total points     : {len(points):>7,}')
        self.get_logger().info(f'  Distance  min    : {dist.min():>7.2f} m')
        self.get_logger().info(f'  Distance  max    : {dist.max():>7.2f} m')
        self.get_logger().info(f'  Distance  mean   : {dist.mean():>7.2f} m')
        self.get_logger().info(f'  Z         min    : {z.min():>7.2f} m')
        self.get_logger().info(f'  Z         max    : {z.max():>7.2f} m')
        self.get_logger().info(f'  Intensity min    : {intensity.min():>7.2f}')
        self.get_logger().info(f'  Intensity max    : {intensity.max():>7.2f}')
        self.get_logger().info(f'  Front 20m points : {front_mask.sum():>7,}  ({100*front_mask.mean():.1f}%)')
        self.get_logger().info(f'  Ground points    : {ground_mask.sum():>7,}  ({100*ground_mask.mean():.1f}%)')
        self.get_logger().info(f'{sep}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
