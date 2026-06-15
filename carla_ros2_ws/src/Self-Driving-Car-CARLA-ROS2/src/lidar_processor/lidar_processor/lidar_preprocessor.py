import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


# ── LiDAR configuration (must match car_definition_lidar_only.json) ───────────
N_SCAN             = 32
UPPER_FOV_DEG      =  2.0
LOWER_FOV_DEG      = -26.8
ROTATION_FREQ_HZ   = 20.0
HORIZON_SCAN       = 512       # power-of-2 bin count for LIO-SAM range image

TOTAL_FOV          = UPPER_FOV_DEG - LOWER_FOV_DEG          # 28.8°
CHANNEL_STEP       = TOTAL_FOV / (N_SCAN - 1)               # ~0.929° per ring
SCAN_PERIOD        = 1.0 / ROTATION_FREQ_HZ                 # 0.05s


class LidarPreprocessor(Node):
    """
    Adds 'ring' and 'time' fields to the raw CARLA LiDAR output.

    LIO-SAM requires PointXYZIRT (x, y, z, intensity, ring, time).
    CARLA only outputs (x, y, z, intensity). This node computes:

      ring  — elevation-angle-based channel ID  [0, N_SCAN-1]
      time  — azimuth-based relative timestamp  [0, SCAN_PERIOD) seconds

    Subscribes:  /carla/hero/lidar           (x, y, z, intensity)
    Publishes:   /carla/hero/lidar_iosam     (x, y, z, intensity, ring, time)
                 (Velodyne XYZIRT format — use sensor: velodyne in LIO-SAM)
    """

    # ── Output PointCloud2 field definitions ──────────────────────────────────
    # Layout: x(4B) y(4B) z(4B) intensity(4B) ring(2B) pad(2B) time(4B) = 24B
    FIELDS = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
        PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    POINT_STEP = 24   # bytes per point  (4×float32 + uint16 + 2B pad + float32)

    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.sub = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.callback,
            10)

        self.pub = self.create_publisher(
            PointCloud2,
            '/carla/hero/lidar_iosam',
            10)

        self.get_logger().info(
            'LiDAR Preprocessor started.\n'
            f'  N_SCAN={N_SCAN}  upper={UPPER_FOV_DEG}°  lower={LOWER_FOV_DEG}°\n'
            f'  channel_step={CHANNEL_STEP:.4f}°  scan_period={SCAN_PERIOD*1000:.1f}ms\n'
            '  /carla/hero/lidar  →  /carla/hero/lidar_iosam  (XYZIRT format)')

    def callback(self, msg: PointCloud2):
        # Read raw points
        raw = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)))

        if raw.size == 0:
            return

        x   = raw['x'].astype(np.float32)
        y   = raw['y'].astype(np.float32)
        z   = raw['z'].astype(np.float32)
        intensity = raw['intensity'].astype(np.float32)
        n   = len(x)

        # ── Compute ring (elevation angle → channel index) ────────────────────
        dist_xy  = np.sqrt(x**2 + y**2)
        elev_deg = np.degrees(np.arctan2(z, dist_xy))
        ring_f   = (elev_deg - LOWER_FOV_DEG) / CHANNEL_STEP
        ring     = np.clip(np.round(ring_f).astype(np.int32), 0, N_SCAN - 1).astype(np.uint16)

        # ── Compute time (azimuth → relative scan timestamp) ──────────────────
        # Azimuth in [0, 2π], time in [0, SCAN_PERIOD)
        azimuth   = np.arctan2(y, x)                          # -π to π
        azimuth   = (azimuth + 2 * np.pi) % (2 * np.pi)      # 0 to 2π
        time_pts  = (azimuth / (2 * np.pi) * SCAN_PERIOD).astype(np.float32)

        # ── Pack into binary buffer (vectorised — no Python loop) ────────────
        # Build a structured numpy array then dump straight to bytes.
        # dtype includes 2-byte padding after ring for 4-byte alignment of time.
        dtype = np.dtype([
            ('x',         np.float32),
            ('y',         np.float32),
            ('z',         np.float32),
            ('intensity', np.float32),
            ('ring',      np.uint16),
            ('_pad',      np.uint16),   # alignment padding
            ('time',      np.float32),
        ])
        arr = np.empty(n, dtype=dtype)
        arr['x']         = x
        arr['y']         = y
        arr['z']         = z
        arr['intensity'] = intensity
        arr['ring']      = ring
        arr['_pad']      = 0
        arr['time']      = time_pts

        # ── Build output PointCloud2 ──────────────────────────────────────────
        out = PointCloud2()
        out.header       = msg.header
        out.height       = 1
        out.width        = n
        out.fields       = self.FIELDS
        out.is_bigendian = False
        out.point_step   = self.POINT_STEP
        out.row_step     = self.POINT_STEP * n
        out.data         = arr.tobytes()
        out.is_dense     = True

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
