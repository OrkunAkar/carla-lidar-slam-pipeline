#!/usr/bin/env python3
"""
compare_trajectories.py
─────────────────────────────────────────────────────────────
Compares LIO-SAM estimated trajectory against CARLA ground-truth
odometry from the recorded bag.

Steps:
  1. Extract GT positions from /carla/hero/odometry in the bag
  2. Load LIO-SAM keyframe positions from trajectory.pcd
  3. Spatially align both using SE(2) Umeyama (optimal R + t, no scale)
  4. Compute ATE-RMSE (Absolute Trajectory Error)
  5. Save side-by-side comparison PNG + print metrics table

Usage:
    source /opt/ros/humble/setup.bash
    python3 compare_trajectories.py

    # or explicit paths:
    python3 compare_trajectories.py \\
        --bag  ~/CARLA_PROJECT/bags/carla_recording_2026_05_30-14_25_43 \\
        --traj ~/CARLA_PROJECT/maps/town10hd/trajectory.pcd \\
        --out  ~/CARLA_PROJECT/maps/town10hd/trajectory_comparison.png
"""

import argparse, os, sys, glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ── Default paths ─────────────────────────────────────────────────────────────
BAG_DIR    = os.path.expanduser('~/CARLA_PROJECT/bags')
TRAJ_PCD   = os.path.expanduser('~/CARLA_PROJECT/maps/town10hd/trajectory.pcd')
OUT_PNG    = os.path.expanduser('~/CARLA_PROJECT/maps/town10hd/trajectory_comparison.png')


# ══════════════════════════════════════════════════════════════════════════════
# 1. Read GT odometry from bag
# ══════════════════════════════════════════════════════════════════════════════

def read_gt_from_bag(bag_path):
    """Extract x, y, z positions from /carla/hero/odometry."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Filter to only the odometry topic
    filter_ = rosbag2_py.StorageFilter(topics=['/carla/hero/odometry'])
    reader.set_filter(filter_)

    positions = []
    timestamps = []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        positions.append([p.x, p.y, p.z])
        timestamps.append(ts * 1e-9)   # nanoseconds → seconds

    positions = np.array(positions, dtype=np.float64)
    timestamps = np.array(timestamps, dtype=np.float64)
    timestamps -= timestamps[0]         # zero-start

    print(f"  GT odometry : {len(positions)} poses  "
          f"({timestamps[-1]:.1f}s duration)")
    return positions, timestamps


# ══════════════════════════════════════════════════════════════════════════════
# 2. Read LIO-SAM trajectory.pcd
# ══════════════════════════════════════════════════════════════════════════════

def read_pcd_xyz(path):
    """Read XYZ from a PCD file (binary or ASCII)."""
    # Try Open3D first
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        if len(pts) > 0:
            return pts
    except Exception:
        pass

    # Manual binary PCD reader
    with open(path, 'rb') as f:
        header = {}
        for _ in range(50):          # read header lines
            line = f.readline().decode('utf-8', errors='ignore').strip()
            parts = line.split()
            if parts:
                header[parts[0].upper()] = parts[1:]
            if parts and parts[0].upper() == 'DATA':
                data_type = parts[1].lower()
                break

        n_pts = int(header.get('POINTS', [0])[0])
        if data_type == 'binary':
            raw = np.frombuffer(f.read(), dtype=np.float32)
            n_fields = len(header.get('FIELDS', []))
            if n_fields == 0:
                n_fields = len(raw) // n_pts
            arr = raw.reshape(n_pts, n_fields)
            return arr[:, :3].astype(np.float64)
        else:  # ascii
            pts = []
            for line in f:
                vals = line.decode('utf-8', errors='ignore').strip().split()
                if len(vals) >= 3:
                    try:
                        pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
                    except ValueError:
                        pass
            return np.array(pts, dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════════════
# 3. SE(2) Umeyama alignment (optimal rotation + translation, no scale)
# ══════════════════════════════════════════════════════════════════════════════

def umeyama_2d(P, Q):
    """
    Find the optimal rigid-body transform (R, t) in 2D such that:
        Q ≈ R @ P + t

    Uses Umeyama (1991) closed-form solution via SVD.

    P, Q : (N, 2) arrays  — estimated and reference trajectories
    Returns R (2×2), t (2,), P_aligned (N, 2), residuals (N,)
    """
    assert P.shape == Q.shape and P.shape[1] == 2

    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)

    P_c = P - mu_P
    Q_c = Q - mu_Q

    H = P_c.T @ Q_c                    # covariance matrix
    U, S, Vt = np.linalg.svd(H)

    # Handle reflection (det = -1 means reflection, not rotation)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, np.sign(d)])

    R = Vt.T @ D @ U.T                 # optimal rotation (2×2)
    t = mu_Q - R @ mu_P                # optimal translation

    P_aligned  = (R @ P.T).T + t
    residuals  = np.linalg.norm(Q - P_aligned, axis=1)

    return R, t, P_aligned, residuals


# ══════════════════════════════════════════════════════════════════════════════
# 4. Interpolate to common length
# ══════════════════════════════════════════════════════════════════════════════

def resample_path(pts, n):
    """Uniformly resample a path to n points using cumulative arc length."""
    if len(pts) == n:
        return pts
    diffs = np.diff(pts, axis=0)
    seg_len = np.sqrt((diffs**2).sum(axis=1))
    arc = np.concatenate([[0], np.cumsum(seg_len)])
    arc_norm = arc / arc[-1]
    out_norm = np.linspace(0, 1, n)
    resampled = np.column_stack([
        np.interp(out_norm, arc_norm, pts[:, i]) for i in range(pts.shape[1])
    ])
    return resampled


# ══════════════════════════════════════════════════════════════════════════════
# 5. Plot
# ══════════════════════════════════════════════════════════════════════════════

def gradient_line(ax, xy, cmap_name, lw=2.0, label='', zorder=3):
    """Draw a path with a colour gradient along its length."""
    pts   = xy.reshape(-1, 1, 2)
    segs  = np.concatenate([pts[:-1], pts[1:]], axis=1)
    t     = np.linspace(0, 1, len(segs))
    lc    = LineCollection(segs, cmap=cmap_name, linewidth=lw,
                           zorder=zorder, label=label)
    lc.set_array(t)
    ax.add_collection(lc)
    return lc


def make_figure(gt_xy, lidar_aligned_xy, lidar_raw_xy,
                residuals, out_path, dpi=200):

    fig = plt.figure(figsize=(18, 8), facecolor='#0d0d0d')
    gs  = fig.add_gridspec(1, 3, width_ratios=[2, 2, 1.2],
                           wspace=0.35, left=0.06, right=0.97,
                           top=0.88, bottom=0.10)

    ax1 = fig.add_subplot(gs[0])   # GT trajectory
    ax2 = fig.add_subplot(gs[1])   # comparison overlay
    ax3 = fig.add_subplot(gs[2])   # error histogram

    dark = '#0d0d0d'
    for ax in [ax1, ax2, ax3]:
        ax.set_facecolor(dark)
        ax.tick_params(colors='white')
        ax.spines[:].set_color('#444')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        ax.grid(True, color='#333', linewidth=0.4, alpha=0.6)

    # ── Panel 1: Ground Truth ────────────────────────────────────────────────
    ax1.set_title('CARLA Ground-Truth Odometry', fontsize=12)
    lc1 = gradient_line(ax1, gt_xy, 'cool', lw=2.5)
    ax1.scatter(*gt_xy[0],  s=80, c='lime', zorder=6, label='Start')
    ax1.scatter(*gt_xy[-1], s=120, c='red', marker='*', zorder=6, label='End')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.legend(facecolor='#111', edgecolor='#555', labelcolor='white',
               fontsize=9, loc='upper right')
    _add_stats(ax1, gt_xy, color='#aaa')

    # ── Panel 2: Overlay ─────────────────────────────────────────────────────
    ax2.set_title('GT vs LIO-SAM (after alignment)', fontsize=12)
    lc2 = gradient_line(ax2, gt_xy,           'cool',  lw=2.5, label='GT (CARLA)')
    lc3 = gradient_line(ax2, lidar_aligned_xy, 'autumn', lw=2.0, label='LIO-SAM')

    # Error segments connecting paired points
    for g, l in zip(gt_xy[::5], lidar_aligned_xy[::5]):
        ax2.plot([g[0], l[0]], [g[1], l[1]],
                 color='white', alpha=0.15, linewidth=0.7)

    ax2.scatter(*gt_xy[0],           s=80, c='lime', zorder=6)
    ax2.scatter(*lidar_aligned_xy[0], s=80, c='orange', zorder=6)
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_aspect('equal')

    # Custom legend
    from matplotlib.lines import Line2D
    handles = [Line2D([0],[0], color='cyan',   lw=2, label='GT (CARLA)'),
               Line2D([0],[0], color='orange', lw=2, label='LIO-SAM'),
               Line2D([0],[0], color='white',  lw=1, alpha=0.4, label='Error links')]
    ax2.legend(handles=handles, facecolor='#111', edgecolor='#555',
               labelcolor='white', fontsize=9, loc='upper right')

    # ── Panel 3: Error distribution ──────────────────────────────────────────
    ax3.set_title('ATE Distribution', fontsize=12)
    ax3.hist(residuals, bins=30, color='#e07b39', edgecolor='#0d0d0d',
             linewidth=0.5, alpha=0.85)
    ax3.axvline(residuals.mean(),   color='yellow', lw=1.5,
                linestyle='--', label=f'Mean: {residuals.mean():.2f}m')
    ax3.axvline(np.median(residuals), color='cyan', lw=1.5,
                linestyle=':', label=f'Median: {np.median(residuals):.2f}m')
    ax3.set_xlabel('Position error (m)')
    ax3.set_ylabel('Count')
    ax3.legend(facecolor='#111', edgecolor='#555', labelcolor='white', fontsize=9)

    # ── Metrics box (title area) ──────────────────────────────────────────────
    rmse   = np.sqrt(np.mean(residuals**2))
    mae    = residuals.mean()
    max_e  = residuals.max()

    fig.suptitle(
        f'LIO-SAM Trajectory Accuracy — Town10HD\n'
        f'ATE RMSE: {rmse:.3f} m   |   MAE: {mae:.3f} m   |   '
        f'Max: {max_e:.3f} m   |   Keyframes: {len(lidar_aligned_xy)}',
        color='white', fontsize=13, y=0.98)

    fig.savefig(out_path, dpi=dpi, facecolor=fig.get_facecolor(),
                bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {out_path}")


def _add_stats(ax, xy, color='#888'):
    w = xy[:, 0].ptp(); h = xy[:, 1].ptp()
    total = np.sqrt(np.diff(xy, axis=0).__pow__(2).sum(axis=1)).sum()
    txt = f"Area: {w:.0f}×{h:.0f}m\nPath: {total:.0f}m"
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, color=color,
            fontsize=8, fontfamily='monospace', verticalalignment='bottom',
            bbox=dict(facecolor='#111', edgecolor='#444', alpha=0.6, boxstyle='round'))


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def find_latest_bag(bag_dir):
    candidates = sorted(glob.glob(os.path.join(bag_dir, 'carla_recording_*')),
                        reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No bags found in {bag_dir}")
    return candidates[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag',  default=None,    help='Bag directory path')
    parser.add_argument('--traj', default=TRAJ_PCD, help='trajectory.pcd path')
    parser.add_argument('--out',  default=OUT_PNG,  help='Output PNG path')
    parser.add_argument('--dpi',  type=int, default=200)
    args = parser.parse_args()

    bag_path = args.bag or find_latest_bag(BAG_DIR)
    print(f"\nBag  : {bag_path}")
    print(f"Traj : {args.traj}")
    print()

    # ── 1. Load data ──────────────────────────────────────────────────────────
    print("Reading GT odometry from bag...")
    gt_pts, gt_ts = read_gt_from_bag(bag_path)

    print("Reading LIO-SAM trajectory.pcd...")
    lidar_pts = read_pcd_xyz(args.traj)
    print(f"  LIO-SAM traj: {len(lidar_pts)} keyframes")

    if len(lidar_pts) < 5:
        print("ERROR: LIO-SAM trajectory too short — did the map save correctly?")
        sys.exit(1)

    # ── 2. Work in XY plane (ignore Z for ATE — standard practice) ───────────
    gt_xy    = gt_pts[:, :2]
    lidar_xy = lidar_pts[:, :2]

    # ── 3. Resample to same count for alignment ───────────────────────────────
    n = min(len(gt_xy), len(lidar_xy))
    gt_rs    = resample_path(gt_xy,    n)
    lidar_rs = resample_path(lidar_xy, n)

    # ── 4. Umeyama alignment ──────────────────────────────────────────────────
    print("Aligning trajectories (Umeyama SE(2))...")
    R, t, lidar_aligned, residuals = umeyama_2d(lidar_rs, gt_rs)

    # ── 5. Metrics ────────────────────────────────────────────────────────────
    rmse   = np.sqrt(np.mean(residuals**2))
    mae    = residuals.mean()
    median = np.median(residuals)
    max_e  = residuals.max()

    path_len = np.sqrt((np.diff(gt_rs, axis=0)**2).sum(axis=1)).sum()
    rel_err  = rmse / path_len * 100

    print()
    print("━" * 50)
    print("  TRAJECTORY ACCURACY RESULTS")
    print("━" * 50)
    print(f"  GT poses          : {len(gt_pts)}")
    print(f"  LIO-SAM keyframes : {len(lidar_pts)}")
    print(f"  Compared poses    : {n}")
    print(f"  GT path length    : {path_len:.1f} m")
    print()
    print(f"  ATE RMSE          : {rmse:.4f} m  ← main metric")
    print(f"  MAE               : {mae:.4f} m")
    print(f"  Median error      : {median:.4f} m")
    print(f"  Max error         : {max_e:.4f} m")
    print(f"  Relative error    : {rel_err:.2f}% of path length")
    print("━" * 50)
    print()

    # ── 6. Plot ───────────────────────────────────────────────────────────────
    print(f"Generating comparison figure ({args.dpi} DPI)...")
    make_figure(gt_rs, lidar_aligned, lidar_rs, residuals, args.out, dpi=args.dpi)

    # Also save metrics to text file
    metrics_path = args.out.replace('.png', '_metrics.txt')
    with open(metrics_path, 'w') as f:
        f.write("LIO-SAM Trajectory Accuracy — Town10HD\n")
        f.write("="*45 + "\n")
        f.write(f"GT poses          : {len(gt_pts)}\n")
        f.write(f"LIO-SAM keyframes : {len(lidar_pts)}\n")
        f.write(f"GT path length    : {path_len:.1f} m\n\n")
        f.write(f"ATE RMSE          : {rmse:.4f} m\n")
        f.write(f"MAE               : {mae:.4f} m\n")
        f.write(f"Median error      : {median:.4f} m\n")
        f.write(f"Max error         : {max_e:.4f} m\n")
        f.write(f"Relative error    : {rel_err:.2f}%\n")
    print(f"Metrics saved: {metrics_path}")
    print("\nDone.")


if __name__ == '__main__':
    main()
