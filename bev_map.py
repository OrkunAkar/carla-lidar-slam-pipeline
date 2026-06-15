#!/usr/bin/env python3
"""
bev_map.py — Bird's Eye View visualisation of a LIO-SAM map.

Renders a top-down 2D image of GlobalMap.pcd coloured by point height,
with the vehicle trajectory overlaid. Saves a high-res PNG suitable
for inclusion in a project paper.

Usage:
    python3 bev_map.py
    python3 bev_map.py --map  ~/CARLA_PROJECT/maps/town10hd/GlobalMap.pcd
                       --traj ~/CARLA_PROJECT/maps/town10hd/trajectory.pcd
                       --out  ~/CARLA_PROJECT/maps/town10hd/bev_map.png
                       --dpi  300
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')          # no display needed — saves directly to PNG
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection


# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_MAP  = os.path.expanduser('~/CARLA_PROJECT/maps/town10hd/GlobalMap.pcd')
DEFAULT_TRAJ = os.path.expanduser('~/CARLA_PROJECT/maps/town10hd/trajectory.pcd')
DEFAULT_OUT  = os.path.expanduser('~/CARLA_PROJECT/maps/town10hd/bev_map.png')


def read_pcd_xyz(path):
    """Read XYZ from a binary or ASCII PCD file using numpy (no Open3D)."""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        return np.asarray(pcd.points)
    except ImportError:
        pass

    # Fallback: manual PCD parser (handles ASCII PCD from LIO-SAM)
    pts = []
    in_data = False
    data_type = 'ascii'
    with open(path, 'rb') as f:
        for raw_line in f:
            try:
                line = raw_line.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if line.upper().startswith('DATA'):
                data_type = line.split()[1].lower()
                in_data = True
                if data_type == 'binary':
                    # Read remaining bytes as float32 array
                    remaining = f.read()
                    arr = np.frombuffer(remaining, dtype=np.float32)
                    # Reshape: each point is at least x,y,z (first 3 floats)
                    n_fields = len(arr) // (len(arr) // max(1, len(arr) // 4))
                    try:
                        arr = arr.reshape(-1, 4)   # XYZI layout
                        return arr[:, :3]
                    except Exception:
                        arr = arr.reshape(-1, 3)
                        return arr
                break
            if in_data and data_type == 'ascii':
                vals = line.split()
                if len(vals) >= 3:
                    try:
                        pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
                    except ValueError:
                        pass

    # ASCII data read line by line after DATA header
    if data_type == 'ascii' and not pts:
        with open(path, 'r', errors='ignore') as f:
            past_header = False
            for line in f:
                line = line.strip()
                if line.upper().startswith('DATA'):
                    past_header = True
                    continue
                if past_header:
                    vals = line.split()
                    if len(vals) >= 3:
                        try:
                            pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
                        except ValueError:
                            pass

    return np.array(pts, dtype=np.float32) if pts else np.empty((0, 3))


def plot_bev(map_pts, traj_pts, out_path, dpi=200, point_size=0.3,
             map_name="Town10HD — LIO-SAM Global Map"):
    """Render a top-down view and save as PNG."""

    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#0d0d0d')
    ax.set_facecolor('#0d0d0d')

    # ── Map points coloured by height ────────────────────────────────────────
    z = map_pts[:, 2]
    z_min, z_max = np.percentile(z, 2), np.percentile(z, 98)  # robust range
    z_norm = np.clip((z - z_min) / (z_max - z_min + 1e-6), 0, 1)

    # Custom colourmap: dark blue → cyan → yellow → white
    cmap = plt.get_cmap('plasma')
    colours = cmap(z_norm)

    scatter = ax.scatter(
        map_pts[:, 0], map_pts[:, 1],
        c=z_norm, cmap='plasma',
        s=point_size, linewidths=0,
        alpha=0.7, rasterized=True)

    # ── Trajectory ────────────────────────────────────────────────────────────
    if traj_pts is not None and len(traj_pts) > 1:
        # Draw trajectory as gradient line (start=green, end=red)
        x, y = traj_pts[:, 0], traj_pts[:, 1]
        n = len(x)
        t_norm = np.linspace(0, 1, n)

        # Build segments
        pts_line = np.column_stack([x, y]).reshape(-1, 1, 2)
        segments = np.concatenate([pts_line[:-1], pts_line[1:]], axis=1)
        t_seg    = (t_norm[:-1] + t_norm[1:]) / 2

        lc = LineCollection(segments, cmap='RdYlGn_r',
                            linewidth=2.5, alpha=0.95, zorder=5)
        lc.set_array(t_seg)
        ax.add_collection(lc)

        # Start marker (green circle)
        ax.scatter(x[0], y[0], s=120, c='lime', zorder=10,
                   edgecolors='white', linewidths=1.5, label='Start')
        # End marker (red star)
        ax.scatter(x[-1], y[-1], s=180, c='red', marker='*', zorder=10,
                   edgecolors='white', linewidths=1.0, label='End')

    # ── Colorbar ──────────────────────────────────────────────────────────────
    cbar = fig.colorbar(scatter, ax=ax, fraction=0.025, pad=0.01)
    cbar.set_label('Height (m)', color='white', fontsize=11)
    cbar.ax.yaxis.set_tick_params(color='white')
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color='white')
    # Re-map colorbar ticks to real Z values
    n_ticks = 5
    tick_vals = np.linspace(0, 1, n_ticks)
    tick_labels = [f'{z_min + v*(z_max-z_min):.1f}m' for v in tick_vals]
    cbar.set_ticks(tick_vals)
    cbar.set_ticklabels(tick_labels)

    # ── Labels and styling ────────────────────────────────────────────────────
    ax.set_xlabel('X (m) — Forward', color='white', fontsize=12)
    ax.set_ylabel('Y (m) — Left',    color='white', fontsize=12)
    ax.tick_params(colors='white')
    ax.spines[:].set_color('#444444')

    ax.set_title(
        f'{map_name}\n'
        f'{len(map_pts):,} pts · '
        f'{map_pts[:,0].ptp():.0f}m × {map_pts[:,1].ptp():.0f}m footprint',
        color='white', fontsize=13, pad=12)

    ax.set_aspect('equal')
    ax.grid(True, color='#333333', linewidth=0.4, alpha=0.5)

    if traj_pts is not None and len(traj_pts) > 1:
        legend = ax.legend(loc='upper right', framealpha=0.3,
                           labelcolor='white', facecolor='#111111',
                           edgecolor='#555555', fontsize=10)

    # ── Stats box ─────────────────────────────────────────────────────────────
    stats = (
        f"Map extent:\n"
        f"  X: [{map_pts[:,0].min():.0f}, {map_pts[:,0].max():.0f}] m\n"
        f"  Y: [{map_pts[:,1].min():.0f}, {map_pts[:,1].max():.0f}] m\n"
        f"  Z: [{map_pts[:,2].min():.1f}, {map_pts[:,2].max():.1f}] m\n"
    )
    if traj_pts is not None:
        stats += f"Keyframes: {len(traj_pts)}"

    ax.text(0.01, 0.01, stats, transform=ax.transAxes,
            fontsize=9, color='#aaaaaa', verticalalignment='bottom',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#111111', alpha=0.6,
                      edgecolor='#444444'))

    plt.tight_layout()
    fig.savefig(out_path, dpi=dpi, facecolor=fig.get_facecolor(),
                bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {out_path}  ({dpi} DPI)")


def main():
    parser = argparse.ArgumentParser(description='BEV map from LIO-SAM PCD')
    parser.add_argument('--map',  default=DEFAULT_MAP,  help='Path to GlobalMap.pcd')
    parser.add_argument('--traj', default=DEFAULT_TRAJ, help='Path to trajectory.pcd')
    parser.add_argument('--out',  default=DEFAULT_OUT,  help='Output PNG path')
    parser.add_argument('--dpi',  type=int, default=200, help='Output resolution (default: 200)')
    parser.add_argument('--pts',  type=float, default=0.3, help='Scatter point size (default: 0.3)')
    parser.add_argument('--voxel', type=float, default=0.0,
                        help='Voxel downsample before plotting (0=none)')
    args = parser.parse_args()

    map_path  = os.path.expanduser(args.map)
    traj_path = os.path.expanduser(args.traj)
    out_path  = os.path.expanduser(args.out)

    if not os.path.exists(map_path):
        print(f"ERROR: map file not found: {map_path}"); sys.exit(1)

    print(f"Loading map:  {map_path}")
    map_pts = read_pcd_xyz(map_path)
    print(f"  {len(map_pts):,} points loaded.")

    # Optional voxel downsample (speeds up plotting for very dense maps)
    if args.voxel > 0 and len(map_pts) > 0:
        try:
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(map_pts)
            pcd = pcd.voxel_down_sample(args.voxel)
            map_pts = np.asarray(pcd.points)
            print(f"  After {args.voxel}m voxel downsample: {len(map_pts):,} points.")
        except ImportError:
            print("  (open3d not available for voxel downsample — skipping)")

    traj_pts = None
    if os.path.exists(traj_path):
        print(f"Loading traj: {traj_path}")
        traj_pts = read_pcd_xyz(traj_path)
        print(f"  {len(traj_pts)} keyframes loaded.")
    else:
        print(f"No trajectory file found at {traj_path} — skipping overlay.")

    print(f"Rendering BEV at {args.dpi} DPI ...")
    plot_bev(map_pts, traj_pts, out_path, dpi=args.dpi, point_size=args.pts)
    print("Done.")


if __name__ == '__main__':
    main()
