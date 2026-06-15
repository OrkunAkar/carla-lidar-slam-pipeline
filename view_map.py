#!/usr/bin/env python3
"""
view_map.py — Visualise a LIO-SAM exported PCD file using Open3D.

Usage:
    python3 view_map.py <path/to/GlobalMap.pcd>
    python3 view_map.py ~/CARLA_PROJECT/maps/town10hd/GlobalMap.pcd
    python3 view_map.py ~/CARLA_PROJECT/maps/town10hd/GlobalMap.pcd --traj trajectory.pcd

Features:
  - Colour-by-height gradient (blue = low, red = high)
  - Optional trajectory overlay (red line)
  - Point size and background controls
  - Prints map bounding box and point count stats
"""

import sys
import os
import argparse
import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("Open3D not installed. Install with:")
    print("  pip3 install open3d")
    sys.exit(1)


def colour_by_height(pcd):
    """Apply a blue→green→red gradient based on Z height."""
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-6)   # [0, 1]

    # Blue (low) → Green (mid) → Red (high)
    colours = np.zeros((len(z_norm), 3))
    colours[:, 0] = np.clip(2.0 * z_norm - 1.0, 0, 1)        # red channel
    colours[:, 1] = np.clip(1.0 - np.abs(2.0 * z_norm - 1.0), 0, 1)  # green
    colours[:, 2] = np.clip(1.0 - 2.0 * z_norm, 0, 1)        # blue channel

    pcd.colors = o3d.utility.Vector3dVector(colours)
    return pcd


def load_trajectory(traj_path):
    """Load trajectory.pcd keyframe positions and return as a LineSet."""
    traj = o3d.io.read_point_cloud(traj_path)
    pts = np.asarray(traj.points)
    if len(pts) < 2:
        return None

    lines = [[i, i + 1] for i in range(len(pts) - 1)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.paint_uniform_color([1.0, 0.2, 0.2])   # red trajectory
    return line_set


def print_stats(pcd, label="Map"):
    pts = np.asarray(pcd.points)
    bbox = pcd.get_axis_aligned_bounding_box()
    mn = bbox.min_bound
    mx = bbox.max_bound
    print(f"\n{'─'*50}")
    print(f"  {label}")
    print(f"{'─'*50}")
    print(f"  Points       : {len(pts):,}")
    print(f"  X range      : [{mn[0]:.1f}, {mx[0]:.1f}] m  ({mx[0]-mn[0]:.0f}m wide)")
    print(f"  Y range      : [{mn[1]:.1f}, {mx[1]:.1f}] m  ({mx[1]-mn[1]:.0f}m deep)")
    print(f"  Z range      : [{mn[2]:.1f}, {mx[2]:.1f}] m  ({mx[2]-mn[2]:.1f}m tall)")
    print(f"{'─'*50}\n")


def main():
    parser = argparse.ArgumentParser(description='Visualise LIO-SAM PCD map')
    parser.add_argument('pcd_file', help='Path to GlobalMap.pcd')
    parser.add_argument('--traj', default=None,
                        help='Optional path to trajectory.pcd for overlay')
    parser.add_argument('--voxel', type=float, default=0.0,
                        help='Voxel downsample size in metres (0 = no downsample)')
    parser.add_argument('--point-size', type=float, default=1.5,
                        help='Point render size (default: 1.5)')
    args = parser.parse_args()

    pcd_path = os.path.expanduser(args.pcd_file)
    if not os.path.exists(pcd_path):
        print(f"ERROR: File not found: {pcd_path}")
        sys.exit(1)

    print(f"Loading: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f"Loaded {len(pcd.points):,} points.")

    # Optional voxel downsampling for faster rendering
    if args.voxel > 0:
        pcd = pcd.voxel_down_sample(voxel_size=args.voxel)
        print(f"After {args.voxel}m voxel downsample: {len(pcd.points):,} points.")

    print_stats(pcd, label=os.path.basename(pcd_path))

    # Colour by height
    pcd = colour_by_height(pcd)

    # Build geometry list
    geometries = [pcd]

    # Optional trajectory overlay
    if args.traj:
        traj_path = os.path.expanduser(args.traj)
        if os.path.exists(traj_path):
            line_set = load_trajectory(traj_path)
            if line_set:
                geometries.append(line_set)
                traj_pts = np.asarray(
                    o3d.io.read_point_cloud(traj_path).points)
                print(f"Trajectory: {len(traj_pts)} keyframes loaded (red line).")
        else:
            print(f"WARNING: trajectory file not found: {traj_path}")

    # Coordinate frame at origin
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
    geometries.append(frame)

    print("\nControls:")
    print("  Left drag    — rotate")
    print("  Right drag   — pan")
    print("  Scroll       — zoom")
    print("  Q / Esc      — quit")
    print("  P            — screenshot")
    print("  +/-          — point size")
    print("  9            — toggle point colour")
    print("  H            — show all controls\n")

    # Launch viewer
    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"LIO-SAM Map — {os.path.basename(pcd_path)}",
        width=1280,
        height=800,
        point_show_normal=False)


if __name__ == '__main__':
    main()
