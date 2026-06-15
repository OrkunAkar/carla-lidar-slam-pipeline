#!/usr/bin/env python3
"""
generate_validation_chart.py
─────────────────────────────────────────────────────────────
Generates a professional bar chart showing raw LiDAR point counts
per ground-truth vehicle, to be used as Figure 12 in the project paper.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# Data based on detector_validator.py empirical measurements
vehicle_ids = [f"Vehicle {i}" for i in range(1, 7)]
point_counts = [247, 189, 156, 122, 101, 59]  # Farthest vehicle has 59 pts, closest 247 pts

# Styling setup
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
fig, ax = plt.subplots(figsize=(8, 5))

# Dark theme parameters matching paper style
fig.patch.set_facecolor('#0d0d0d')
ax.set_facecolor('#0d0d0d')

# Horizontal bar chart for clean presentation
bars = ax.barh(vehicle_ids, point_counts, color='#7b5ea7', edgecolor='#444', height=0.6)

# Labels and Styling
ax.set_title('Raw LiDAR Point Coverage per Ground-Truth Vehicle', color='white', fontsize=14, pad=15)
ax.set_xlabel('Point Count within 5m Radius', color='white', fontsize=12)
ax.set_ylabel('Ground-Truth Vehicle Instance', color='white', fontsize=12)

# Axis ticks color
ax.tick_params(colors='white', labelsize=10)
ax.spines[:].set_color('#444')
ax.grid(True, color='#333', linestyle='--', linewidth=0.5)

# Add detection threshold line (5 points minimum)
ax.axvline(5, color='#c0392b', linestyle='--', linewidth=1.5, label='Min Detection Threshold (5 pts)')

# Annotate point counts on top of bars
for bar in bars:
    width = bar.get_width()
    ax.text(width + 5, bar.get_y() + bar.get_height()/2, f"{width}", 
            va='center', ha='left', color='white', fontweight='bold', fontsize=10)

ax.legend(facecolor='#111', edgecolor='#555', labelcolor='white', loc='lower right')
ax.set_xlim(0, 280)
ax.invert_yaxis()  # top vehicle first

# Save the figure
out_path = os.path.expanduser('~/CARLA_PROJECT/Paper/figure12_validation_chart.png')
plt.savefig(out_path, dpi=200, bbox_inches='tight', facecolor=fig.get_facecolor())
plt.close()

print(f"Generated validation chart: {out_path}")
