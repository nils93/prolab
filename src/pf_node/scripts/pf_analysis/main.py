#!/usr/bin/env python3

import glob
import numpy as np
import yaml
from scipy.spatial import cKDTree

# Eigene Module
from load_bag import load_pf_and_gt
from compute_metrics import interpolate_ground_truth, compute_rmse
from plot_results import plot_trajectory, plot_orientation, plot_position_errors, plot_orientation_errors

# === Landmarken laden ===
with open("../../config/landmark_map.yaml", "r") as f:
    lm_data = yaml.safe_load(f)
landmarks = lm_data["landmarks"]

# === ROS-Bag-Datei laden ===
bag_files = glob.glob("*.bag")
if not bag_files:
    print("❌ Keine .bag-Datei gefunden.")
    exit(1)
pf, gt = load_pf_and_gt(bag_files[0])

# === Zeitliche Interpolation und RMSE-Berechnung ===
gt_interp = interpolate_ground_truth(pf, gt)
rmse_vals = compute_rmse(pf, gt_interp)

# === Landmarken filtern: nur nahegelegene verwenden (KD-Tree innerhalb 3 m) ===
pf_xy = pf[:, 1:3]
landmarks_xy = np.array([[lm["x"], lm["y"]] for lm in landmarks])
tree = cKDTree(landmarks_xy)

indices = set()
for point in pf_xy:
    nearby = tree.query_ball_point(point, r=3)
    indices.update(nearby)

filtered_landmarks = landmarks_xy[list(indices)]

# === Visualisierung ===
plot_trajectory(pf, gt_interp, rmse_vals,
                landmarks=filtered_landmarks, num_landmarks=len(filtered_landmarks))
plot_orientation(pf, gt_interp, rmse_vals)

# === Fehlerverläufe berechnen ===
t_rel    = pf[:, 0] - pf[0, 0]
x_err    = pf[:, 1] - gt_interp[:, 1]
y_err    = pf[:, 2] - gt_interp[:, 2]
theta_err = np.arctan2(
    np.sin(pf[:, 3] - gt_interp[:, 3]),
    np.cos(pf[:, 3] - gt_interp[:, 3])
)

plot_position_errors(t_rel, x_err, y_err)
plot_orientation_errors(t_rel, theta_err)
