#!/usr/bin/env python3

import glob
import numpy as np
import yaml
from scipy.spatial import cKDTree

# Eigene Module
from load_bag import load_ekf_and_gt
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
ekf, gt = load_ekf_and_gt(bag_files[0])

# === Zeitliche Interpolation und RMSE-Berechnung ===
gt_interp = interpolate_ground_truth(ekf, gt)
rmse_vals = compute_rmse(ekf, gt_interp)

# === Landmarken filtern: nur nahegelegene verwenden (KD-Tree innerhalb 3 m) ===
ekf_xy = ekf[:, 1:3]
landmarks_xy = np.array([[lm["x"], lm["y"]] for lm in landmarks])
tree = cKDTree(landmarks_xy)

indices = set()
for point in ekf_xy:
    nearby = tree.query_ball_point(point, r=3)
    indices.update(nearby)

filtered_landmarks = landmarks_xy[list(indices)]

# === Visualisierung ===
plot_trajectory(ekf, gt_interp, rmse_vals,
                landmarks=filtered_landmarks, num_landmarks=len(filtered_landmarks))
plot_orientation(ekf, gt_interp, rmse_vals)

# === Fehlerverläufe berechnen ===
t_rel = ekf[:, 0] - ekf[0, 0]
x_err = ekf[:, 1] - gt_interp[:, 1]
y_err = ekf[:, 2] - gt_interp[:, 2]
theta_err = np.arctan2(np.sin(ekf[:, 3] - gt_interp[:, 3]), np.cos(ekf[:, 3] - gt_interp[:, 3]))

plot_position_errors(t_rel, x_err, y_err)
plot_orientation_errors(t_rel, theta_err)
