#!/usr/bin/env python3

import glob
import numpy as np

# eigene Module
from load_bag import load_kf_and_gt
from compute_metrics import interpolate_ground_truth, compute_rmse
from plot_results import (
    plot_trajectory,
    plot_orientation,
    plot_position_errors,
    plot_orientation_errors
)

# === ROS-Bag laden ===
bag_files = glob.glob("*.bag")
if not bag_files:
    print("❌ Keine .bag-Datei gefunden.")
    exit(1)

# KF- und GT-Daten einlesen
# Erwartet ein Topic /kf_filter/odom (nav_msgs/Odometry) und /gazebo/model_states
kf, gt = load_kf_and_gt(bag_files[0])

# === Interpolation und RMSE ===
gt_interp = interpolate_ground_truth(kf, gt)
rmse_vals = compute_rmse(kf, gt_interp)

# === Visualisierung ===
# Hier ohne Landmarken
plot_trajectory(kf, gt_interp, rmse_vals)
plot_orientation(kf, gt_interp, rmse_vals)

# === Fehlerverläufe ===
t_rel    = kf[:, 0] - kf[0, 0]
x_err    = kf[:, 1] - gt_interp[:, 1]
y_err    = kf[:, 2] - gt_interp[:, 2]
theta_err = np.arctan2(
    np.sin(kf[:, 3] - gt_interp[:, 3]),
    np.cos(kf[:, 3] - gt_interp[:, 3])
)

plot_position_errors(t_rel, x_err, y_err)
plot_orientation_errors(t_rel, theta_err)
