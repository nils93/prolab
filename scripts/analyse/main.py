#!/usr/bin/env python3

import os
import glob
import argparse
import numpy as np
import yaml
from scipy.spatial import cKDTree

from load_bag import load_pose_from_bag
from compute_metrics import interpolate_ground_truth, compute_rmse
from plot_results import plot_trajectory, plot_trajectory_no_landmarks, plot_orientation, plot_position_errors, plot_orientation_errors

# === Argumente: --filter pf|ekf|kf ===
parser = argparse.ArgumentParser()
parser.add_argument("--filter", required=True, choices=["pf", "ekf", "kf", "ekf_wlm"],
                    help="Filtertyp auswählen: pf | ekf | kf | ekf_wlm" )
args = parser.parse_args()
filter_dir = args.filter

# === Landmarken laden ===
with open("landmark_map.yaml", "r") as f:
    lm_data = yaml.safe_load(f)
landmarks = lm_data["landmarks"]

# === ROS-Bag-Datei interaktiv auswählen ===
bag_files = glob.glob(os.path.join(filter_dir, "*.bag"))
if not bag_files:
    print("❌ Keine .bag-Dateien gefunden.")
    exit(1)

print("Verfügbare .bag-Dateien:")
for i, file in enumerate(bag_files, 1):
    print(f"[{i}] {file}")

try:
    idx = int(input("Welche Datei analysieren? [Zahl] ")) - 1
    bag_file = bag_files[idx]
except (IndexError, ValueError):
    print("❌ Ungültige Auswahl.")
    exit(1)

# === ROS-Bag-Datei laden ===
bag_file = bag_files[idx]
pose, gt, pose_topic, gt_topic = load_pose_from_bag(bag_file, filter_type=args.filter)

# === Exportverzeichnis festlegen ===
bag_basename = os.path.splitext(os.path.basename(bag_file))[0]
export_dir = os.path.join(filter_dir, "export", f"EXPORT_{bag_basename}")

# === Interpolation und RMSE ===
gt_interp = interpolate_ground_truth(pose, gt)
rmse_vals = compute_rmse(pose, gt_interp)

# === Landmarken in der Nähe finden ===
pose_xy = pose[:, 1:3]
landmarks_xy = np.array([[lm["x"], lm["y"]] for lm in landmarks])
tree = cKDTree(landmarks_xy)
indices = set()
for point in pose_xy:
    indices.update(tree.query_ball_point(point, r=3))
filtered_landmarks = landmarks_xy[list(indices)]

# === Plots ===
plot_trajectory(pose, gt_interp, rmse_vals,
                landmarks=filtered_landmarks, num_landmarks=len(filtered_landmarks),
                save_path=export_dir, pf_topic=pose_topic, gt_topic=gt_topic)

plot_trajectory_no_landmarks(pose, gt_interp, rmse_vals, 
                save_path=export_dir, pf_topic=pose_topic, gt_topic=gt_topic)

plot_orientation(pose, gt_interp, rmse_vals,
                 save_path=export_dir, pf_topic=pose_topic, gt_topic=gt_topic)

t_rel = pose[:, 0] - pose[0, 0]
x_err = pose[:, 1] - gt_interp[:, 1]
y_err = pose[:, 2] - gt_interp[:, 2]
theta_err = np.arctan2(np.sin(pose[:, 3] - gt_interp[:, 3]),
                       np.cos(pose[:, 3] - gt_interp[:, 3]))
theta_err = np.degrees(theta_err)

plot_position_errors(t_rel, x_err, y_err, save_path=export_dir)
plot_orientation_errors(t_rel, theta_err, save_path=export_dir)

print(f"✅ Plots gespeichert unter: {export_dir}")
