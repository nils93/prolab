#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_trajectory(pose, gt_interp, rmse_vals, landmarks=None, num_landmarks=None, save_path="export", pf_topic="Filter", gt_topic="GT"):
    """
    Zeichnet Trajektorien (Filter vs. Ground Truth), Start-/Endpunkte und Landmarken.
    """
    x_p, y_p = pose[:, 1], pose[:, 2]
    x_g, y_g = gt_interp[:, 1], gt_interp[:, 2]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(x_g, y_g, 'b.', markersize=2, label=f'{gt_topic}')
    ax.plot(x_p, y_p, 'r-', linewidth=1, label=f'{pf_topic}')
    ax.plot(x_p[0], y_p[0], 'go', label='Start')
    ax.plot(x_p[-1], y_p[-1], 'ro', label='Ende')

    if landmarks is not None:
        lm = np.array(landmarks)
        ax.plot(lm[:, 0], lm[:, 1], 'kx', label='Landmarks')

    ax.legend()

    lm_info = f"\nLandmarks: {num_landmarks}" if num_landmarks is not None else ""
    ax.text(0.98, 0.02,
            f"RMSE X: {rmse_vals[0]*1000:.1f} mm\nRMSE Y: {rmse_vals[1]*1000:.1f} mm{lm_info}",
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=9, bbox=dict(facecolor='white', edgecolor='gray', boxstyle='round,pad=0.3'))

    ax.set_title('Trajektorie')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')

    os.makedirs(save_path, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, "trajectory.png"), dpi=300)
    plt.close()


def plot_trajectory_no_landmarks(pose, gt_interp, rmse_vals, save_path="export", pf_topic="Filter", gt_topic="GT"):
    """
    Zeichnet Trajektorien (Filter vs. Ground Truth), Start-/Endpunkte ohne Landmarken.
    """
    x_p, y_p = pose[:, 1], pose[:, 2]
    x_g, y_g = gt_interp[:, 1], gt_interp[:, 2]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(x_g, y_g, 'b.', markersize=2, label=f'{gt_topic}')
    ax.plot(x_p, y_p, 'r-', linewidth=1, label=f'{pf_topic}')
    ax.plot(x_p[0], y_p[0], 'go', label='Start')
    ax.plot(x_p[-1], y_p[-1], 'ro', label='Ende')

    ax.legend()

    ax.text(0.98, 0.02,
            f"RMSE X: {rmse_vals[0]*1000:.1f} mm\nRMSE Y: {rmse_vals[1]*1000:.1f} mm",
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=9, bbox=dict(facecolor='white', edgecolor='gray', boxstyle='round,pad=0.3'))

    ax.set_title('Trajektorie (ohne Landmarken)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')

    os.makedirs(save_path, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, "trajectory_no_lm.png"), dpi=300)
    plt.close()


def plot_orientation(pose, gt_interp, rmse_vals, save_path="export", pf_topic="Filter", gt_topic="GT"):
    """
    Zeichnet Yaw-Winkel (Filter vs. Ground Truth) über die Zeit.
    """
    t_rel = pose[:, 0] - pose[0, 0]
    theta_p = pose[:, 3]
    theta_g = gt_interp[:, 3]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t_rel, np.degrees(theta_g), 'b.', markersize=2, label=f'{gt_topic}')
    ax.plot(t_rel, np.degrees(theta_p), 'r-', linewidth=1, label=f'{pf_topic}')
    ax.set_title('Orientierung über Zeit')
    ax.set_xlabel('Zeit [s]')
    ax.set_ylabel('Yaw [°]')
    ax.legend()

    ax.text(0.98, 0.02,
            f"RMSE θ: {rmse_vals[2]:.2f}°",
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=9, bbox=dict(facecolor='white', edgecolor='gray', boxstyle='round,pad=0.3'))

    os.makedirs(save_path, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, "orientation.png"), dpi=300)
    plt.close()


def plot_position_errors(t_rel, x_err, y_err, save_path="export"):
    """
    Zeichnet den Positionsfehler (x und y) über die Zeit.
    """
    plt.figure(figsize=(10, 3.5))
    plt.plot(t_rel, x_err, label='x-Fehler [m]')
    plt.plot(t_rel, y_err, label='y-Fehler [m]')
    plt.title('Positionsfehler über Zeit')
    plt.xlabel('Zeit [s]')
    plt.ylabel('Fehler [m]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    os.makedirs(save_path, exist_ok=True)
    plt.savefig(os.path.join(save_path, "position_errors.png"), dpi=300)
    plt.close()


def plot_orientation_errors(t_rel, theta_err, save_path="export"):
    """
    Zeichnet den Orientierungsfehler über die Zeit (in Grad).
    """
    plt.figure(figsize=(10, 3.5))
    plt.plot(t_rel, theta_err, label='θ-Fehler [°]')
    plt.title('Orientierungsfehler über Zeit')
    plt.xlabel('Zeit [s]')
    plt.ylabel('Fehler [°]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    os.makedirs(save_path, exist_ok=True)
    plt.savefig(os.path.join(save_path, "orientation_errors.png"), dpi=300)
    plt.close()


def draw_covariance_ellipse(ax, x, y, cov, n_std=1.0, **kwargs):
    """
    Zeichnet eine Konfidenzellipse.
    """
    if cov.shape != (2, 2):
        return

    vals, vecs = np.linalg.eigh(cov)
    angle = np.degrees(np.arctan2(*vecs[:, 1][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)

    ellipse = patches.Ellipse((x, y), width, height, angle, **kwargs)
    ax.add_patch(ellipse)
