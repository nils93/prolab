#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_trajectory(ekf, gt_interp, rmse_vals, landmarks=None, num_landmarks=None, save_path="export"):
    """
    Zeichnet Trajektorien (EKF vs. Ground Truth), Start-/Endpunkte, Landmarken und Kovarianz-Ellipsen.
    """
    x_e, y_e = ekf[:, 1], ekf[:, 2]
    x_g, y_g = gt_interp[:, 1], gt_interp[:, 2]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(x_g, y_g, 'b.', markersize=2, label='GT')
    ax.plot(x_e, y_e, 'r-', linewidth=1, label='EKF')
    ax.plot(x_e[0], y_e[0], 'go', label='Start')
    ax.plot(x_e[-1], y_e[-1], 'ro', label='Ende')

    if landmarks is not None:
        landmarks = np.array(landmarks)
        ax.plot(landmarks[:, 0], landmarks[:, 1], 'kx', label='Landmarks')

    # Zeichne Ellipsen in regelmäßigen Abständen
    for i in range(0, len(ekf), 100):
        cov = np.array([[ekf[i, 4], ekf[i, 5]],
                        [ekf[i, 5], ekf[i, 6]]])
        draw_covariance_ellipse(ax, ekf[i, 1], ekf[i, 2], cov,
                                n_std=2, alpha=0.2, edgecolor='k', facecolor='none')

    # Dummy-Eintrag für Ellipsen in Legende
    ellipse_dummy = plt.Line2D([0], [0], color='gray', alpha=0.3, linewidth=5, label='95% Konfidenz')
    handles, labels = ax.get_legend_handles_labels()
    handles.append(ellipse_dummy)
    ax.legend(handles=handles)

    # RMSE-Textbox
    landmark_info = f"\nLandmarks: {num_landmarks}" if num_landmarks is not None else ""
    ax.text(0.98, 0.02,
            f"RMSE X: {rmse_vals[0]*1000:.1f} mm\nRMSE Y: {rmse_vals[1]*1000:.1f} mm{landmark_info}",
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


def plot_orientation(ekf, gt_interp, rmse_vals, save_path="export"):
    """
    Zeichnet Yaw-Winkel (EKF vs. Ground Truth) über die Zeit.
    """
    t_rel = ekf[:, 0] - ekf[0, 0]
    theta_e = ekf[:, 3]
    theta_g = gt_interp[:, 3]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t_rel, np.degrees(theta_g), 'b.', markersize=2, label='GT')
    ax.plot(t_rel, np.degrees(theta_e), 'r-', linewidth=1, label='EKF')
    ax.set_title('Orientierung über Zeit')
    ax.set_xlabel('Zeit [s]')
    ax.set_ylabel('Yaw [°]')
    ax.legend()

    ax.text(0.98, 0.02,
            f"RMSE θ: {np.degrees(rmse_vals[2]):.2f}°",
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
    Zeichnet den Orientierungsfehler über die Zeit.
    """
    plt.figure(figsize=(10, 3.5))
    plt.plot(t_rel, theta_err, label='θ-Fehler [rad]')
    plt.title('Orientierungsfehler über Zeit')
    plt.xlabel('Zeit [s]')
    plt.ylabel('Fehler [rad]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    os.makedirs(save_path, exist_ok=True)
    plt.savefig(os.path.join(save_path, "orientation_errors.png"), dpi=300)
    plt.close()


def draw_covariance_ellipse(ax, x, y, cov, n_std=1.0, **kwargs):
    """
    Zeichnet eine Konfidenzellipse aus der 2x2-Kovarianzmatrix.
    """
    if cov.shape != (2, 2):
        return

    vals, vecs = np.linalg.eigh(cov)
    angle = np.degrees(np.arctan2(*vecs[:, 1][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)

    ellipse = patches.Ellipse((x, y), width, height, angle, **kwargs)
    ax.add_patch(ellipse)
