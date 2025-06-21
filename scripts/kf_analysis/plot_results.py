#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt

def plot_trajectory(kf, gt_interp, rmse_vals, save_path="export"):
    """
    Zeichnet Trajektorien (KF vs. Ground Truth), Start-/Endpunkte.
    """
    x_k, y_k = kf[:, 1], kf[:, 2]
    x_g, y_g = gt_interp[:, 1], gt_interp[:, 2]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(x_g, y_g, 'b.', markersize=2, label='GT')
    ax.plot(x_k, y_k, 'r-', linewidth=1, label='KF')
    ax.plot(x_k[0], y_k[0], 'go', label='Start')
    ax.plot(x_k[-1], y_k[-1], 'ro', label='Ende')

    ax.legend()
    ax.set_title('Trajektorie')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')

    ax.text(0.98, 0.02,
            f"RMSE X: {rmse_vals[0]*1000:.1f} mm\nRMSE Y: {rmse_vals[1]*1000:.1f} mm",
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=9, bbox=dict(facecolor='white', edgecolor='gray', boxstyle='round,pad=0.3'))

    os.makedirs(save_path, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, "trajectory_kf.png"), dpi=300)
    plt.close()


def plot_orientation(kf, gt_interp, rmse_vals, save_path="export"):
    """
    Zeichnet Yaw-Winkel (KF vs. Ground Truth) über die Zeit.
    """
    t_rel = kf[:, 0] - kf[0, 0]
    theta_k = kf[:, 3]
    theta_g = gt_interp[:, 3]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t_rel, np.degrees(theta_g), 'b.', markersize=2, label='GT')
    ax.plot(t_rel, np.degrees(theta_k), 'r-', linewidth=1, label='KF')
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
    plt.savefig(os.path.join(save_path, "orientation_kf.png"), dpi=300)
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
    os.makedirs(save_path, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, "position_errors_kf.png"), dpi=300)
    plt.close()


def plot_orientation_errors(t_rel, theta_err, save_path="export"):
    """
    Zeichnet den Orientierungsfehler über die Zeit in Grad.
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
    plt.savefig(os.path.join(save_path, "orientation_errors_kf.png"), dpi=300)
    plt.close()
