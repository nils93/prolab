#!/usr/bin/env python3

import numpy as np

def interpolate_ground_truth(kf, gt):
    """
    Interpoliert die Ground-Truth-Daten auf die Zeitstempel der KF-Ausgabe.

    Parameter:
    - kf: np.ndarray mit Spalten [t, x, y, theta], Zeitreihe vom Kalman-Filter
    - gt: np.ndarray mit Spalten [t, x, y, theta], Zeitreihe der Ground Truth

    Rückgabe:
    - gt_interp: np.ndarray mit interpolierten Ground-Truth-Werten zur KF-Zeitbasis
    """
    t_kf = kf[:, 0]
    t_gt, x_gt, y_gt, th_gt = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]
    
    # Interpolation der Positionen und der Orientierung
    x_i  = np.interp(t_kf, t_gt, x_gt)
    y_i  = np.interp(t_kf, t_gt, y_gt)
    th_i = np.interp(t_kf, t_gt, th_gt)

    return np.stack((t_kf, x_i, y_i, th_i), axis=1)


def compute_rmse(kf, gt_interp):
    """
    Berechnet den Root Mean Square Error (RMSE) zwischen KF-Ausgabe und Ground Truth.

    Parameter:
    - kf:        np.ndarray mit Spalten [t, x, y, theta]
    - gt_interp: np.ndarray, interpolierte Ground Truth mit gleichen Zeitstempeln

    Rückgabe:
    - Tuple aus RMSE-Werten für x, y und theta
    """
    dx  = kf[:, 1] - gt_interp[:, 1]
    dy  = kf[:, 2] - gt_interp[:, 2]
    dth = np.arctan2(
        np.sin(kf[:, 3] - gt_interp[:, 3]),
        np.cos(kf[:, 3] - gt_interp[:, 3])
    )

    rmse_x     = np.sqrt(np.mean(dx ** 2))
    rmse_y     = np.sqrt(np.mean(dy ** 2))
    rmse_theta = np.sqrt(np.mean(dth ** 2))

    return rmse_x, rmse_y, rmse_theta
