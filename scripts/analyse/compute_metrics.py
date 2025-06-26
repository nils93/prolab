#!/usr/bin/env python3

import numpy as np

def interpolate_ground_truth(pose, gt):
    """
    Interpoliert die Ground-Truth-Daten auf die Zeitstempel der Filter-Ausgabe.

    Parameter:
    - pose: np.ndarray mit Spalten [t, x, y, theta], Zeitreihe des Filters
    - gt:   np.ndarray mit Spalten [t, x, y, theta], Zeitreihe der Ground Truth

    Rückgabe:
    - gt_interp: np.ndarray mit interpolierten GT-Werten zur Filter-Zeitbasis
    """
    t_pose = pose[:, 0]
    t_gt, x_gt, y_gt, th_gt = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]

    x_i  = np.interp(t_pose, t_gt, x_gt)
    y_i  = np.interp(t_pose, t_gt, y_gt)
    th_i = np.interp(t_pose, t_gt, th_gt)

    return np.stack((t_pose, x_i, y_i, th_i), axis=1)


def compute_rmse(pose, gt_interp, return_in_degrees=True):
    """
    Berechnet den Root Mean Square Error (RMSE) zwischen Filter-Ausgabe und Ground Truth.

    Parameter:
    - pose:               np.ndarray mit Spalten [t, x, y, theta]
    - gt_interp:          np.ndarray mit interpolierter Ground Truth
    - return_in_degrees:  bool, ob der Winkel-RMSE in Grad zurückgegeben werden soll

    Rückgabe:
    - Tuple (rmse_x, rmse_y, rmse_theta) – letzterer in Grad, falls aktiviert
    """
    dx  = pose[:, 1] - gt_interp[:, 1]
    dy  = pose[:, 2] - gt_interp[:, 2]
    dth = np.arctan2(
        np.sin(pose[:, 3] - gt_interp[:, 3]),
        np.cos(pose[:, 3] - gt_interp[:, 3])
    )

    rmse_x     = np.sqrt(np.mean(dx ** 2))
    rmse_y     = np.sqrt(np.mean(dy ** 2))
    rmse_theta = np.sqrt(np.mean(dth ** 2))

    if return_in_degrees:
        rmse_theta = np.degrees(rmse_theta)

    return rmse_x, rmse_y, rmse_theta
