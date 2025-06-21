#!/usr/bin/env python3

import numpy as np

def interpolate_ground_truth(pf, gt):
    """
    Interpoliert die Ground-Truth-Daten auf die Zeitstempel der PF-Ausgabe.

    Parameter:
    - pf:   np.ndarray mit Spalten [t, x, y, theta], Zeitreihe des Partikelfilters
    - gt:   np.ndarray mit Spalten [t, x, y, theta], Zeitreihe der Ground Truth

    Rückgabe:
    - gt_interp: np.ndarray mit interpolierten GT-Werten zur PF-Zeitbasis
    """
    t_pf    = pf[:, 0]
    t_gt, x_gt, y_gt, th_gt = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]

    x_i  = np.interp(t_pf, t_gt, x_gt)
    y_i  = np.interp(t_pf, t_gt, y_gt)
    th_i = np.interp(t_pf, t_gt, th_gt)

    return np.stack((t_pf, x_i, y_i, th_i), axis=1)


def compute_rmse(pf, gt_interp):
    """
    Berechnet den Root Mean Square Error (RMSE) zwischen PF-Ausgabe und Ground Truth.

    Parameter:
    - pf:        np.ndarray mit Spalten [t, x, y, theta]
    - gt_interp: np.ndarray, interpolierte Ground Truth (gleiche Zeitbasis wie pf)

    Rückgabe:
    - Tuple (rmse_x, rmse_y, rmse_theta)
    """
    dx  = pf[:, 1] - gt_interp[:, 1]
    dy  = pf[:, 2] - gt_interp[:, 2]
    dth = np.arctan2(
        np.sin(pf[:, 3] - gt_interp[:, 3]),
        np.cos(pf[:, 3] - gt_interp[:, 3])
    )

    rmse_x     = np.sqrt(np.mean(dx ** 2))
    rmse_y     = np.sqrt(np.mean(dy ** 2))
    rmse_theta = np.sqrt(np.mean(dth ** 2))

    return rmse_x, rmse_y, rmse_theta
