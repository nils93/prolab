#!/usr/bin/env python3

import numpy as np

def interpolate_ground_truth(ekf, gt):
    """
    Interpoliert die Ground-Truth-Daten auf die Zeitstempel des EKF-Ausgangs.

    Parameter:
    - ekf: np.ndarray mit Spalten [t, x, y, theta], Zeitreihe vom EKF
    - gt: np.ndarray mit Spalten [t, x, y, theta], Zeitreihe der Ground Truth

    Rückgabe:
    - gt_interp: np.ndarray mit interpolierten Ground-Truth-Werten zur EKF-Zeitbasis
    """
    t_ekf = ekf[:, 0]
    t_gt, x_gt, y_gt, th_gt = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]
    
    # Interpolation der Positionen und der Orientierung
    x_i = np.interp(t_ekf, t_gt, x_gt)
    y_i = np.interp(t_ekf, t_gt, y_gt)
    th_i = np.interp(t_ekf, t_gt, th_gt)

    return np.stack((t_ekf, x_i, y_i, th_i), axis=1)

def compute_rmse(ekf, gt_interp):
    """
    Berechnet den Root Mean Square Error (RMSE) zwischen EKF-Ausgabe und Ground Truth.

    Parameter:
    - ekf: np.ndarray mit Spalten [t, x, y, theta]
    - gt_interp: np.ndarray, interpolierte Ground Truth mit gleichen Zeitstempeln

    Rückgabe:
    - Tuple aus RMSE-Werten für x, y und theta
    """
    dx = ekf[:, 1] - gt_interp[:, 1]
    dy = ekf[:, 2] - gt_interp[:, 2]

    # Korrektur von Rundungsfehlern bei Winkeldifferenz
    dth = np.arctan2(np.sin(ekf[:, 3] - gt_interp[:, 3]), np.cos(ekf[:, 3] - gt_interp[:, 3]))

    # RMSE für x, y, theta
    return np.sqrt(np.mean(dx**2)), np.sqrt(np.mean(dy**2)), np.sqrt(np.mean(dth**2))
