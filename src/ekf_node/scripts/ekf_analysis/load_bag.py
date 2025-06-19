#!/usr/bin/env python3

import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion

def load_ekf_and_gt(bag_path, robot_name='turtlebot3_burger'):
    """
    Lädt EKF-Ausgaben und Ground-Truth-Daten aus einer ROS-Bag-Datei.

    Parameter:
    - bag_path: Pfad zur .bag-Datei
    - robot_name: Name des Roboters in der Gazebo-Modelldatenstruktur

    Rückgabe:
    - ekf: np.ndarray mit Spalten [t, x, y, theta, cov_xx, cov_xy, cov_yy]
    - gt: np.ndarray mit Spalten [t, x, y, theta]
    """
    ekf, gt = [], []
    bag = rosbag.Bag(bag_path)

    # EKF-Daten aus dem Topic /ekf_node/ekf_pose extrahieren
    for _, msg, _ in bag.read_messages(topics=['/ekf_node/ekf_pose']):
        t = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Extrahiere relevante Kovarianzwerte (xx, xy, yy)
        cov = msg.pose.covariance
        cov_xx = cov[0]
        cov_xy = cov[1]
        cov_yy = cov[7]

        ekf.append((t, x, y, th, cov_xx, cov_xy, cov_yy))

    # Ground-Truth aus dem Gazebo-Modelle-State extrahieren
    for _, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        if robot_name in msg.name:
            idx = msg.name.index(robot_name)
            p = msg.pose[idx]
            x, y = p.position.x, p.position.y
            q = p.orientation
            _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
            gt.append((t.to_sec(), x, y, th))

    bag.close()
    return np.array(ekf), np.array(gt)
