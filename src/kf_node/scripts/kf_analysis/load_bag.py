#!/usr/bin/env python3

import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion

def load_kf_and_gt(bag_path, robot_name='turtlebot3_burger'):
    """
    Lädt KF-Ausgaben und Ground-Truth-Daten aus einer ROS-Bag-Datei.

    Parameter:
    - bag_path: Pfad zur .bag-Datei
    - robot_name: Name des Roboters in der Gazebo-Modelldatenstruktur

    Rückgabe:
    - kf: np.ndarray mit Spalten [t, x, y, theta]
    - gt: np.ndarray mit Spalten [t, x, y, theta]
    """
    kf, gt = [], []
    bag = rosbag.Bag(bag_path)

    gt_index = None
    for _, msg, _ in bag.read_messages(topics=['/gazebo/model_states']):
        try:
            gt_index = msg.name.index(robot_name)
            break
        except ValueError:
            continue
    if gt_index is None:
        raise RuntimeError(f"Robot '{robot_name}' nicht in /gazebo/model_states gefunden")

    # KF-Daten aus dem Topic /kf_node/kf_pose extrahieren
    for _, msg, _ in bag.read_messages(topics=['/kf_node/kf_pose']):
        t = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        kf.append((t, x, y, th))

    # Ground-Truth aus dem Gazebo-Modelle-State extrahieren
    for _, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        if robot_name in msg.name:
            idx = msg.name.index(robot_name)
            p = msg.pose[idx]
            q = p.orientation
            _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
            gt.append((t.to_sec(), p.position.x, p.position.y, th))

    bag.close()
    return np.array(kf), np.array(gt)
