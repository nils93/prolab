#!/usr/bin/env python3

import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion

def load_pose_from_bag(bag_path, filter_type, robot_name='turtlebot3_burger'):
    """
    Lädt Pose-Daten (PF, EKF oder KF) + Ground Truth aus der Bag-Datei.
    Rückgabe:
    - pose: np.ndarray [t, x, y, theta]
    - gt:   np.ndarray [t, x, y, theta]
    - pose_topic, gt_topic: Topicnamen als String
    """
    pose, gt = [], []
    bag = rosbag.Bag(bag_path)
    available_topics = bag.get_type_and_topic_info()[1].keys()

    print("Verfügbare Topics:")
    for t in available_topics:
        print(f"  - {t}")

    # === Topic-Erkennung ===
    pose_topic = None
    for t in available_topics:
        if filter_type == "pf" and "pf_particles" in t:
            pose_topic = t
            break
        elif filter_type == "ekf" and "ekf_pose" in t:
            pose_topic = t
            break
        elif filter_type == "kf" and "kf_pose" in t:
            pose_topic = t
            break
        elif filter_type == "ekf_wlm" and "ekf_pose_wlm" in t:
            pose_topic = t
            break

    gt_topic = next((t for t in available_topics if "model_states" in t), None)

    # === Pose lesen ===
    if pose_topic is None:
        print(f"❌ Kein Topic für '{filter_type}' gefunden.")
    else:
        for _, msg, _ in bag.read_messages(topics=[pose_topic]):
            t = msg.header.stamp.to_sec()

            if hasattr(msg, 'poses'):  # PF: PoseArray
                xs = [p.position.x for p in msg.poses]
                ys = [p.position.y for p in msg.poses]
                yaws = [euler_from_quaternion([
                    p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w])[2]
                        for p in msg.poses]
                pose.append((
                    t,
                    np.mean(xs),
                    np.mean(ys),
                    np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
                ))

            else:  # KF/EKF: Einzelpose
                pose_msg = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
                x = pose_msg.position.x
                y = pose_msg.position.y
                _, _, th = euler_from_quaternion([
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w
                ])
                pose.append((t, x, y, th))

    # === Ground Truth lesen ===
    if gt_topic is None:
        print("❌ Kein Ground-Truth-Topic gefunden.")
    else:
        for _, msg, t in bag.read_messages(topics=[gt_topic]):
            if robot_name in msg.name:
                idx = msg.name.index(robot_name)
                p = msg.pose[idx]
                _, _, th = euler_from_quaternion([
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    p.orientation.w
                ])
                gt.append((t.to_sec(), p.position.x, p.position.y, th))

    bag.close()

    # === Leere Pose/GT abfangen ===
    if not pose or not gt:
        return np.array([]), np.array([]), pose_topic or "?", gt_topic or "?"

    return np.array(pose), np.array(gt), pose_topic or "?", gt_topic or "?"
