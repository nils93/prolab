#!/usr/bin/env python3

import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion

def load_pf_and_gt(bag_path, robot_name='turtlebot3_burger'):
    """
    Lädt PF-Ausgaben und Ground-Truth aus einer ROS-Bag.

    Rückgabe:
    - pf: np.ndarray mit Spalten [t, x, y, theta]
    - gt: np.ndarray mit Spalten [t, x, y, theta]
    """
    pf, gt = [], []
    bag = rosbag.Bag(bag_path)

    # PF-Schätzung aus /pf_node/particle_cloud
    for _, msg, _ in bag.read_messages(topics=['/pf_node/particle_cloud']):
        t = msg.header.stamp.to_sec()
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        yaws = [euler_from_quaternion([p.orientation.x,
                                       p.orientation.y,
                                       p.orientation.z,
                                       p.orientation.w])[2]
                for p in msg.poses]
        pf.append((
            t,
            np.mean(xs),
            np.mean(ys),
            np.arctan2(np.mean(np.sin(yaws)),
                      np.mean(np.cos(yaws)))
        ))

    # Ground-Truth aus /gazebo/model_states
    for _, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        if robot_name in msg.name:
            idx = msg.name.index(robot_name)
            p = msg.pose[idx]
            _, _, th = euler_from_quaternion([
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w
            ])
            gt.append((t.to_sec(),
                       p.position.x,
                       p.position.y,
                       th))

    bag.close()
    return np.array(pf), np.array(gt)
