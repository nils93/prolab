#!/usr/bin/env python3

import rospy, math
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import tf_conversions

if __name__ == '__main__':
    rospy.init_node('sim_landmark_pub')

    # Lade Landmarkenliste aus Parameter-Server (/ekf_node/landmarks)
    lms = rospy.get_param('/ekf_node/landmarks')

    # Publisher für Landmarkenmessungen im EKF-Format
    pub = rospy.Publisher('/ekf_node/landmark_obs',
                          PoseWithCovarianceStamped, queue_size=1)

    # Initialisiere TF-Buffer zum Abrufen der Roboterpose
    tfbuf  = tf2_ros.Buffer()
    tfrecv = tf2_ros.TransformListener(tfbuf)

    rate = rospy.Rate(1.0)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Aktuelle Roboterpose in map-Frame abfragen
            tr = tfbuf.lookup_transform('map', 'base_footprint',
                                        rospy.Time(0), rospy.Duration(0.5))

            x = tr.transform.translation.x
            y = tr.transform.translation.y
            q = tr.transform.rotation
            _, _, theta = tf_conversions.transformations.euler_from_quaternion(
                            [q.x, q.y, q.z, q.w])

            # Schleife über alle Landmarken
            for lm in lms:
                # Messung: Bereich r und Winkel b relativ zum Roboter
                dx = lm['x'] - x
                dy = lm['y'] - y
                r = math.hypot(dx, dy)
                b = math.atan2(dy, dx) - theta

                # Messnachricht aufbauen
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = rospy.Time.now()
                msg.header.frame_id = str(lm['id'])  # Landmarken-ID als Frame-ID

                msg.pose.pose.position.x = r
                msg.pose.pose.position.y = b
                msg.pose.pose.orientation.w = 1.0  # Orientierung wird nicht verwendet

                # Kovarianzmatrix: Unsicherheit nur in r (x) und b (y)
                cov = [0.01 if i in (0, 7) else 0 for i in range(36)]
                msg.pose.covariance = cov

                # Nachricht veröffentlichen
                pub.publish(msg)

            rate.sleep()

        except Exception as e:
            rospy.logwarn_once(f"sim_landmark_pub: {e}")
            rate.sleep()
