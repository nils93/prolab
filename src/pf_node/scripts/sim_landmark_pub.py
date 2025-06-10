#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import tf_conversions

if __name__ == '__main__':
    rospy.init_node('sim_landmark_pub')

    # Lade Landmarkenliste aus Parameter-Server (/particle_filter/landmarks)
    # Format: [{ "id": 0, "x": 1.23, "y": 4.56 }, …]
    lms = rospy.get_param('/pf_node/landmarks')

    # Publisher für Landmarken-Messungen im Particle-Filter-Format
    pub = rospy.Publisher('/pf_node/landmark_obs',
                          PoseWithCovarianceStamped, queue_size=1)

    # TF-Buffer, um die aktuelle Roboterpose aus dem map→base_link-Transform zu holen
    tfbuf  = tf2_ros.Buffer()
    tfrecv = tf2_ros.TransformListener(tfbuf)

    rate = rospy.Rate(1.0)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Aktuelle Roboterpose im map-Frame abrufen
            tr = tfbuf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.5))

            x = tr.transform.translation.x
            y = tr.transform.translation.y
            q = tr.transform.rotation
            _, _, theta = tf_conversions.transformations.euler_from_quaternion(
                            [q.x, q.y, q.z, q.w])

            # Für jede Landmarke eine Messung simulieren
            for lm in lms:
                # Distanz r und Rel-Winkel b
                dx = lm['x'] - x
                dy = lm['y'] - y
                r = math.hypot(dx, dy)
                b = math.atan2(dy, dx) - theta

                # Baue PoseWithCovarianceStamped auf
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = rospy.Time.now()
                # Landmarken-ID als frame_id, so kann der PF-Node sie auseinanderhalten
                msg.header.frame_id = str(lm['id'])

                # Wir packen r in pos.x, b in pos.y
                msg.pose.pose.position.x = r
                msg.pose.pose.position.y = b
                # keine Orientierung nötig
                msg.pose.pose.orientation.w = 1.0

                # Kovarianz: Varianz für r und b, alles andere 0
                cov = [0.01 if i in (0, 7) else 0 for i in range(36)]
                #cov = [0.0]*36
                #cov[0] = 0.01    # Var(r)
                #cov[7] = 0.01    # Var(b)
                msg.pose.covariance = cov

                # veröffentlichen
                pub.publish(msg)

            rate.sleep()

        except Exception as e:
            rospy.logwarn_once(f"[sim_landmark_pub] TF-Error: {e}")
            rate.sleep()
