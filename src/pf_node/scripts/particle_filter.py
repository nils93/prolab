#!/usr/bin/env python3
import rospy, math, numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import tf
import tf_conversions
import tf2_ros

class ParticleFilterNode:
    def __init__(self):
        rospy.init_node('particle_filter')
        # Parameter
        self.M = rospy.get_param('~num_particles', 500)
        self.resample_thresh = rospy.get_param('~resample_threshold', 0.5)
        # Motion noise params (alpha1..alpha4 analog ProbRob)
        self.alpha1 = rospy.get_param('~alpha1', 0.1)
        self.alpha2 = rospy.get_param('~alpha2', 0.1)
        self.alpha3 = rospy.get_param('~alpha3', 0.2)
        self.alpha4 = rospy.get_param('~alpha4', 0.2)
        # Landmarks
        self.landmarks = rospy.get_param('~landmarks', [])
        # State: Partikel und Gewichte
        self.particles = np.zeros((self.M,3))   # [x,y,yaw]
        self.weights   = np.ones(self.M)/self.M
        self.last_odom = None
        self.br = tf.TransformBroadcaster()
        # Subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
        self.lm_sub   = rospy.Subscriber('particle_filter/landmark_obs',
                                         PoseWithCovarianceStamped,
                                         self.landmark_cb, queue_size=1)
        # Publisher
        self.pf_pub  = rospy.Publisher('particles', PoseArray, queue_size=1)
        self.est_pub = rospy.Publisher('state_estimate', PoseStamped, queue_size=1)
        rospy.loginfo(f"[PF] initialized with {self.M} particles")

    def odom_cb(self, msg):
        # 1) Prediction Schritt
        pose = msg.pose.pose
        x = pose.position.x; y = pose.position.y
        _,_,yaw = tf_conversions.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        )
        if self.last_odom is None:
            self.last_odom = (x,y,yaw)
            return
        x0,y0,y0aw = self.last_odom
        # odometry-based control u = (rot1, trans, rot2)
        dx = x - x0; dy = y - y0
        trans = math.hypot(dx,dy)
        rot1  = math.atan2(dy,dx) - y0aw
        rot2  = yaw - y0aw - rot1
        self.last_odom = (x,y,yaw)
        # Noisevariablen
        sigma_rot1 = math.sqrt(self.alpha1*rot1**2 + self.alpha2*trans**2)
        sigma_trans= math.sqrt(self.alpha3*trans**2 + self.alpha4*(rot1**2+rot2**2))
        sigma_rot2 = math.sqrt(self.alpha1*rot2**2 + self.alpha2*trans**2)
        # Partikel update
        for i in range(self.M):
            r1 = rot1  + np.random.normal(0, sigma_rot1)
            dt = trans + np.random.normal(0, sigma_trans)
            r2 = rot2  + np.random.normal(0, sigma_rot2)
            x_i, y_i, th_i = self.particles[i]
            x_i += dt * math.cos(th_i + r1)
            y_i += dt * math.sin(th_i + r1)
            th_i += r1 + r2
            self.particles[i] = [x_i, y_i, self.normalize_angle(th_i)]
        # direkt Partikel visualisieren
        self.publish_particles(msg.header)

    def landmark_cb(self, msg):
        # 2) Messungsupdate
        # Landmarken-ID aus frame_id
        lm_id = int(msg.header.frame_id)
        lm = next((l for l in self.landmarks if l['id']==lm_id), None)
        if lm is None: return
        r_meas = msg.pose.pose.position.x
        b_meas = msg.pose.pose.position.y
        cov = msg.pose.covariance
        sigma_r = math.sqrt(cov[0])
        sigma_b = math.sqrt(cov[7])
        # Likelihood für jedes Partikel
        for i in range(self.M):
            x_i,y_i,th_i = self.particles[i]
            dx = lm['x'] - x_i
            dy = lm['y'] - y_i
            r_exp = math.hypot(dx,dy)
            b_exp = math.atan2(dy,dx) - th_i
            # zweidimensionaler Gauß
            dr = r_meas - r_exp
            db = self.normalize_angle(b_meas - b_exp)
            p_r = self.gauss(dr, sigma_r)
            p_b = self.gauss(db, sigma_b)
            self.weights[i] *= (p_r * p_b)
        # normalize
        self.weights += 1e-300      # avoid zeros
        self.weights /= np.sum(self.weights)
        # 3) Resample falls nötig
        if 1.0/np.sum(self.weights**2) < self.resample_thresh * self.M:
            self.resample()
        # estimate & visualize
        self.publish_estimate(msg.header)
        self.publish_particles(msg.header)

    def resample(self):
        """Low-variance resampling"""
        new_particles = np.zeros_like(self.particles)
        c = np.cumsum(self.weights)
        r = np.random.uniform(0, 1/self.M)
        i = 0
        for m in range(self.M):
            u = r + m*(1.0/self.M)
            while u > c[i]:
                i += 1
            new_particles[m] = self.particles[i]
        self.particles = new_particles
        self.weights.fill(1.0/self.M)

    def publish_particles(self, header):
        pa = PoseArray()
        pa.header.stamp = header.stamp
        pa.header.frame_id = "map"
        pa.poses = []
        for x,y,th in self.particles:
            q = tf.transformations.quaternion_from_euler(0,0,th)
            p = Pose(Point(x,y,0), Quaternion(*q))
            pa.poses.append(p)
        self.pf_pub.publish(pa)

    def publish_estimate(self, header):
        # gewichtetes Mittel
        x = np.average(self.particles[:,0], weights=self.weights)
        y = np.average(self.particles[:,1], weights=self.weights)
        # für Yaw: Zirkuläres Mittel
        siny = np.average(np.sin(self.particles[:,2]), weights=self.weights)
        cosy = np.average(np.cos(self.particles[:,2]), weights=self.weights)
        th= math.atan2(siny, cosy)
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = header.stamp
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0,0,th)
        ps.pose.orientation = Quaternion(*q)
        self.est_pub.publish(ps)
        # optional: TF broadcast map->base_link
        self.br.sendTransform((x,y,0), q, header.stamp, "base_link", "map")

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def gauss(e, sigma):
        return math.exp(-0.5*(e**2)/(sigma**2)) / (math.sqrt(2*math.pi)*sigma)

if __name__=='__main__':
    try:
        ParticleFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
