#include "ekf_node/ekf_localization.h"

EKFLocalization::EKFLocalization(ros::NodeHandle& nh)
: 
  odom_sub_(nh, "/odom", 1),
  imu_sub_(nh,  "/imu",  1),
  sync_(SyncPolicy(10), odom_sub_, imu_sub_)
{
  // ### Initiale Pose aus TF holen ###
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped t;
  // Warte kurz, bis AMCL die map→odom (und damit map→base_link) publiziert
  ros::Duration(1.0).sleep();
  try {
    t = tfBuffer.lookupTransform("map", "base_link",
                                 ros::Time(0), ros::Duration(1.0));
    // Körperpose in map-Frame in mu_ übernehmen
    double yaw_init = tf2::getYaw(t.transform.rotation);
    mu_ << t.transform.translation.x,
           t.transform.translation.y,
           yaw_init;
    last_time_ = t.header.stamp;

  } catch (tf2::TransformException &ex) {
    ROS_WARN("Initial TF fehlgeschlagen: %s", ex.what());
    // Fallback auf 0,0,0
    mu_.setZero();
    last_time_ = ros::Time::now();
  }
// ### Ende Initialisierung ###

  // Filter-Initialisierung
  Sigma_ = Eigen::Matrix3d::Identity() * 0.1;
  G_     = Eigen::Matrix3d::Identity();        // Bewegungs-Jacobian
  R_     = Eigen::Matrix3d::Identity() * 1e-3; // Prozessrauschen
  //Q_     = Eigen::Matrix2d::Identity() * 1e-2; // Messrauschen
  Q_ = Eigen::Matrix<double, 1, 1>::Constant(1e-2);
  
  // synchronisierten Callback registrieren
  sync_.registerCallback(
    boost::bind(&EKFLocalization::ekfCallback, this, _1, _2)
  );

  // Publisher für gefilterte Pose
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
}

void EKFLocalization::ekfCallback(
  const nav_msgs::Odometry::ConstPtr& odom,
  const sensor_msgs::Imu::ConstPtr& imu)
{
  // 1. Prädiktion
  predict(odom);

  // 2. Messung extrahieren (z = [v; θ])
  double v_meas = odom->twist.twist.linear.x;
  double theta_meas = tf2::getYaw(imu->orientation);
  Eigen::VectorXd z(1);
  z << theta_meas;

  // 3. Korrektur
  update(z);

  // 4. Publikation
  publishPose();
}

void EKFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  double dt = (odom->header.stamp - last_time_).toSec();
  last_time_ = odom->header.stamp;

  double theta = mu_(2);

  // Bewegungsgleichung (g)
  if (std::abs(w) < 1e-5) {
    mu_(0) += v * dt * cos(theta);
    mu_(1) += v * dt * sin(theta);
    // mu_(2) bleibt gleich
  } else {
    mu_(0) += -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
    mu_(1) +=  (v/w) * cos(theta) - (v/w) * cos(theta + w*dt);
    mu_(2) += w * dt;
  }

  // Winkel normalisieren
  mu_(2) = atan2(sin(mu_(2)), cos(mu_(2)));

  // Jacobi-Matrix G_t
  G_ = Eigen::Matrix3d::Identity();
  if (std::abs(w) < 1e-5) {
    G_(0,2) = -v * dt * sin(theta);
    G_(1,2) =  v * dt * cos(theta);
  } else {
    G_(0,2) = (v/w) * cos(theta) - (v/w) * cos(theta + w * dt);
    G_(1,2) = (v/w) * sin(theta) - (v/w) * sin(theta + w * dt);
  }

  // Kovarianz-Update
  Sigma_ = G_ * Sigma_ * G_.transpose() + R_;
}

void EKFLocalization::update(const Eigen::VectorXd& z) {
  // Erwartete Messung h(mu): nur theta
  Eigen::VectorXd h(1);
  h << mu_(2);

  // Ableitung von h nach mu (Messmatrix H_t)
  Eigen::MatrixXd H(1, 3);
  H << 0, 0, 1;

  // Kalman-Gain K_t
  Eigen::MatrixXd S = H * Sigma_ * H.transpose() + Q_;
  Eigen::MatrixXd K = Sigma_ * H.transpose() * S.inverse();

  // Update Schritt
  Eigen::VectorXd innovation = z - h;
  innovation(0) = atan2(sin(innovation(0)), cos(innovation(0)));  // Winkeldifferenz normalisieren

  mu_ = mu_ + K * innovation;
  mu_(2) = atan2(sin(mu_(2)), cos(mu_(2)));  // Winkel wieder normalisieren

  Sigma_ = (Eigen::Matrix3d::Identity() - K * H) * Sigma_;
}

void EKFLocalization::publishPose() {
  // ROS-Nachricht erstellen
  geometry_msgs::PoseWithCovarianceStamped out;
  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "map";

  out.pose.pose.position.x = mu_(0);
  out.pose.pose.position.y = mu_(1);
  tf2::Quaternion q;
  q.setRPY(0, 0, mu_(2));
  out.pose.pose.orientation = tf2::toMsg(q);

  // 6×6-Covariance (nur obere 3×3 füllen)
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      out.pose.covariance[i * 6 + j] = (i < 3 && j < 3) ? Sigma_(i, j) : 0;

  pose_pub_.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_node");
  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");
  EKFLocalization ekf(nh);
  ros::spin();
  return 0;
}
