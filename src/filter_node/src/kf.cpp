#include "kf_node/kf_localization.h"

KFLocalization::KFLocalization(ros::NodeHandle& nh)
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
           yaw_init,
           0.0;  // Startgeschwindigkeit auf 0 setzen
    last_time_ = t.header.stamp;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Initial TF fehlgeschlagen: %s", ex.what());
    // Fallback auf 0,0,0
    mu_.setZero();
    last_time_ = ros::Time::now();
  }
// ### Ende Initialisierung ###

  // Filter-Initialisierung
  Sigma_ = Eigen::Matrix4d::Identity() * 0.1;     // Anfangs-Un­sicherheit
  A_ = Eigen::Matrix4d::Identity();           // State Transition Matrix
  R_ = Eigen::Matrix4d::Identity() * 1e-3;    // Prozessrauschen (Process noise)
  // Messmatrix H und R
  C_.setZero();
  C_(0,3) = 1;  // v
  C_(1,2) = 1;  // θ
  Q_ = Eigen::Matrix2d::Identity() * 1e-2;    // Messrauschen

  // synchronisierten Callback registrieren
  sync_.registerCallback(
    boost::bind(&KFLocalization::kfCallback, this, _1, _2)
  );

  // Publisher für gefilterte Pose
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("kf_pose", 10);
}

void KFLocalization::kfCallback(
  const nav_msgs::Odometry::ConstPtr& odom,
  const sensor_msgs::Imu::ConstPtr& imu)
{
  // 1. Prädiktion
  predict(odom);

  // 2. Messung extrahieren (z = [v; θ])
  double v_meas = odom->twist.twist.linear.x;
  double theta_meas = tf2::getYaw(imu->orientation);
  Eigen::Vector2d z;
  z << v_meas, theta_meas;

  // 3. Korrektur
  update(z);

  // 4. Publikation
  publishPose();
}

void KFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
  // Zeitdifferenz
  double dt = (odom->header.stamp - last_time_).toSec();
  last_time_ = odom->header.stamp;

  double v = odom->twist.twist.linear.x;
  double theta = mu_(2);

  // Zustand vorhersagen: μ̄ₜ = A μₜ₋₁ + B uₜ
  mu_(0) += v * dt * cos(theta);  // x
  mu_(1) += v * dt * sin(theta);  // y

  // A aktualisieren
  A_.setIdentity();
  A_(0,2) = -v * dt * sin(theta);
  A_(0,3) =  dt * cos(theta);
  A_(1,2) =  v * dt * cos(theta);
  A_(1,3) =  dt * sin(theta);

  // Kovarianz vorhersagen: Σ̄ₜ = A Σₜ₋₁ Aᵀ + R
  Sigma_ = A_ * Sigma_ * A_.transpose() + R_;
}

void KFLocalization::update(const Eigen::Vector2d& z) {
  // Innovation: y = z - C * mu
  Eigen::Vector2d y = z - C_ * mu_;

  // Innovationskovarianz: S = C * Sigma * Cᵀ + Q
  Eigen::Matrix2d S = C_ * Sigma_ * C_.transpose() + Q_;

  // Kalman-Gain: K = Sigma * Cᵀ * S⁻¹
  Eigen::Matrix<double, 4, 2> K = Sigma_ * C_.transpose() * S.inverse();

  // Zustands-Update: mu = mu + K * y
  mu_ = mu_ + K * y;

  // Kovarianz-Update: Sigma = (I - K * C) * Sigma
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  Sigma_ = (I - K * C_) * Sigma_;
}

void KFLocalization::publishPose() {
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
  ros::init(argc, argv, "kf_node");
  // Erzeuge NodeHandle im privaten Namensraum (~)
  ros::NodeHandle nh("~");
  KFLocalization kf(nh);
  ros::spin();
  return 0;
}