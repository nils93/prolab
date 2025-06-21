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
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Initial TF fehlgeschlagen: %s", ex.what());
    // Fallback auf 0,0,0
    x_.setZero();
    last_time_ = ros::Time::now();
  }
  // Körperpose in map-Frame in x_ übernehmen
  double yaw_init = tf2::getYaw(t.transform.rotation);
  x_ << t.transform.translation.x,
        t.transform.translation.y,
        yaw_init,
        0.0; // Startgeschwindigkeit auf 0 setzen
  last_time_ = t.header.stamp;
  // ### Ende Initialisierung ###

  // Filter-Initialisierung
  P_ = Eigen::Matrix4d::Identity() * 0.1;   // Anfangs-Un­sicherheit
  F_ = Eigen::Matrix4d::Identity();           
  Q_ = Eigen::Matrix4d::Identity() * 1e-3;  // Prozessrauschen
  // Messmatrix H und R
  H_.setZero();
  H_(0,3) = 1;  // v
  H_(1,2) = 1;  // θ
  R_ = Eigen::Matrix2d::Identity() * 1e-2;  // Messrauschen

  // Callback registrieren
  sync_.registerCallback(
    boost::bind(&EKFLocalization::ekfCallback, this, _1, _2)
  );

  // Publisher
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
}

void EKFLocalization::ekfCallback(
  const nav_msgs::Odometry::ConstPtr& odom,
  const sensor_msgs::Imu::ConstPtr& imu)
{
  // 1) Prediction
  predict(odom);

  // 2) Messvektor [v_meas; ω_meas]
  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  Eigen::Vector2d z(v, w);

  // 3) Correction
  update(z);

  // 4) Publish
  publishPose();
}

void EKFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
  ros::Time now = odom->header.stamp;
  double dt = (now - last_time_).toSec();
  last_time_ = now;

  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  double theta = x_(2);

  x_(0) += v * dt * std::cos(theta);
  x_(1) += v * dt * std::sin(theta);
  x_(2) += w * dt;
  // x_(3) bleibt v

  F_.setIdentity();
  F_(0,2) = -v * dt * std::sin(theta);
  F_(0,3) =      dt * std::cos(theta);
  F_(1,2) =  v * dt * std::cos(theta);
  F_(1,3) =      dt * std::sin(theta);

  P_ = F_ * P_ * F_.transpose() + Q_;
}

// Messkorrektur
void EKFLocalization::update(const Eigen::Vector2d& z) {
  Eigen::Vector2d y = z - H_ * x_;
  // Winkel-Rate direkt, keine Normierung nötig

  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<double,4,2> K = P_ * H_.transpose() * S.inverse();

  x_ += K * y;
  P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
}

void EKFLocalization::publishPose() {
  // ROS-Nachricht erstellen
  geometry_msgs::PoseWithCovarianceStamped out;
  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "map";

  out.pose.pose.position.x = x_(0);
  out.pose.pose.position.y = x_(1);
  tf2::Quaternion q;
  q.setRPY(0, 0, x_(2));
  out.pose.pose.orientation = tf2::toMsg(q);

  // 6×6-Covariance (nur obere 3×3 füllen)
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      out.pose.covariance[i * 6 + j] = (i < 3 && j < 3) ? P_(i, j) : 0;

  pose_pub_.publish(out);
}