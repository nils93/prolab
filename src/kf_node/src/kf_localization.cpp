#include "kf_node/kf_localization.h"

KFLocalization::KFLocalization(ros::NodeHandle& nh)
: 
  odom_sub_(nh, "/odom", 1),
  imu_sub_(nh,  "/imu",  1),
  sync_(SyncPolicy(10), odom_sub_, imu_sub_)
{
  // Filter-Initialisierung
  x_.setZero();
  P_ = Eigen::Matrix3d::Identity() * 0.1;     // Anfangs-Un­sicherheit
  F_ = Eigen::Matrix3d::Identity();           
  Q_ = Eigen::Matrix3d::Identity() * 1e-3;    // Prozessrauschen
  H_ = Eigen::Matrix3d::Identity();           // z = [x; y; ωₓ] direkt
  R_ = Eigen::Matrix3d::Identity() * 1e-2;    // Messrauschen

  last_time_ = ros::Time::now();

  // synchronisierten Callback registrieren
  sync_.registerCallback(
    boost::bind(&KFLocalization::kfCallback, this, _1, _2)
  );

  // Publisher für gefilterte Pose
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    "kf_pose", 10
  );
}

void KFLocalization::kfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu) {
  // 1) Prediction
  predict(odom);

  // 2) Messvektor: x, y und tatsächlicher IMU-Yaw
  tf2::Quaternion q;
  tf2::fromMsg(imu->orientation, q);
  double imu_yaw = tf2::getYaw(q);

  Eigen::Vector3d z;
  z << odom->pose.pose.position.x,
       odom->pose.pose.position.y,
       imu_yaw;

  // 3) Correction
  update(z);

  // 4) Publish
  publishPose();
}


void KFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
  ros::Time now = odom->header.stamp;
  double dt = (now - last_time_).toSec();
  last_time_ = now;

  double v = odom->twist.twist.linear.x;
  double w = odom->twist.twist.angular.z;
  double theta = x_(2);

  // Zustandsvorhersage
  x_(0) += v * dt * cos(theta);
  x_(1) += v * dt * sin(theta);
  x_(2) += w * dt;

  // lineare Jacobi-Matrix
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0,2) = -v * dt * sin(theta);
  F(1,2) =  v * dt * cos(theta);

  P_ = F * P_ * F.transpose() + Q_;
}

void KFLocalization::update(const Eigen::Vector3d& z) {
  Eigen::Vector3d y;
  y(0) = z(0) - x_(0);
  y(1) = z(1) - x_(1);
  double dtheta = z(2) - x_(2);
  y(2) = std::atan2(std::sin(dtheta), std::cos(dtheta));

  Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix3d K = P_ * H_.transpose() * S.inverse();
  x_ += K * y;
  P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
}

void KFLocalization::publishPose() {
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
