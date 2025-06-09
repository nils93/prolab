#include "ekf_node/ekf_localization.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>
#include <cmath>

EKFLocalization::EKFLocalization(ros::NodeHandle& nh) {
  // Landmarken laden
  XmlRpc::XmlRpcValue lm_list;
  nh.getParam("landmarks", lm_list);
  for (int i = 0; i < lm_list.size(); ++i) {
    Landmark L;
    L.id = (int)lm_list[i]["id"];
    L.x  = (double)lm_list[i]["x"];
    L.y  = (double)lm_list[i]["y"];
    LM_.push_back(L);
  }

  // Initialisierung
  x_.setZero();
  P_.setIdentity();  Q_ = Eigen::Matrix3d::Identity() * 1e-3;
  R_ = Eigen::Matrix2d::Identity() * 1e-2;
  last_time_ = ros::Time::now();

  // Subs & Pub
  // odom_sub_ = nh.subscribe("odom", 10, &EKFLocalization::odomCallback, this);
  odom_sub_ = nh.subscribe("/odom", 10, &EKFLocalization::odomCallback, this); 
  lm_sub_   = nh.subscribe("landmark_obs", 10, &EKFLocalization::lmCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
}

void EKFLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  predict(msg);
  publishPose();
}

void EKFLocalization::lmCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // hier id und Messvektor extrahieren (z.B. aus msg->pose.covariance oder eigener Struktur)
  int id = msg->header.frame_id.empty() ? 0 : std::stoi(msg->header.frame_id);
  Eigen::Vector2d z(msg->pose.pose.position.x, msg->pose.pose.position.y);
  update(id, z);
}

void EKFLocalization::predict(const nav_msgs::Odometry::ConstPtr& odom) {
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

  // Jacobi F
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0,2) = -v * dt * sin(theta);
  F(1,2) =  v * dt * cos(theta);

  P_ = F * P_ * F.transpose() + Q_;
}

void EKFLocalization::update(int id, const Eigen::Vector2d& z) {
  // Landmark suchen
  auto it = std::find_if(LM_.begin(), LM_.end(),
    [&](const Landmark& L){ return L.id == id; });
  if (it == LM_.end()) return;

  double dx = it->x - x_(0), dy = it->y - x_(1);
  double q  = dx*dx + dy*dy;
  double r_pred = std::sqrt(q), b_pred = std::atan2(dy,dx) - x_(2);

  Eigen::Vector2d z_hat(r_pred, b_pred);
  Eigen::Vector2d y = z - z_hat;

  Eigen::Matrix<double,2,3> H;
  H << -dx/r_pred, -dy/r_pred, 0,
        dy/q,      -dx/q,     -1;

  Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
  Eigen::Matrix<double,3,2> K = P_ * H.transpose() * S.inverse();

  x_ += K * y;
  P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;

  publishPose();
}

void EKFLocalization::publishPose() {
  geometry_msgs::PoseWithCovarianceStamped out;
  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "map";
  out.pose.pose.position.x = x_(0);
  out.pose.pose.position.y = x_(1);
  tf2::Quaternion q; q.setRPY(0,0,x_(2));
  out.pose.pose.orientation = tf2::toMsg(q);

  // 6×6-Covariance aus 3×3 P_
  for(int i=0;i<6;i++) for(int j=0;j<6;j++)
    out.pose.covariance[i*6+j] = (i<3&&j<3) ? P_(i,j) : 0;

  pose_pub_.publish(out);
}
