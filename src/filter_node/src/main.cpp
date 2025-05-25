#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "kf.h"
#include "ekf.h"
#include "pf.h"

ros::Publisher pub_kf_filter;
ros::Publisher pub_ekf_filter;
ros::Publisher pub_pf_filter;

int main(int argc, char** argv) {
  ros::init(argc, argv, "filter_node");
  ros::NodeHandle nh;

  // Publisher
  pub_kf_filter = nh.advertise<nav_msgs::Odometry>("/kf_filter/odom", 10);
  pub_ekf_filter = nh.advertise<nav_msgs::Odometry>("/ekf_filter/odom", 10);
  pub_pf_filter = nh.advertise<nav_msgs::Odometry>("/pf_filter/odom", 10);

  // Initialisierung EKF
  x_ekf = Eigen::VectorXd(3);
  x_ekf << 0.0, 0.0, 0.0;
  P_ekf = Eigen::MatrixXd::Identity(3,3) * 0.1;
  Q_ekf = Eigen::MatrixXd::Identity(3,3) * 0.01;
  R_ekf = Eigen::MatrixXd::Identity(3,3) * 0.05;

  // Initialisierung KF
  x_kf = Eigen::VectorXd::Zero(3);
  P_kf = Eigen::MatrixXd::Identity(3,3) * 0.1;
  Q_kf = Eigen::MatrixXd::Identity(3,3) * 0.01;
  R_kf = Eigen::MatrixXd::Identity(3,3) * 0.05;
  H_kf = Eigen::MatrixXd::Identity(3,3);
  F_kf = Eigen::MatrixXd::Identity(3,3);

  // Initialisierung PF
  particles.clear();
  for (int i = 0; i < 100; ++i) {  // z.B. 100 Partikel
    Particle p;
    p.state = Eigen::VectorXd::Zero(3);
    p.weight = 1.0 / 100.0;
    particles.push_back(p);
  }


  // Synchronisierte Subscriber
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 10);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu", 10);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, imu_sub);
  sync.registerCallback(kfCallback);
  sync.registerCallback(ekfCallback);
  sync.registerCallback(pfCallback);

  ros::spin();
  return 0;
}
