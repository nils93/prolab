#ifndef KF_NODE_KF_LOCALIZATION_H
#define KF_NODE_KF_LOCALIZATION_H

// ROS
#include <ros/ros.h>
#include <ros/topic.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>                            // für tf2::getYaw
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind/bind.hpp> 
#include <cmath>

// Eigen für lineare Algebra
#include <Eigen/Dense>

class KFLocalization {
public:
  KFLocalization(ros::NodeHandle& nh);

private:
  // KF-Zustand und Kovarianzmatrizen
  Eigen::Vector4d x_;           // [x, y, θ, v]
  Eigen::Matrix4d P_;           // State-Cov
  Eigen::Matrix4d F_;           // Systemmatrix
  Eigen::Matrix4d Q_;           // Prozessrauschen
  Eigen::Matrix<double,2,4> H_; // Messmatrix [v_meas; θ_meas]
  Eigen::Matrix2d R_;           // Messrauschen

  // ROS Subscriber und Publisher
  ros::Publisher pose_pub_; // veröffentlichte geschätzte Pose
  ros::Time last_time_;


  // Synchronisierter Callback für Odom + IMU
  void kfCallback(
    const nav_msgs::Odometry::ConstPtr& odom,
    const sensor_msgs::Imu::ConstPtr& imu);

  // KF-Schritte
  void predict(const nav_msgs::Odometry::ConstPtr& odom); // Bewegungsvorhersage
  void update(const Eigen::Vector2d& z);           // Korrektur mit Messvektor
  void publishPose();                          // Ausgabe der geschätzten Pose als ROS-Topic

  // Message-Filters für Odom + IMU
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::Imu>    imu_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry,
    sensor_msgs::Imu
  > SyncPolicy2;
  message_filters::Synchronizer<SyncPolicy2> sync2_;
};

#endif  // KF_NODE_KF_LOCALIZATION_H