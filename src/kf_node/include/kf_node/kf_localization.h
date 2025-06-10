#ifndef KF_NODE_KF_LOCALIZATION_H
#define KF_NODE_KF_LOCALIZATION_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // für fromMsg
#include <tf2/utils.h>                            // für tf2::getYaw
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Eigen für lineare Algebra
#include <Eigen/Dense>

class KFLocalization {
public:
  KFLocalization(ros::NodeHandle& nh);

private:
  // ROS Subscriber und Publisher
  ros::Publisher pose_pub_; // veröffentlichte geschätzte Pose

  // KF-Zustand und Kovarianzmatrizen
  Eigen::Vector3d x_;     // [x, y, θ]
  Eigen::Matrix3d P_;     // State-Cov
  Eigen::Matrix3d F_;     // Systemmatrix
  Eigen::Matrix3d Q_;     // Prozessrauschen
  Eigen::Matrix3d H_;     // Messmatrix
  Eigen::Matrix3d R_;     // Messrauschen

  ros::Time last_time_;


  // Synchronisierter Callback für Odom + IMU
  void kfCallback(const nav_msgs::OdometryConstPtr& odom,
                  const sensor_msgs::ImuConstPtr& imu);

  // KF-Schritte
  void predict(const nav_msgs::OdometryConstPtr& odom); // Bewegungsvorhersage
  void update(const Eigen::Vector3d& z);           // Korrektur mit Messvektor
  void publishPose();                          // Ausgabe der geschätzten Pose als ROS-Topic

  // Message-Filters für Odom + IMU
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::Imu>    imu_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
};

#endif  // KF_NODE_KF_LOCALIZATION_H