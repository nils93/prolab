#ifndef EKF_NODE_EKF_LOCALIZATION_H
#define EKF_NODE_EKF_LOCALIZATION_H

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

class EKFLocalization {
public:
  EKFLocalization(ros::NodeHandle& nh);

private:
  // EKF-Zustand
  Eigen::Vector3d mu_;           // Zustand [x, y, theta]
  Eigen::Matrix3d Sigma_;        // Kovarianzmatrix
  Eigen::Matrix3d G_;            // Jacobi-Matrix der Bewegung
  Eigen::Matrix3d R_;            // Prozessrauschen
  Eigen::Matrix<double, 1, 1> Q_;  // Messrauschen für θ (Yaw)

  // ROS Subscriber und Publisher
  ros::Publisher pose_pub_; // veröffentlichte geschätzte Pose
  ros::Time last_time_;


  // Synchronisierter Callback für Odom + IMU
  void ekfCallback(
    const nav_msgs::Odometry::ConstPtr& odom,
    const sensor_msgs::Imu::ConstPtr& imu);

  // KF-Schritte
  void predict(const nav_msgs::Odometry::ConstPtr& odom); // Bewegungsvorhersage
  void update(const Eigen::VectorXd& z); // Korrektur mit Messvektor
  void publishPose(); // Ausgabe der geschätzten Pose als ROS-Topic

  // Message-Filters für Odom + IMU
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::Imu>    imu_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry,
    sensor_msgs::Imu
  > SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
};

#endif  // EKF_NODE_EKF_LOCALIZATION_H