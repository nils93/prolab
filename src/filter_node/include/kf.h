#ifndef KF_H
#define KF_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Globale Variablen für den Kalman-Filter
extern Eigen::VectorXd x_kf;
extern Eigen::MatrixXd P_kf;
extern Eigen::MatrixXd F_kf;
extern Eigen::MatrixXd Q_kf;
extern Eigen::MatrixXd H_kf;
extern Eigen::MatrixXd R_kf;

// Callback für ROS-Subscriber
void kfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);

#endif  // KF_H
