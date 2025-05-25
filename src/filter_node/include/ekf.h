#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Globale Variablen für den erweiterten Kalman-Filter
extern Eigen::VectorXd x_ekf;
extern Eigen::MatrixXd P_ekf;
extern Eigen::MatrixXd Q_ekf;
extern Eigen::MatrixXd R_ekf;




// EKF-spezifische Funktionen
Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd& x);
Eigen::MatrixXd computeJacobianH(const Eigen::VectorXd& x);
Eigen::VectorXd nonlinearStateTransition(const Eigen::VectorXd& x, double v, double omega);
Eigen::VectorXd nonlinearMeasurementFunction(const Eigen::VectorXd& x);

void ekfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);

#endif  // EKF_H
