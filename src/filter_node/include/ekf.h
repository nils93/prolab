#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Zustands- und Rauschgrößen
extern Eigen::VectorXd x_ekf;
extern Eigen::MatrixXd P_ekf;
extern Eigen::MatrixXd Q_ekf;
extern Eigen::MatrixXd R_ekf;
extern std::vector<Eigen::Vector2d> landmark_map;
extern ros::Publisher pub_ekf_filter;

extern Eigen::Vector2d latest_scan_measurement;
extern bool got_scan;


// EKF-Callback
void ekfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);

void loadLandmarks(const std::string& filepath);

void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);



#endif  // EKF_H
