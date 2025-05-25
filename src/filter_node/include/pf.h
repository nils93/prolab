#ifndef PF_H
#define PF_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <random>


// Struktur für Partikel
struct Particle {
  Eigen::VectorXd state;
  double weight;
};

// Globale Partikelliste
extern std::vector<Particle> particles;

// Funktionen für Partikelfilter
Eigen::VectorXd motionModel(const Eigen::VectorXd& state, const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);
double measurementLikelihood(const Eigen::VectorXd& state, const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);
std::vector<Particle> resampleParticles(const std::vector<Particle>& particles);

// Callback für ROS-Subscriber
void pfCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu);

#endif  // PF_H
