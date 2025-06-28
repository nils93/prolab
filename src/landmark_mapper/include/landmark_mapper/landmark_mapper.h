// landmark_mapper.h
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <random>

class ColorSampler;  // Forward declaration

class LandmarkMapper {
public:
  LandmarkMapper(ros::NodeHandle& nh);
  void setColorSampler(ColorSampler* cs);  // <-- NEU

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void detectHoughLines(const std::vector<Eigen::Vector2d>& points);
  void detectCircle(const std::vector<Eigen::Vector2d>& points);
  void publishLandmark(double x, double y, double rho_or_radius, double theta, int type);

  ros::NodeHandle nh_;
  ros::Subscriber sub_scan_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::vector<Eigen::Vector2d> known_positions_;
  ColorSampler* cs_ = nullptr;  // <-- NEU
};
