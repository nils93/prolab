// landmark_mapper.h

#pragma once

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/package.h>  // 🔧 für ros::package::getPath()
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>                            // für tf2::getYaw
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind/bind.hpp> 
#include <vector>
#include <cmath>
#include <random>          // Für std::default_random_engine & std::uniform_int_distribution
#include <fstream>         // Für std::ofstream
#include <string>

// Eigen für lineare Algebra
#include <Eigen/Dense>

struct Landmark {
  double x, y;
  double rho_or_radius;
  double theta;
  int type; // 0 = Linie, 1 = Kreis
  std::array<int, 3> rgb; // [r, g, b]
};

class LandmarkMapper {
public:
  LandmarkMapper(ros::NodeHandle& nh);
  void finalizeLandmarks();

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::vector<Landmark> raw_landmarks_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void detectHoughLines(const std::vector<Eigen::Vector2d>& points);
  void detectCircle(const std::vector<Eigen::Vector2d>& points);
  void saveLandmarks();
  std::array<int, 3> getSimulatedColor(double x, double y);
};
