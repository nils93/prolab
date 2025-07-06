#ifndef PF_NODE_PF_LOCALIZATION_H
#define PF_NODE_PF_LOCALIZATION_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Dense>
#include <random>
#include <cmath>
#include "angles/angles.h"

struct Particle {
  double x;
  double y;
  double theta;
  double weight;
};

class PFLocalization {
public:
  PFLocalization(ros::NodeHandle& nh);

private:
  std::vector<Particle> particles_;
  int num_particles_ = 100;
  ros::Publisher pose_pub_;
  ros::Publisher particle_pub_;
  ros::Time last_time_;
  ros::Subscriber laser_sub_;
  ros::Subscriber map_sub_;
  nav_msgs::OccupancyGrid map_;
  bool map_received_ = false;

  // Rauschparameter
  double alpha1_, alpha2_, alpha3_, alpha4_;
  std::default_random_engine rng_;

  // ROS-Callback
  void pfCallback(const nav_msgs::Odometry::ConstPtr& odom,
                  const sensor_msgs::Imu::ConstPtr& imu);

  // Partikelfilter-Schritte
  void motionUpdate(double v, double w, double dt);
  void resample(); 
  void publishPose();  
  void publishParticles();  
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);              
  void measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan);         

  // Message Filter
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
};

#endif  // PF_NODE_PF_LOCALIZATION_H