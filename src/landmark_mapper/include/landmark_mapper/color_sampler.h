// color_sampler.h

#pragma once

#include <array>
#include <vector>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <random>

class ColorSampler {
public:
  ColorSampler();
  ColorSampler(ros::NodeHandle& nh);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void addLandmark(double x, double y, double rho, double theta, int type);

  ros::Time last_add_time_;  // MARKIERT: Zeitstempel des letzten addLandmark()
  void saveToYAML(const std::string& path = "");

private:
  bool sampleColorAt(double x, double y, std::array<int, 3>& rgb_out);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  cv::Mat cv_img_;
  ros::Time last_image_time_;
  std::vector<std::tuple<double, double, double, double, int, std::array<int, 3>>> landmarks_;
  
};
